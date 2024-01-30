#!/usr/bin/env python3
from functools import cached_property
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, IntegerRange
from keya_diff_drive.motor_driver import MotorDriver, SerialSettings, MotorDriverSettings
import traceback

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import rclpy
from rclpy.constants import S_TO_NS

from rclpy.node import Node
import numpy as np

class TwistToMotors(Node):
  def parameters_callback(self, params):
    motor_settings = self.motor_driver
    for param in params:
      match param.name:
        case 'base_width':
          self._base_width = param.value
        case 'publish_motors':
          self._publish_motors = param.value
          
        case 'rotations_per_metre':
          motor_settings.rotations_per_metre = param.value
        case 'swap_motors':
          motor_settings.swap_motors = param.value
        case 'inverse_left_motor':
          motor_settings.inverse_left_motor = param.value
        case 'inverse_right_motor':
          motor_settings.inverse_right_motor = param.value

        case 'acceleration':
          self.motor_driver.set_acceleration(param.value)
        case 'deceleration':
          self.motor_driver.set_deceleration(param.value)
    return SetParametersResult(successful=True)

  def __init__(self):
    super().__init__('twist_to_motors')

    params = [
      ('odometry_rate', 50),
      ('twist_topic', '/matilda_velocity_controller/cmd_vel_unstamped'),
      ('odom_topic', '/wheel_odometry'),

      ('odom_frame', 'odom'),
      ('base_frame', 'wheel_odom_link'),

      ('publish_odom', True),
      ('publish_motors', False),
      ('debug', False),

      ('serial_port', '/dev/ttyUSB0'),
      ('baud_rate', 115200),
      ('serial_timeout', 0.1),
      ('command_timeout', 1),

      ('wheel_separation', 0.43),
      ('wheel_radius',     0.215),
      ('rotations_per_metre', 10),
      ('max_rpm', 3000),

      ('swap_motors', False),
      ('inverse_left_motor', False),
      ('inverse_right_motor', False),
      ('acceleration', 10000, ParameterDescriptor(
        name='acceleration', 
        type=2, 
        description='Acceleration', 
        integer_range=[IntegerRange(from_value=100, to_value=32000, step=100)])
        ),
      ('deceleration', 10000, ParameterDescriptor(
        name='deceleration', 
        type=2, 
        description='deceleration', 
        integer_range=[IntegerRange(from_value=100, to_value=32000, step=100)])
        ),
    ]
    self.declare_parameters('', parameters=params)
    
    self._command_timeout = self.get_parameter('command_timeout').value
    self._wheel_separation = self.get_parameter('wheel_separation').value
    self._wheel_radius = self.get_parameter('wheel_radius').value
    self._wheel_circum = 2 * np.pi * self._wheel_radius

    self._odom_frame = self.get_parameter('odom_frame').value
    self._base_frame = self.get_parameter('base_frame').value

    self._twist_topic = self.get_parameter('twist_topic').value
    self._odom_topic = self.get_parameter('odom_topic').value

    self._publish_odom = self.get_parameter('publish_odom').value
    self._publish_motors = self.get_parameter('publish_motors').value
    self.debug = self.get_parameter('debug').value

    self.add_on_set_parameters_callback(self.parameters_callback)
  
    self.linear_in = 0.0
    self.angular_in = 0.0

    serial_settings = SerialSettings(
      port = self.get_parameter('serial_port').value,
      baud_rate = self.get_parameter('baud_rate').value,
      timeout = self.get_parameter('serial_timeout').value)
    motor_settings = MotorDriverSettings(
      self.get_parameter('rotations_per_metre').value,
      self.get_parameter('max_rpm').value,
      self.get_parameter('swap_motors').value,
      self.get_parameter('inverse_left_motor').value,
      self.get_parameter('inverse_right_motor').value,
    )
    self.motor_driver = MotorDriver(serial_settings, motor_settings)

    accel = self.get_parameter('acceleration').value
    self.motor_driver.set_acceleration(accel)

    decel = self.get_parameter('deceleration').value
    self.motor_driver.set_deceleration(decel)

    self.odom_msg = Odometry()
    self.odom_msg.header.frame_id = self._odom_frame
    self.odom_msg.child_frame_id = self._base_frame
    self.static_cov = 1e-3
    self.linear_cov = 0.02
    self.angular_cov = 0.04

    odometry_period = 1 / self.get_parameter('odometry_rate').value
    self.last_odom_time = self.get_clock().now()
    self.last_command_time = self.get_clock().now()
    self.loop_timer = self.create_timer(odometry_period, self.loop)

    self.twist_sub = self.twist_subscriber()


  @cached_property
  def left_wheel_publisher(self):
    return self.create_publisher(Float32, 'lwheel_vtarget', 10)
  

  @cached_property
  def right_wheel_publisher(self):
    return self.create_publisher(Float32, 'rwheel_vtarget', 10)
  

  @cached_property
  def wheel_odometry_publisher(self):
    return self.create_publisher(Odometry, self._odom_topic, 10)

  def publish_odom(self, current_time, forward: float, ccw: float):
    self.odom_msg.header.stamp = current_time.to_msg()
    self.odom_msg.pose.pose.position.x += forward * ( current_time - self.last_odom_time ).nanoseconds/1e9
    self.odom_msg.pose.pose.position.z += ccw * ( current_time - self.last_odom_time ).nanoseconds/1e9
    self.odom_msg.twist.twist.linear.x = forward
    self.odom_msg.twist.twist.angular.z = ccw
    cov = np.array(self.odom_msg.twist.covariance).reshape(6, 6)
    cov[0, 0] = (forward * self.linear_cov + self.static_cov) ** 2
    cov[1, 1] = self.static_cov ** 2
    cov[2, 2] = self.static_cov ** 2
    cov[5, 5] = (ccw * self.angular_cov) ** 2
    self.odom_msg.twist.covariance = cov.flatten().tolist()
    self.wheel_odometry_publisher.publish(self.odom_msg)
    self.last_odom_time = current_time


  def loop(self):
    try:

      # Send Input
      current_time = self.get_clock().now()
      sec_since_last_command = ( current_time - self.last_command_time ).nanoseconds / 1e9
      left_in, right_in = self.twist2diff(self.linear_in , self.angular_in)

      if sec_since_last_command < self._command_timeout:
        self.motor_driver.send_velocity(left_in, right_in)

      if self.debug:
        self.get_logger().warn('------------------------------------------------------')
        self.get_logger().warn(f'Forward in = {self.linear_in }')
        self.get_logger().warn(f'Angular in = {self.angular_in}')
        self.get_logger().warn(f'LEFT IN = {left_in}')
        self.get_logger().warn(f'RIGHT IN = {right_in}')


      # Read Encoder Output
      response = self.motor_driver.get_relative_encoders()
      left_out, right_out = response
      linear_out, angular_out = self.diff2twist(left_out, right_out)

      if self._publish_motors: 
        self.left_wheel_publisher.publish(Float32(data=left_out))
        self.right_wheel_publisher.publish(Float32(data=right_out))

      if self._publish_odom:
          self.publish_odom(current_time, linear_out, angular_out)
      
      # Read Motor Status
      # state = self.motor_driver.get_state_status()
      # motor1, motor2 = self.motor_driver.get_fault_status()

      if self.debug:
        self.get_logger().warn(f'LEFT OUT = {left_out}')
        self.get_logger().warn(f'RIGHT OUT = {right_out}')
        self.get_logger().warn(f'Forward out = {linear_out}')
        self.get_logger().warn(f'Angular out = {angular_out}')
        
    except Exception as e:
      self.get_logger().error(traceback.format_exc())  
      return


  def twist2diff(self, forward, ccw):
    angular_to_linear = ccw * (self._wheel_separation / 2.0) 
    left_linear_val  = float((forward - angular_to_linear) / self._wheel_circum)
    right_linear_val = float((forward + angular_to_linear) / self._wheel_circum)
    return left_linear_val, right_linear_val

  def diff2twist(self, left, right):
    forward = ((left+right) / 2) * self._wheel_circum
    ccw = ((right-left) / self._wheel_separation) * self._wheel_circum
    return forward, ccw


  def twist_subscriber(self):
    def update_target(msg: Twist):
      self.linear_in = msg.linear.x
      self.angular_in = msg.angular.z
      self.last_command_time = self.get_clock().now()
    return self.create_subscription(Twist, self._twist_topic, update_target, 10)
  
  def destroy_node(self):
    self.loop_timer.destroy()
    self.motor_driver.serial.close()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  TwistToMotors_node = TwistToMotors()
  rclpy.spin(TwistToMotors_node)
  TwistToMotors_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
