#!/usr/bin/env python3
from functools import cached_property
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from keya_diff_drive.motor_driver import MotorDriver
from transforms3d.euler import euler2quat

import rclpy

from rclpy.node import Node
import numpy as np

class TwistToMotors(Node):
  def parameters_callback(self, params):
    for param in params:
      match param.name:
        case 'base_width':
          self._base_width = param.value
        case 'publish_motors':
          self._publish_motors = param.value
        case 'rotations_per_metre':
          self.motor_driver.rotations_per_metre = param.value
        case 'swap_motors':
          self.motor_driver.swap_motors = param.value
        case 'inverse_left_motor':
          self.motor_driver.inverse_left_motor = param.value
        case 'inverse_right_motor':
          self.motor_driver.inverse_right_motor = param.value
    return SetParametersResult(successful=True)

  def __init__(self):
    super().__init__('twist_to_motors')

    params = [
      ('odometry_rate', 50),
      ('twist_topic', '/cmd_vel'),
      ('odom_topic', '/wheel_odometry'),

      ('odom_frame', 'odom'),
      ('base_frame', 'wheel_odom_link')

      ('publish_odom', True),
      ('publish_motors', False),

      ('serial_port', '/dev/ttyUSB0'),
      ('baud_rate', 115200),

      ('wheel_separation', 0.43),
      ('wheel_radius',     0.215),
      ('rotations_per_metre', 10),
      ('swap_motors', False),
      ('inverse_left_motor', False),
      ('inverse_right_motor', False)
    ]
    self.declare_parameters('', parameters=params)
    self._wheel_separation = self.get_parameter('wheel_separation').value
    self._wheel_radius = self.get_parameter('wheel_radius').value
    self._wheel_circum = 2 * np.pi * self._wheel_radius

    self._twist_topic = self.get_parameter('twist_topic').value
    self._odom_topic = self.get_parameter('odom_topic').value

    self._publish_odom = self.get_parameter('publish_odom').value
    self._publish_motors = self.get_parameter('publish_motors').value

    self._odom_covar_scale = 0.01

    self.add_on_set_parameters_callback(self.parameters_callback)
  
    self.left = 0.0
    self.right = 0.0
    self.motor_driver = MotorDriver(
      self.get_parameter('serial_port').value,
      self.get_parameter('baud_rate').value,
      self.get_parameter('rotations_per_metre').value,
      self.get_parameter('swap_motors').value,
      self.get_parameter('inverse_left_motor').value,
      self.get_parameter('inverse_right_motor').value
    )

    odometry_period = 1 / self.get_parameter('odometry_rate').value
    self.odom_timer = self.create_timer(odometry_period, self.publish_odometry)

    self.twist_sub = self.twist_subscriber()

    if self._publish_odom:
      self.old_pos_l = 0
      self.old_pos_r = 0
  
      # setup message
      self.odom_msg = Odometry()
      self.odom_msg.header.frame_id = self.get_parameter('odom_frame').value
      self.odom_msg.child_frame_id = self.get_parameter('base_frame').value
      self.odom_msg.pose.pose.position.x = 0.0
      self.odom_msg.pose.pose.position.y = 0.0
      self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
      self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
      self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
      self.odom_msg.pose.pose.orientation.z = 0.0
      self.odom_msg.pose.pose.orientation.w = 1.0
      self.odom_msg.twist.twist.linear.x = 0.0
      self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
      self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
      self.odom_msg.twist.twist.angular.x = 0.0 # or roll
      self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
      self.odom_msg.twist.twist.angular.z = 0.0
      
      # store current location to be updated. 
      self.x = 0.0
      self.y = 0.0
      self.theta = 0.0


  @cached_property
  def left_wheel_publisher(self):
    return self.create_publisher(Float32, 'lwheel_vtarget', 10)
  

  @cached_property
  def right_wheel_publisher(self):
    return self.create_publisher(Float32, 'rwheel_vtarget', 10)
  

  @cached_property
  def wheel_odometry_publisher(self):
    return self.create_publisher(String, self._odom_topic, 10)
  
  def publish_velocity(self, left, right):
    if self._publish_motors:
      self.left_wheel_publisher.publish(Float32(data=left))
      self.right_wheel_publisher.publish(Float32(data=right))

  def publish_odometry(self):
    if self._publish_odom:
      self.odom_msg.header.stamp = self.get_clock().now()

      message = self.motor_driver.get_encoders()
      
      s = message.split(':')
      if len(s) == 0:
        return
      left_diff = int(s[0][2:])
      right_diff = int(s[1])
      forward, ccw = self.diff2twist(left_diff, right_diff)
    
      self.odom_msg.twist.twist.linear.x = forward
      self.odom_msg.twist.twist.angular.z = ccw
  
      self.new_pos_l = left_diff
      self.new_pos_r = right_diff
      # Position
      delta_pos_l = self.new_pos_l - self.old_pos_l
      delta_pos_r = self.new_pos_r - self.old_pos_r
      
      self.old_pos_l = self.new_pos_l
      self.old_pos_r = self.new_pos_r
            
      # counts to metres
      delta_pos_l_m = delta_pos_l * self._wheel_circum
      delta_pos_r_m = delta_pos_r * self._wheel_circum
  
      # Distance travelled
      d = (delta_pos_l_m+delta_pos_r_m) / 2.0  # delta_ps
      th = (delta_pos_r_m-delta_pos_l_m) / self._wheel_separation # works for small angles
  
      xd = np.cos(th)*d
      yd = -np.sin(th)*d
  
      # Pose: updated from previous pose + position delta
      self.x += np.cos(self.theta)*xd - np.sin(self.theta)*yd
      self.y += np.sin(self.theta)*xd + np.cos(self.theta)*yd
      self.theta = (self.theta + th) % (2*np.pi)
      
      # fill odom message and publish
      
      self.odom_msg.pose.pose.position.x = self.x
      self.odom_msg.pose.pose.position.y = self.y
      q = euler2quat(0.0, 0.0, self.theta)
      self.odom_msg.pose.pose.orientation.z = q[2] # math.sin(self.theta)/2
      self.odom_msg.pose.pose.orientation.w = q[3] # math.cos(self.theta)/2
  
      self.odom_msg.twist.covariance[0]  = forward * self._odom_covar_scale
      self.odom_msg.twist.covariance[7]  = forward * self._odom_covar_scale
      self.odom_msg.twist.covariance[14] = forward * self._odom_covar_scale
      
      # ... and publish!
      self.wheel_odometry_publisher.publish(String(data=message))   
    

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
      dx = msg.linear.x
      dr = msg.angular.z
      self.left, self.right = self.twist2diff(dx, dr)
      self.motor_driver.send_velocity(self.left, self.right)
      self.publish_velocity(self.left, self.right)
    return self.create_subscription(Twist, self._twist_topic, update_target, 10)
  
  def destroy_node(self):
    self.odom_timer.destroy()
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