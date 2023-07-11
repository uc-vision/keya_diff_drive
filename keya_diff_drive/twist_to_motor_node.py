#!/usr/bin/env python3
import asyncio
from functools import cached_property
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from keya_diff_drive.motor_driver import MotorDriver

import rclpy

from rclpy.node import Node

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
    return SetParametersResult(successful=True)

  def __init__(self):
    super().__init__('twist_to_motors')

    params = [
      ('rate', 50),
      ('ticks_per_target', 2),
      ('base_width', 0.77),
      ('twist_topic', '/cmd_vel'),
      ('publish_motors', False),
      
      ('serial_port', '/dev/ttyUSB0'),
      ('baud_rate', 115200),
      ('rotations_per_metre', 10),
      ('swap_motors', False),
      ('inverse_left_motor', False),
      ('inverse_right_motor', False)
    ]
    self.declare_parameters('', parameters=params)
    self._base_width = self.get_parameter('base_width').value
    self._twist_topic = self.get_parameter('twist_topic').value
    self._publish_motors = self.get_parameter('publish_motors').value
    self._ticks_per_target = self.get_parameter('ticks_per_target').value
    self.add_on_set_parameters_callback(self.parameters_callback)
  
    self.left = 0
    self.right = 0
    self.motor_driver = MotorDriver(
      self.get_parameter('serial_port').value,
      self.get_parameter('baud_rate').value,
      self.get_parameter('rotations_per_metre').value,
      self.get_parameter('swap_motors').value,
      self.get_parameter('inverse_left_motor').value,
      self.get_parameter('inverse_right_motor').value
    )
    timer_period = 1 / self.get_parameter('rate').value
    self.current_ticks = self._ticks_per_target
    self.create_timer(timer_period, self.send_velocity)
    self.twist_sub = self.twist_subscriber()
  
  @cached_property
  def left_wheel_publisher(self):
    return self.create_publisher(Float32, 'lwheel_vtarget', 10)
  
  @cached_property
  def right_wheel_publisher(self):
    return self.create_publisher(Float32, 'rwheel_vtarget', 10)
  
  def publish_velocity(self, left, right):
    if self._publish_motors:
      self.left_wheel_publisher.publish(Float32(data=left))
      self.right_wheel_publisher.publish(Float32(data=right))

  def send_velocity(self):
    if self.current_ticks < self._ticks_per_target:
      self.motor_driver.send_velocity(self.left, self.right)
      self.publish_velocity(self.left, self.right)
      self.current_ticks += 1

  def twist_subscriber(self):
    def update_target(msg: Twist):
      dx = msg.linear.x
      dr = msg.angular.z
      self.right = 1.0 * dx + dr * self._base_width / 2
      self.left = 1.0 * dx - dr * self._base_width / 2
      self.current_ticks = 0
    return self.create_subscription(Twist, self._twist_topic, update_target, 10)

def main(args=None):
  rclpy.init(args=args)
  TwistToMotors_node = TwistToMotors()
  rclpy.spin(TwistToMotors_node)
  TwistToMotors_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()