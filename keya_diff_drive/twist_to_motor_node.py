#!/usr/bin/env python3
import asyncio
from functools import cached_property
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from keya_diff_drive.motor_driver import MotorDriver

import rclpy

from rclpy.node import Node
import numpy

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
      ('rate', 50),
      ('ticks_per_target', 2),
      ('twist_topic', '/cmd_vel'),
      ('publish_motors', False),
      ('serial_port', '/dev/ttyUSB0'),
      ('baud_rate', 115200),

      ('wheel_separation', 0.43),
      ('wheel_radius',     0.215)
      ('rotations_per_metre', 10),
      ('swap_motors', False),
      ('inverse_left_motor', False),
      ('inverse_right_motor', False)
    ]
    self.declare_parameters('', parameters=params)
    self._wheel_separation = self.get_parameter('wheel_separation').value
    self._wheel_radius = self.get_parameter('wheel_radius').value
    self._wheel_circum = 2 * numpy.pi * self._wheel_radius

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