from rclpy.impl import rcutils_logger
import serial
import time

logger = rcutils_logger.RcutilsLogger(name="Motor_driver")

class MotorDriver(object):

  def __init__(self, 
               port="/dev/ttyUSB0", 
               baud_rate=115200, 
               rotations_per_metre=10,
               swap_motors=False,
               inverse_left_motor=False,
               inverse_right_motor=False):
    self.serial = serial.Serial(port, baud_rate)
    self.rotations_per_metre = rotations_per_metre
    self.swap_motors = swap_motors
    self.inverse_left_motor = -1 if inverse_left_motor else 1
    self.inverse_right_motor = -1 if inverse_right_motor else 1

  def linear_to_rps(self, left, right):
    m1 = left * self.rotations_per_metre
    m2 = right * self.rotations_per_metre
    
    if self.swap_motors:
      m1, m2 = m2, m1

    m1 = int(m1 * self.inverse_left_motor)
    m2 = int(m2 * self.inverse_right_motor)
    return m1, m2
  
  def rps_to_linear(self, m1, m2):
    m1 = int(m1 * self.inverse_left_motor)
    m2 = int(m2 * self.inverse_right_motor)

    if self.swap_motors:
      m1, m2 = m2, m1
    
    left = m1 / self.rotations_per_metre
    right = m2 / self.rotations_per_metre
    return left, right

  def get_response(self):
    return self.serial.read_until(expected=b"\r").decode("ascii")

  def get_relative_encoders(self):
    self.send("?S")
    success = self.get_response()
    values = self.get_response()
    
    s = values.split(':')
    if len(s) <= 1:
      return

    # encoders report 3x speed
    m1 = int(s[0][2:])/3.0
    m2 = int(s[1][:-1])/3.0

    left, right = self.rps_to_linear(m1,m2)

    return left, right

  def send(self, message):
    packet = message.encode("ascii") + b'\r'
    self.serial.write(packet)
    

  def send_velocity(self, left, right):
    m1, m2 = self.linear_to_rps(left, right)
    self.send("!m %d %d" %(m1,m2))
    # clear buffer of responses
    self.get_response()
    self.get_response()

  def close(self):
    self.serial.close()

if __name__ == "__main__":
  conan = MotorDriver()