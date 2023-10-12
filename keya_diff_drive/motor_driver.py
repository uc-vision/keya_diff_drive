from rclpy.impl import rcutils_logger
import serial
import time
from dataclasses import dataclass

logger = rcutils_logger.RcutilsLogger(name="Motor_driver")

@dataclass
class SerialSettings(object):
  port: str = "/dev/ttyUSB0"
  baud_rate: int = 115200
  timeout: float = 0.1

@dataclass
class MotorDriverSettings(object):
  rotations_per_metre: int = 10
  swap_motors: bool = False
  inverse_left_motor: bool = False
  inverse_right_motor: bool = False


class MotorDriver(object):

  def __init__(self, serial_settings: SerialSettings, motor_settings: MotorDriverSettings):
    self.serial = serial.Serial(
      serial_settings.port, 
      serial_settings.baud_rate, 
      timeout=serial_settings.timeout)
    self.motor_settings = motor_settings

  def format_rps(self, m1, m2):
    inverse_left_motor = -1 if self.motor_settings.inverse_left_motor else 1
    inverse_right_motor = -1 if self.motor_settings.inverse_right_motor else 1
    m1 = int(m1 * inverse_left_motor)
    m2 = int(m2 * inverse_right_motor)
    return m1, m2

  def linear_to_rps(self, left, right):
    m1 = left * self.motor_settings.rotations_per_metre
    m2 = right * self.motor_settings.rotations_per_metre
    
    if self.motor_settings.swap_motors:
      m1, m2 = m2, m1

    return self.format_rps(m1, m2)
  
  def rps_to_linear(self, m1, m2):
    m1, m2 = self.format_rps(m1, m2)

    if self.motor_settings.swap_motors:
      m1, m2 = m2, m1
    
    left = m1 / self.motor_settings.rotations_per_metre
    right = m2 / self.motor_settings.rotations_per_metre
    return left, right
  
  def set_acceleration(self, rpm):
    """ Sets Acceleration of motors 

    Sets the rate at which the motors goes from a low speed to a higher speed
    Acceleration value is 0.1*rpm per second.
    Allowed Range: 100 to 32000
    """
    self.send(f"^MAC 1 {rpm}")
    succ_1 = self.get_response()
    succ_2 = self.get_response()

    self.send(f"^MAC 2 {rpm}")
    succ_1 = self.get_response()
    succ_2 = self.get_response()

  def set_deceleration(self, rpm):
    """ Sets Deceleration of motors 

    Sets the rate at which the motors goes from a high speed to a lower speed
    Deceleration value is 0.1*rpm per second.
    Allowed Range: 100 to 32000
    """
    self.send(f"^MDEC 1 {rpm}")
    succ_1 = self.get_response()
    succ_2 = self.get_response()
    
    self.send(f"^MDEC 2 {rpm}")
    succ_1 = self.get_response()
    succ_2 = self.get_response()


  def get_response(self):
    return self.serial.read_until(expected=b"\r").decode("ascii")
  
  def get_analog_input(self):
    self.send("?AI")
    success = self.get_response()
    return self.get_response()

  def get_digital_input(self):
    self.send("?DI")
    success = self.get_response()
    return self.get_response()
  
  def get_motor_command(self):
    self.send("?M")
    success = self.get_response()
    return self.get_response()

  def get_relative_encoders(self):
    self.send("?S")
    success = self.get_response()
    values = self.get_response()
    
    s = values.split(':')
    if len(s) <= 1:
      print(s)
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