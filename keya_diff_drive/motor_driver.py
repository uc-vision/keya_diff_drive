from rclpy.impl import rcutils_logger
import serial
import time
from dataclasses import dataclass
from enum import Flag, auto

logger = rcutils_logger.RcutilsLogger(name="Motor_driver")

@dataclass
class SerialSettings(object):
  port: str = "/dev/ttyUSB0"
  baud_rate: int = 115200
  timeout: float = 0.1

@dataclass
class MotorDriverSettings(object):
  rotations_per_metre: int = 10
  max_rpm: int = 3000
  swap_motors: bool = False
  inverse_left_motor: bool = False
  inverse_right_motor: bool = False

class FaultStatus(Flag):
  NO_FAULT = 0
  OVERHEAT = auto()
  OVERVOLTAGE = auto()
  UNDERVOLTAGE = auto()
  SHORT_CIRCUIT = auto()
  EMERGENCY_STOP = auto()
  SEPEX_EXCITATION = auto()
  MOSFET_FAILURE = auto()
  CONFIGURATION_ERR = auto()


class StateStatus(Flag):
  NO_INPUT = 0
  PULSE_MODE = auto()
  ANALOG_MODE = auto()
  POWER_STAGE_OFF = auto()
  SHORT_CIRCUIT = auto()
  EMERGENCY_STOP = auto()
  SEPEX_EXCITATION = auto()
  MOSFET_FAILURE = auto()
  CONFIGURATION_ERR = auto()




class MotorDriver(object):

  def __init__(self, serial_settings: SerialSettings, motor_settings: MotorDriverSettings):
    self.serial = serial.Serial(
      serial_settings.port, 
      serial_settings.baud_rate, 
      timeout=serial_settings.timeout)
  
    self.rotations_per_metre: int = motor_settings.rotations_per_metre
    self.max_rpm: int = motor_settings.max_rpm
    self.swap_motors: bool = motor_settings.swap_motors
    self.inverse_left_motor: bool = motor_settings.inverse_left_motor 
    self.inverse_right_motor: bool = motor_settings.inverse_right_motor

  def format_rps(self, m1, m2):
    inverse_left_motor = -1 if self.inverse_left_motor else 1
    inverse_right_motor = -1 if self.inverse_right_motor else 1
    m1 = int(m1 * inverse_left_motor)
    m2 = int(m2 * inverse_right_motor)
    return m1, m2

  def linear_to_rps(self, left, right):
    m1 = left * self.rotations_per_metre
    m2 = right * self.rotations_per_metre
    
    if self.swap_motors:
      m1, m2 = m2, m1

    return self.format_rps(m1, m2)
  
  def rps_to_linear(self, m1, m2):
    m1, m2 = self.format_rps(m1, m2)

    if self.swap_motors:
      m1, m2 = m2, m1
    
    left = m1 / self.rotations_per_metre
    right = m2 / self.rotations_per_metre
    return left, right


  def get_response(self):
    return self.serial.read_until(expected=b"\r").decode("ascii", errors="replace")

  
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


  def get_state_status(self):
    self.send("?FS")
    success = self.get_response()
    result = self.get_response()
    return StateStatus(int(result[3:]))
  
  def get_fault_status(self):
    self.send("?FF 0")
    success = self.get_response()
    result1 =  self.get_response() # Reply: "FF = f1 + f2*2 + f3*4 + ... + fn*2n-1"

    self.send("?FF 1")
    success = self.get_response()
    result2 =  self.get_response()

    result1_parsed = int(result1[3:])
    result2_parsed = int(result2[3:])

    return FaultStatus(result1_parsed), FaultStatus(result2_parsed)


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
    try:
      self.send("?S")
      success = self.get_response()
      values = self.get_response()

      s = values.split(':')
      m1 = int(s[0][2:])/3.0  # encoders report 3x speed
      m2 = int(s[1][:-1])/3.0 # encoders report 3x speed

      return self.rps_to_linear(m1, m2)
    except ValueError as e:
      raise ValueError(str(s)) from e
    

  def send(self, message):
    packet = message.encode("ascii") + b'\r'
    self.serial.write(packet)
    

  def send_velocity(self, left, right):
    m1, m2 = self.linear_to_rps(left, right)

    if m1 >= 0:
      capped_m1 = min( m1, self.max_rpm/3 )
    else:
      capped_m1 = max( m1 , -1 * self.max_rpm/3 )

    if m2 >= 0:
      capped_m2 = min( m2, self.max_rpm/3 )
    else:
      capped_m2 = max( m2 , -1 * self.max_rpm/3 )

    self.send("!m %d %d" %(capped_m1,capped_m2))
    # clear responses
    self.get_response()
    self.get_response()

  def close(self):
    self.serial.close()

if __name__ == "__main__":
  conan = MotorDriver()
