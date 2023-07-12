import serial

class MotorDriver(object):

  def __init__(self, 
               port="/dev/ttyUSB0", 
               baud_rate=115200, 
               rotations_per_metre=10,
               swap_motors=True,
               inverse_left_motor=False,
               inverse_right_motor=False):
    self.serial = serial.Serial(port, baud_rate)
    self.run_read_loop = False
    self.rotations_per_metre = rotations_per_metre
    self.swap_motors = swap_motors
    self.inverse_left_motor = -1 if inverse_left_motor else 1
    self.inverse_right_motor = -1 if inverse_right_motor else 1

  def mix(self, left, right):
    """
    Takes left and right comands in m/s (float) and 
    returns values to be sent to motor controller.

    Maps Metres per second to Rotations per second
    """
    m1 = left * self.rotations_per_metre
    m2 = right * self.rotations_per_metre

    if not self.swap_motors:
      m1, m2 = m2, m1

    m1 = int(m1 * self.inverse_left_motor)
    m2 = int(m2 * self.inverse_right_motor)
    
    return m1, m2

  def get_response(self):
    return self.serial.read_until(expected=b"\r").decode("ascii")

  def get_encoders(self):
    self.send("?C")
    self.get_response()
    return self.get_response()

  def send(self, message):
    packet = message.encode("ascii") + b'\r'
    self.serial.write(packet)

  def send_velocity(self, left, right):
    self.send("!m %d %d" %self.mix(left,right))

if __name__ == "__main__":
  conan = MotorDriver(interactive=False)