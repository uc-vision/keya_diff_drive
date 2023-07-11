import serial
import threading
import sys
import time

class MotorDriver(object):

  def __init__(self, 
               port="/dev/ttyUSB0", 
               baud_rate=115200, 
               rotations_per_metre=10,
               swap_motors=True,
               reverse_left_motor=False,
               reverse_right_motor=False,
               interactive=True):
    self.serial = serial.Serial(port, baud_rate)
    self.run_read_loop = False
    self.rotations_per_metre = rotations_per_metre
    self.swap_motors = swap_motors
    self.reverse_left_motor = -1 if reverse_left_motor else 1
    self.reverse_right_motor = -1 if reverse_right_motor else 1
    if interactive:
      self.start_read_loop()

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

    m1 = int(m1 * self.reverse_left_motor)
    m2 = int(m2 * self.reverse_right_motor)
    
    return m1, m2
      
  def start_read_loop(self):
    self.run_read_loop = True
    threading.Thread(target=self.read_loop).start()
  
  def read_loop(self):
    while self.run_read_loop:
      b = self.serial.read()
      if b == b'\r':
          sys.stdout.write("\n\r")
      sys.stdout.write(b.decode("ascii"))
      sys.stdout.flush()

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

  def move_distance(self, left, right, duration, stop=True):
    end_time = time.time() + duration
    while time.time() < end_time:
      self.send_velocity(left / duration, right / duration)
      time.sleep(0.1)
    if stop:
      self.send_velocity(0.0,0.0)

if __name__ == "__main__":
  conan = MotorDriver(interactive=False)