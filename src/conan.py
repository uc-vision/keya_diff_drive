import serial
import threading
import sys
import time

class Conan(object):

    def __init__(self, port="/dev/ttyUSB0", interactive=True):
        self.serial = serial.Serial(port, 115200)
        self.run_read_loop = False
        if interactive:
            self.start_read_loop()

    def mix(self, left, right):
        """
        Takes left and right comands in m/s (float) and 
        returns values to be sent to motor controller.
        """
        m1 = right / 0.000455 / 3
        m2 = left / 0.000455 / 3
        return int(m1), int(m2) 
        
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
        #self.get_response()
        #self.get_response()

    def move_distance(self, left, right, duration, stop=True):
        end_time = time.time() + duration
        while time.time() < end_time:
            self.send_velocity(left / duration, right / duration)
            time.sleep(0.1)
        if stop:
            self.send_velocity(0.0,0.0)


if __name__ == "__main__":
    conan = Conan(interactive=False)
    #while True:
    #    print(conan.get_encoders())

    while True:
        conan.move_distance(2,2,5)
        time.sleep(1)
        conan.move_distance(-2,-2,5)
        time.sleep(1)
        conan.move_distance(2,2,20)
        time.sleep(1)
        conan.move_distance(-2,-2,20)
        time.sleep(1)

