import serial
import struct


class StepperData:
    def __init__(self, port):
        self.serial = serial.Serial(port=port, baudrate=1000000, timeout=0.1)
        self.position = None
        self.velocity = None
        self.limit = 0

    def read(self):
        self.serial.reset_input_buffer()
        data = self.serial.read(9)
        try:
            self.position, self.velocity, self.limit = struct.unpack("ffB", data)
        except:
            self.serial.reset_input_buffer()
            data = self.serial.read(9)
            try:
                self.position, self.velocity, self.limit = struct.unpack("ffB", data)
            except:
                self.position, self.velocity, self.limit = None, None, 0
