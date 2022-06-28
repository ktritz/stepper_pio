import serial
import struct


class StepperData:
    def __init__(self, port):
        self.serial = serial.Serial(port=port, baudrate=1000000, timeout=0.1)
        self.position = None
        self.velocity = None

    def read(self):
        self.serial.reset_input_buffer()
        data = self.serial.read(8)
        try:
            self.position, self.velocity = struct.unpack("ff", data)
        except:
            self.serial.reset_input_buffer()
            data = self.serial.read(8)
            try:
                self.position, self.velocity = struct.unpack("ff", data)
            except:
                self.position, self.velocity = None, None
