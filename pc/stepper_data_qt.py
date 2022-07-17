from PyQt5.QtSerialPort import QSerialPort
from PyQt5.QtCore import QIODevice
import struct


class StepperData:
    def __init__(self, port):
        self.serial = QSerialPort()
        self.serial.setPortName(port)
        self.serial.baudRate = 1000000
        self.serial.open(QIODevice.ReadOnly)
        self.serial.setReadBufferSize(9)
        self.serial.readyRead.connect(self._read_data)
        self.serial.clear()

        self.position = None
        self.velocity = None
        self.limit = 0

    def _read_data(self):
        data = self.serial.read(9)
        self.position, self.velocity, self.limit = struct.unpack("ffB", data)

