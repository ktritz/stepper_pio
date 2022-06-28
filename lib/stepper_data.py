import usb_cdc
import struct

serial = usb_cdc.data


class StepperData:
    serial = serial

    def __init__(self, stepper):
        self.stepper = stepper

    def send(self):
        p = self.stepper.position
        v = self.stepper.velocity
        bdata = struct.pack("ff", p, v)
        self.serial.write(bdata)
