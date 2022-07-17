import usb_cdc
import struct


class StepperData:

    def __init__(self, stepper):
        self.stepper = stepper
        self.serial = usb_cdc.data

    def send(self):
        try:
            p = self.stepper.position
            v = self.stepper.velocity
            l = self.stepper.limit
        except:
            p = 0
            v = 0
            l = 0
        bdata = struct.pack("ffB", p, v, l)
        self.serial.write(bdata)
