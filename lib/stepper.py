from stepper_pio import Stepper as StepperPIO
import rotaryio


class Stepper(StepperPIO):
    ENC_TICKS_PER_MM = 50

    def __init__(self, step_pin, encoder_pins=None, limit_pins=None, **kwargs):

        if encoder_pins:
            self.encoder = rotaryio.IncrementalEncoder(*encoder_pins)
        else:
            self.encoder = None

        self.limit_pins = limit_pins
        super().__init__(step_pin, **kwargs)

    @property
    def position(self):
        if self.encoder:
            return self.encoder.position / self.ENC_TICKS_PER_MM
        else:
            return self.steps / self.STEPS_PER_REV * self.MM_PER_REV

    @position.setter
    def position(self, value):
        delta_mm = value - self.position
        self.steps = delta_mm / self.MM_PER_REV * self.STEPS_PER_REV

    def _set_dir(self, direction):
        self.direction = direction
        if self.limit_pins:
            self.jmp_pin = self.limit_pins[direction]
            self._setup_sm()
        super()._set_dir(direction)

