import time
import array
import numpy as np
import matplotlib.pyplot as plt


class StateMachine:
    pass


class Stepper:
    PIO_FREQ = 4e6
    DIR_BIT = False  # set direction in asm?
    DELAY_BIT_LIMIT = 16
    PIO_DELAY = 21  # additional asm clock cycles per step

    def __init__(
        self, max_velocity=2000, acceleration=0.5, delays="linear",
    ):
        if delays == "scurve":
            from scurve_delay import SCurve as Delay
        else:
            from linear_delay import Linear as Delay
        self.delays = Delay(self, delay_bits=self.DELAY_BIT_LIMIT, dir_bit=self.DIR_BIT)
        self.max_velocity = max_velocity  # velocity in Hz (1 / delay)
        self.acceleration = acceleration  # time to max_velocity in sec.
        self.micro_steps = 8  # set to stepper driver microstepping

        self.direction = 0  # forward or backward direction
        self._velocity = 0
        self._steps = 0
        self.pio_delay = self.PIO_DELAY
        self._sm = StateMachine()
        self._sm.frequency = self.PIO_FREQ

    def _set_dir(self, direction):
        self.direction = direction

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):  # accelerate to contiuous rotation
        self._velocity = value
        self._set_dir(int(value < 0))
        value = abs(value)
        velocity = abs(self.velocity)
        acc_delays = self.delays._change_velocity(velocity, value)
        return acc_delays

    @property
    def steps(self):
        return int(self._steps / self.micro_steps)

    @steps.setter
    def steps(self, value):
        self._set_dir(int(value < 0))
        sign = 1 - 2 * self.direction
        self._steps += sign * value

    def stop(self):  # gracefully decelerate
        v = abs(self.velocity)
        dec_delays = array.array(
            "L", list(self.delays._accel_delays(velocity=v)[:-1])[::-1]
        )
        return dec_delays

    def plot(self, steps):
        delays = self.delays.gen_delays(steps)
        STEP_BITS = 32 - self.DELAY_BIT_LIMIT
        STEP_MASK = 2 ** STEP_BITS - 1
        dd = [(d >> STEP_BITS) + self.pio_delay for d in delays]
        ss = [(s & STEP_MASK) + 1 for s in delays]
        steps = [0] + list(np.cumsum(ss) / self.micro_steps)
        vel = [0] + [self.PIO_FREQ / d / self.micro_steps for d in dd]
        t = [0] + list(np.cumsum([d * s / self.PIO_FREQ for d, s in zip(dd, ss)]))
        plt.plot(t, vel)
        plt.plot(t, steps)

