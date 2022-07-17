from math import ceil, sqrt
import array
from step_gen import Delay


class Linear(Delay):
    def _accel_delays(self, velocity=None):
        if velocity is None:
            velocity = self.stepper.max_velocity
        velocity = max(velocity, 10)
        frequency = self.stepper._sm.frequency
        micro_steps = self.stepper.micro_steps
        acceleration = self.stepper.acceleration

        delay_limit = 2 ** self.delay_bits - 1

        min_delay = int(frequency / velocity / micro_steps)
        num_steps = max(int(velocity * micro_steps * acceleration / 100), 1)
        delays = [
            min(int(min_delay * num_steps / (i + 1)), delay_limit)
            for i in range(num_steps)
        ]
        delays[-1] = min_delay
        return self._unique(delays)

    def _accel_steps(self, velocity=None):
        if velocity is None:
            velocity = self.stepper.max_velocity
        micro_steps = self.stepper.micro_steps
        acceleration = self.stepper.acceleration

        n = velocity * micro_steps * acceleration / 100 + 1
        return int((n * n - n) / 2)

    def _accel_velocity(self, steps):
        micro_steps = self.stepper.micro_steps
        acceleration = self.stepper.acceleration

        n = (1 + sqrt(8 * steps + 1)) / 2 - 1
        return 100 * n / acceleration / micro_steps

    def _unique(self, delays):

        direction = self.stepper.direction
        STEP_BIT_LIMIT = 32 - self.delay_bits - int(self.dir_bit)

        u_delays = delays
        u_counts = list(range(len(u_delays)))

        # set the MSB direction bit if used
        dir_bit = int(self.dir_bit) * direction << 31

        # additional asm step delay per loop
        dd = self.stepper.pio_delay

        # build the dir|delay|step buffer for DMA writing
        return array.array(
            "L",
            [
                dir_bit | (d - dd) << STEP_BIT_LIMIT | c
                for d, c in zip(u_delays, u_counts)
            ],
        )
