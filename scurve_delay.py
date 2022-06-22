import array
from math import sqrt, ceil
from step_gen import Delay

try:
    file = open("lib/cos_table.py")
except FileNotFoundError:
    file = open("cos_table.py")
finally:
    num = int(file.readline().strip())
    TABLE = array.array("H", [0] * num)
    for i in range(num):
        TABLE[i] = int(file.readline().strip())
    file.close()

PI = 3.1415926535
PI2 = PI * PI
MAX = num - 1
AT = PI2 / MAX
SQAT = PI / MAX

TABLE_SCALE = 2 * PI / 2 ** 16
ACC_CONST = 0.624


class SCurve(Delay):
    def __init__(self, stepper, **kwargs):
        super().__init__(stepper, **kwargs)
        self._cache = [0, 1, 2]
        self._cache_dict = dict(zip(self._cache, [0] * 3))

    def _accel_delays(self, velocity=None):
        # generate list of (delays in us, # of pulses)
        # for cos-based acceleration S-curve
        if velocity is None:
            velocity = self.stepper.max_velocity
        frequency = self.stepper._sm.frequency
        micro_steps = self.stepper.micro_steps
        acceleration = self.stepper.acceleration
        args = (velocity, frequency, micro_steps, acceleration)
        if (args) in self._cache:
            return self._cache_dict[args]

        delay_limit = 2 ** self.delay_bits - 1

        steps = self._accel_steps(velocity=velocity)

        # keep the acceleration steps < 2000 by switching
        # to a coarser step-delay curve and repeating delays
        # speeds calc time and reduces buffer size
        self._repeat = ceil(steps / 1000)

        # scale factor to hit the correct delay
        # for max_velocity at the end of acceleration
        # subject to statemachine frequency resolution
        kt = 0.1 * frequency * acceleration / ACC_CONST * TABLE_SCALE
        step_scale = 1 / 4 / steps  # precalc for delay loop

        # get fine-grained delays for first few acceleration steps
        delays = [
            min(int(self._interp_scurve(i, steps, step_scale) * kt), delay_limit)
            for i in range(0, self._repeat)
        ]
        # switch to coarser acceleration steps if self._repeat > 1
        delays += [
            min(int(self._interp_scurve(i, steps, step_scale) * kt), delay_limit)
            for i in range(self._repeat, steps, self._repeat)
        ]
        min_delay = int(frequency / velocity / micro_steps)
        u_delays = self._unique(delays, min_delay)

        # cache delays
        old_args = self._cache.pop()
        del self._cache_dict[old_args]
        self._cache.insert(0, args)
        self._cache_dict[args] = u_delays
        return u_delays

    def _interp_scurve(self, i, steps, step_scale):
        # will return the interpolated time (0..2*PI) that corresponds to a given distance sqrt(0 .. PI*PI)
        # according to inverse lookup table
        sqt = TABLE

        # avoid div0 for the first delay
        if i == 0:
            return (sqt[1] - sqt[0]) * PI * sqrt(1 / steps) / SQAT

        # find scaled step position in lookup table
        sqppos = PI * sqrt(i / steps)

        # approximate sqrt((i+1)/steps) - sqrt(i/steps) to estimate delay at step i
        dp = sqrt(step_scale / i) - sqrt(step_scale / 16 / i) / i
        dp *= PI

        # handle scaled step index at end of table
        if sqppos >= PI:
            return (sqt[-1] - sqt[-2]) * dp / SQAT

        # interpolate the delay value based on closest indices
        n = int(sqppos / SQAT)
        return (sqt[n + 1] - sqt[n]) * dp / SQAT

    def _unique(self, delays, min_delay):
        STEP_BIT_LIMIT = 32 - self.delay_bits - int(self.dir_bit)
        # collect unique delay values
        # and count how many are in delay list
        delay_limit = 2 ** self.delay_bits - 1
        for i, d in enumerate(delays):
            delays[i] = min(max(d, min_delay), delay_limit)

        u_delays = sorted(set(delays))[::-1]

        # start at -1 because asm adds a step to each delay item
        u_counts = [-1] * len(u_delays)

        # if we're in 'coarse mode' (self._repeat > 1)
        # use single steps for first few acceleration delays
        for d in delays[0 : self._repeat]:
            u_counts[u_delays.index(d)] += 1

        # use coarse stepping for the remainder
        for d in delays[self._repeat :]:
            u_counts[u_delays.index(d)] += self._repeat

        # set the MSB direction bit if used
        dir_bit = int(self.dir_bit) * self.stepper.direction << 31

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

    def _accel_steps(self, velocity=None):
        # number of (micro) steps to reach velocity in acceleration time
        if velocity is None:
            velocity = self.stepper.max_velocity
        micro_steps = self.stepper.micro_steps
        acceleration = self.stepper.acceleration

        kt = 0.1 * acceleration / ACC_CONST * micro_steps
        max_delay = 1 / velocity / kt * 2
        steps = (TABLE[-1] - TABLE[-2]) / max_delay * MAX * TABLE_SCALE
        return int(steps)

    def _accel_velocity(self, steps):
        # reduce max_velocity if steps < _accel_steps(max_velocity)
        micro_steps = self.stepper.micro_steps
        acceleration = self.stepper.acceleration

        kt = 0.1 * acceleration / ACC_CONST * micro_steps
        max_delay = (TABLE[-1] - TABLE[-2]) / steps * MAX * TABLE_SCALE
        velocity = 1 / max_delay / kt * 2
        return velocity
