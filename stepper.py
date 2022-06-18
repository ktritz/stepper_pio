import cos_calc
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt, ceil
import array


PI = 3.1415926535
PI2 = PI * PI
MAX = 1000
AT = PI2 / MAX
SQAT = PI / MAX

with open("cos_table.py") as file:
    num = int(file.readline().strip())
    table = array.array("L", [0] * num)
    for i in range(num):
        table[i] = int(file.readline().strip())

TABLE_SCALE = 2 * PI / 2 ** 16


class Stepper:
    PIO_FREQ = 1e6
    DIR_BIT = False
    DELAY_BIT_LIMIT = 16

    def __init__(self, max_velocity=4000, acceleration=1.0):
        self.max_velocity = max_velocity  # velocity in Hz (1 / delay)
        self.acceleration = acceleration  # time to max_velocity in sec.
        self.micro_steps = 8
        self._table = table
        self._repeat = 1

    def _gen_table(self, num=MAX + 1):
        table = (cos_calc.sq_table(num) / 2 / PI * 2 ** 16).astype(np.uint16)
        table[-1] = 2 ** 16 - 1
        table_str = "\n".join([f"{t}" for t in table])
        with open("cos_table.py", "w", encoding="utf-8") as file:
            # file.write(f"table = ({table_str})")
            file.write(f"{num}\n")
            file.write(table_str)

    def _unique(self, delays):
        u_delays = sorted(set(delays))[::-1]
        u_counts = [0] * len(u_delays)
        for d in delays[0 : self._repeat]:
            u_counts[u_delays.index(d)] += 1
        for d in delays[self._repeat :]:
            u_counts[u_delays.index(d)] += self._repeat
        return u_delays, u_counts

    def _accel_steps(self, velocity=None):
        # number of (micro) steps to reach velocity in acceleration time
        if velocity is None:
            velocity = self.max_velocity
        kt = 0.1 * self.acceleration / 0.63 * self.micro_steps
        max_delay = 1 / velocity / kt * 2
        steps = (self._table[-1] - self._table[-2]) / max_delay * 1000 * TABLE_SCALE
        return int(steps)

    def _calc_velocity(self, steps):
        # reduce max_velocity if steps < _accel_steps(max_velocity)
        kt = 0.1 * self.acceleration / 0.63 * self.micro_steps
        max_delay = (self._table[-1] - self._table[-2]) / steps * 1000 * TABLE_SCALE
        velocity = 1 / max_delay / kt * 2
        return velocity

    def _delay(self, i, steps, step_scale):
        # will return the interpolated time (0..2*PI) that corresponds to a given distance sqrt(0 .. PI*PI)
        # according to inverse lookup table
        sqt = self._table

        if i == 0:
            return (sqt[1] - sqt[0]) * PI * sqrt(1 / steps) / SQAT

        sqppos = PI * sqrt(i / steps)

        # approximate sqrt((i+1)/steps) - sqrt(i/steps) to estimate delay at step i
        dp = sqrt(step_scale / i) - sqrt(step_scale / 16 / i) / i
        dp *= PI
        if sqppos >= PI:
            return (sqt[-1] - sqt[-2]) * dp / SQAT
        n = int(sqppos / SQAT)
        return (sqt[n + 1] - sqt[n]) * dp / SQAT

    def _accel_delays(self, velocity=None):
        delay_limit = 2 ** self.DELAY_BIT_LIMIT - 1
        if velocity is None:
            velocity = self.max_velocity
        # generate list of (delays in us, # of pulses) for cos-based S-curve
        steps = self._accel_steps(velocity=velocity)
        self._repeat = ceil(steps / 2000)
        kt = 0.1 * self.PIO_FREQ * self.acceleration / 0.63 * TABLE_SCALE
        step_scale = 1 / 4 / steps  # precalc for delay loop
        delays = [
            min(int(self._delay(i, steps, step_scale) * kt), delay_limit)
            for i in range(0, self._repeat)
        ]
        delays += [
            min(int(self._delay(i, steps, step_scale) * kt), delay_limit)
            for i in range(self._repeat, steps, self._repeat)
        ]
        return list(zip(*self._unique(delays)))

    def bit_convert(self, delay_list):
        STEP_BIT_LIMIT = 32 - self.DELAY_BIT_LIMIT - int(self.DIR_BIT)
        num = len(delay_list)
        delay_arr = array.array("L", [0] * num)
        for i, (delay, steps) in enumerate(delay_list):
            # pioasm program adds 1 step to each delay entry
            delay_arr[i] = delay << STEP_BIT_LIMIT | (steps - 1)
        return delay_arr

    def gen_delays(self, steps):
        STEP_BIT_LIMIT = 32 - self.DELAY_BIT_LIMIT - int(self.DIR_BIT)
        velocity = self.max_velocity
        steps = int(steps) * self.micro_steps
        step_limit = 2 ** STEP_BIT_LIMIT - 1
        acc_steps = self._accel_steps()
        half_steps = steps // 2
        if half_steps < acc_steps:
            velocity = self._calc_velocity(half_steps)
            acc_steps = self._accel_steps(velocity=velocity)

        cruise_steps = steps - 2 * acc_steps
        acc_delays = self._accel_delays(velocity=velocity)
        dec_delays = acc_delays[:-1][::-1]
        c_delay = acc_delays[-1][0]
        c_steps = acc_delays[-1][1]
        c_steps = 2 * c_steps + cruise_steps
        while c_steps > step_limit:
            acc_delays[-1] = (c_delay, step_limit)
            acc_delays.append((c_delay, 0))
            c_steps -= step_limit
        acc_delays[-1] = (c_delay, c_steps)
        acc_delays.extend(dec_delays)
        return acc_delays

    def plot(self, steps):
        delays = self.gen_delays(steps)
        steps = [0] + list(np.cumsum([t[1] for t in delays]))
        vel = [0] + [self.PIO_FREQ / t[0] for t in delays]
        t = [0] + list(np.cumsum([t[1] * t[0] / self.PIO_FREQ for t in delays]))
        plt.plot(t, vel)
        plt.plot(t, steps)

