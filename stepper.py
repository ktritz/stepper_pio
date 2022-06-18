import cos_calc
from math import sqrt

import numpy as np
import matplotlib.pyplot as plt

PI = 3.1415926535
PI2 = PI * PI
MAX = 1000
AT = PI2 / MAX
SQAT = PI / MAX


class Stepper:
    PIO_FREQ = 1e6
    BIT_LIMIT = 16

    def __init__(self, max_velocity=1000, acceleration=1.0):
        self.max_velocity = max_velocity  # velocity in Hz (1 / delay)
        self.acceleration = acceleration  # time to max_velocity in sec.
        self.micro_steps = 1
        self._table = cos_calc.sq_table(MAX + 1)

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
        for d in delays:
            u_counts[u_delays.index(d)] += 1
        return u_delays, u_counts

    def _solve(self, sqppos):
        # will return the interpolated time (0..2*PI) that corresponds to a given distance sqrt(0 .. PI*PI)
        # according to inverse lookup table
        sqt = self._table
        if sqppos >= PI:
            return sqt[MAX - 1]
        n = int(sqppos / SQAT)
        dp = sqppos - n * SQAT
        if dp == 0:
            return sqt[n]
        return (sqt[n + 1] - sqt[n]) * dp / SQAT + sqt[n]

    def _accel_steps(self, velocity=None):
        # number of (micro) steps to reach velocity in acceleration time
        if velocity is None:
            velocity = self.max_velocity
        kt = 0.1 * self.acceleration / 0.63 * self.micro_steps
        max_delay = 1 / velocity / kt * 2
        steps = (self._table[-1] - self._table[-2]) / max_delay * 1000
        return int(steps)

    def _calc_velocity(self, steps):
        # reduce max_velocity if steps < _accel_steps(max_velocity)
        kt = 0.1 * self.acceleration / 0.63 * self.micro_steps
        max_delay = (self._table[-1] - self._table[-2]) / steps * 1000
        velocity = 1 / max_delay / kt * 2
        return velocity

    def _accel_delays(self, velocity=None):
        limit = 2 ** self.BIT_LIMIT - 1
        if velocity is None:
            velocity = self.max_velocity
        # generate list of (delays in us, # of pulses) for cos-based S-curve
        steps = self._accel_steps(velocity=velocity) + 1
        kt = 0.1 * self.PIO_FREQ * self.acceleration / 0.63 * steps / (steps - 1)
        d = [self._solve(sqrt(PI2 * i / (steps))) * kt for i in range(steps)]
        delays = [min(int(d[i + 1] - d[i]), limit) for i in range(len(d[:-1]))]
        return list(zip(*self._unique(delays)))

    def gen_delays(self, steps):
        velocity = self.max_velocity
        steps = int(steps) * self.micro_steps
        limit = 2 ** self.BIT_LIMIT - 1
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
        while c_steps > limit:
            acc_delays[-1] = (c_delay, limit)
            acc_delays.append((c_delay, 0))
            c_steps -= limit
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

