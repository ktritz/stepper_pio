import array


class Delay:
    def __init__(self, stepper, delay_bits=16, dir_bit=False):
        self.stepper = stepper
        self.delay_bits = delay_bits
        self.dir_bit = dir_bit
        self._repeat = 1

    def _accel_steps(self):
        raise NotImplementedError

    def _accel_velocity(self):
        raise NotImplementedError

    def _accel_delays(self):
        raise NotImplementedError

    def gen_delays(self, steps):
        # generate delay|step buffer
        # including acceleration + cruise + deceleration
        STEP_BIT_LIMIT = 32 - self.delay_bits - int(self.dir_bit)
        dir_bit = int(self.dir_bit) * self.stepper.direction
        # scale steps for microstepping
        steps = int(steps) * self.stepper.micro_steps
        step_limit = 2 ** STEP_BIT_LIMIT - 1

        # number of steps during accleration phase
        acc_steps = self._accel_steps()
        half_steps = steps // 2

        # lower max_velocity if steps won't complete acceleration
        if half_steps < acc_steps:
            velocity = self._accel_velocity(half_steps)
            acc_steps = self._accel_steps(velocity=velocity)
        else:
            velocity = self.stepper.max_velocity

        # necessary for step-accurate count
        # not exactly sure why the math works
        if acc_steps % self._repeat:
            acc_steps += self._repeat - (acc_steps % self._repeat)

        # remaining steps at cruising velocity
        acc_delays = self._accel_delays(velocity=velocity)
        sum_steps = sum(d & 0xFFFF for d in acc_delays) + len(acc_delays)
        cruise_steps = steps - 2 * sum_steps
        # deceleration mirrors acceleration curve
        dec_delays = array.array("L", list(acc_delays)[::-1])

        # fill in the buffer for the cruise steps
        c_delay = acc_delays[-1] >> STEP_BIT_LIMIT
        c_delay -= dir_bit << self.delay_bits
        # c_steps = acc_delays[-1] & (2 ** STEP_BIT_LIMIT - 1)

        # remove a step because PIO adds one
        c_steps = cruise_steps - 1
        cruise_arr = array.array("L")

        # use STEP_BIT sized chunks to break up cruise steps
        if c_steps > 0:
            while c_steps > step_limit:
                cruise_arr.append(
                    dir_bit << 31 | c_delay << STEP_BIT_LIMIT | step_limit - 1
                )
                c_steps -= step_limit
            cruise_arr.append(c_delay << 16 | c_steps)

        # final buffer for DMA writes
        return acc_delays + cruise_arr + dec_delays

    def _change_velocity(self, v_start, v_end):
        # acc/decelerate from one velocity to another
        # typically while continuosly rotating

        STEP_BIT_LIMIT = 32 - self.delay_bits - int(self.dir_bit)
        # speed up or slow down
        sign = 1 - 2 * int(v_end < v_start)
        mag = abs(v_end - v_start)

        # smooth curve, but accel time != self.acceleration
        acc_delays = self._accel_delays(velocity=mag)

        # no calculations when starting from 0
        if v_start == 0:
            return acc_delays

        v_to_d = self.stepper._sm.frequency / self.stepper.micro_steps
        d_start = int(v_to_d / v_start)
        d_mask = 2 ** self.delay_bits - 1
        d_mask <<= STEP_BIT_LIMIT
        neg_mask = 0xFFFFFFFF - d_mask

        # add offset delay to delay curve
        for i, delay_bits in enumerate(acc_delays):
            d = delay_bits & d_mask
            d = (d >> STEP_BIT_LIMIT) + self.stepper.pio_delay
            d = int(d * d_start / (d + sign * d_start))
            d -= self.stepper.pio_delay
            d <<= STEP_BIT_LIMIT
            acc_delays[i] = (delay_bits & neg_mask) | d

        # final delay for highest accuracty velocity
        acc_delays[-1] = (acc_delays[-1] & neg_mask) | (
            (int(v_to_d / v_end) - self.stepper.pio_delay) << STEP_BIT_LIMIT
        )
        return acc_delays
