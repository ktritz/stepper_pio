from math import sqrt, ceil
import array
import board
from rp2pio import StateMachine
from adafruit_pioasm import Program

_asm_str = """
reload:
    {0}        ; direction bit if used
    out y, {1} ; delay
    out x, {2} ; steps
    mov isr, y 
step_loop:
    set pins 0b1 [10]
    set pins 0b0
    {3}        ; jmp if limit switch enabled
    mov y isr
delay_loop:
    jmp y-- delay_loop
    jmp x-- step_loop
    jmp reload
stall:
    jmp stall
 """


with open("lib/cos_table.py") as file:
    num = int(file.readline().strip())
    table = array.array("H", [0] * num)
    for i in range(num):
        table[i] = int(file.readline().strip())

PI = 3.1415926535
PI2 = PI * PI
MAX = num - 1
AT = PI2 / MAX
SQAT = PI / MAX

TABLE_SCALE = 2 * PI / 2 ** 16


class Stepper:
    PIO_FREQ = 1e6
    DIR_BIT = False  # set direction in asm?
    DELAY_BIT_LIMIT = 16
    PIO_DELAY = 14  # additional asm clock cycles per step

    def __init__(self, max_velocity=2000, acceleration=0.5):
        self.max_velocity = max_velocity  # velocity in Hz (1 / delay)
        self.acceleration = acceleration  # time to max_velocity in sec.
        self.micro_steps = 8  # set to stepper driver microstepping
        self._table = table  # cos s-curve lookup table
        self._repeat = 1  # repeated delays for coarser acceleration
        self.direction = 0  # forward or backward direction
        self.jmp_pin = None  # used for stepper limit switch
        self._setup_sm()
        self._setup_delays()

    def _setup_sm(self):
        try:
            self._sm.deinit()
        except:
            pass
        if self.DIR_BIT:  # asm dir control
            _dir_asm = "out pins 1"
            dir_pin = board.D1
        else:  # general digital I/O dir control
            _dir_asm = ""
            dir_pin = None

        if self.jmp_pin:  # add a jmp command to stall at limit
            _jmp_asm = "jmp pin stall"
        else:
            _jmp_asm = ""

        STEP_LIMIT = 32 - self.DELAY_BIT_LIMIT - int(self.DIR_BIT)

        # fill in the stepper_asm variables
        _asm = _asm_str.format(_dir_asm, self.DELAY_BIT_LIMIT, STEP_LIMIT, _jmp_asm)
        _stepper_asm = Program(_asm)
        self._sm = StateMachine(
            _stepper_asm.assembled,
            auto_pull=True,
            pull_threshold=32,
            first_set_pin=board.D0,
            first_out_pin=dir_pin,
            jmp_pin=self.jmp_pin,
            exclusive_pin_use=False,
            out_shift_right=False,
            frequency=int(self.PIO_FREQ),
            **_stepper_asm.pio_kwargs,
        )

    def go(self, steps):  # run the sm, output the steps
        self._setup_sm()
        self._sm.background_write(self.gen_delays(steps))
        self._sm.clear_txstall()

    def stopped(self):  # check to see if buffer is done
        return self._sm.txstall

    def _setup_delays(self):  # stored values for faster step calcs
        self._stored_velocity = 0
        self._stored_accel = 0
        self._stored_delays = []
        self._stored_freq = self.PIO_FREQ

    def _unique(self, delays):
        STEP_BIT_LIMIT = 32 - self.DELAY_BIT_LIMIT - int(self.DIR_BIT)
        # collect unique delay values
        # and count how many are in delay list
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
        dir_bit = int(self.DIR_BIT) * self.direction << 31

        # additional asm step delay per loop
        dd = self.PIO_DELAY + int(self.jmp_pin is not None)
        
        # build the dir|delay|step buffer for DMA writing
        return array.array(
            "L",
            [
                dir_bit | (d - dd) << STEP_BIT_LIMIT | c
                for d, c in zip(u_delays, u_counts)
            ],
        )

    def _delay(self, i, steps, step_scale):
        # will return the interpolated time (0..2*PI) that corresponds to a given distance sqrt(0 .. PI*PI)
        # according to inverse lookup table
        sqt = self._table

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

    def _accel_delays(self, velocity=None):
        # generate list of (delays in us, # of pulses)
        # for cos-based acceleration S-curve
        delay_limit = 2 ** self.DELAY_BIT_LIMIT - 1
        if velocity is None:
            velocity = self.max_velocity
        steps = self._accel_steps(velocity=velocity)

        # keep the acceleration steps < 2000 by switching
        # to a coarser step-delay curve and repeating delays
        # speeds calc time and reduces buffer size
        self._repeat = ceil(steps / 2000)

        # scale factor to hit the correct delay
        # for max_velocity at the end of acceleration
        # subject to statemachine frequency resolution
        kt = (
            0.1
            * self.PIO_FREQ
            * self.acceleration
            / 0.624
            * steps
            / (steps - 1)
            * TABLE_SCALE
        )
        step_scale = 1 / 4 / steps  # precalc for delay loop

        # get fine-grained delays for first few acceleration steps
        delays = [
            min(int(self._delay(i, steps, step_scale) * kt), delay_limit)
            for i in range(0, self._repeat)
        ]
        # switch to coarser acceleration steps if self._repeat > 1
        delays += [
            min(int(self._delay(i, steps, step_scale) * kt), delay_limit)
            for i in range(self._repeat, steps, self._repeat)
        ]
        return self._unique(delays)

    def gen_delays(self, steps):
        # generate delay|step buffer
        # including acceleration + cruise + deceleration
        STEP_BIT_LIMIT = 32 - self.DELAY_BIT_LIMIT - int(self.DIR_BIT)
        dir_bit = int(self.DIR_BIT) * self.direction
        velocity = self.max_velocity
        # scale steps for microstepping
        steps = int(steps) * self.micro_steps
        step_limit = 2 ** STEP_BIT_LIMIT - 1

        # number of steps during accleration phase
        acc_steps = self._accel_steps()
        half_steps = steps // 2

        # lower max_velocity if steps won't complete acceleration
        if half_steps < acc_steps:
            velocity = self._calc_velocity(half_steps)
            acc_steps = self._accel_steps(velocity=velocity)

        # sacrifice some storage to speed calcs
        # if acceleration parameters are stable
        if (
            (velocity == self._stored_velocity)
            & (self.acceleration == self._stored_accel)
            & (self.PIO_FREQ == self._stored_freq)
        ):
            acc_delays = self._stored_delays
        else:
            acc_delays = self._accel_delays(velocity=velocity)
            self._stored_velocity = velocity
            self._stored_accel = self.acceleration
            self._stored_delays = acc_delays
            self._stored_freq = self.PIO_FREQ

        # necessary for step-accurate count
        # not exactly sure why the math works
        if acc_steps % self._repeat:
            acc_steps += self._repeat - (acc_steps % self._repeat)

        # remaining steps at cruising velocity
        cruise_steps = steps - 2 * acc_steps

        # deceleration mirrors acceleration curve
        dec_delays = array.array("L", list(acc_delays[:-1])[::-1])

        # fill in the buffer for the cruise steps
        c_delay = acc_delays[-1] >> STEP_BIT_LIMIT
        c_delay -= dir_bit << self.DELAY_BIT_LIMIT
        c_steps = acc_delays[-1] & (2 ** STEP_BIT_LIMIT - 1)

        # have to add extra step because extra asm
        # step is subtracted twice (2 * c_steps)
        c_steps = 2 * c_steps + cruise_steps + 1
        cruise_arr = array.array("L")

        # use STEP_BIT sized chunks to break up cruise steps
        while c_steps > step_limit:
            cruise_arr.append(
                dir_bit << 31 | c_delay << STEP_BIT_LIMIT | step_limit - 1
            )
            c_steps -= step_limit
        cruise_arr.append(c_delay << 16 | c_steps)

        # final buffer for DMA writes
        return acc_delays[:-1] + cruise_arr + dec_delays

