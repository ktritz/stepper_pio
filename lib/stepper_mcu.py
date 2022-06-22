import time
import array
import board
from rp2pio import StateMachine
from adafruit_pioasm import Program
import digitalio
import countio
import rotaryio

_asm_str = """
reload:
    {0}        ; direction bit if used
    out y, {1} ; delay
    out x, {2} ; steps
    mov isr, y
    jmp skip 
step_loop:
    nop [3]
skip:
    set pins {3} [10]
    set pins {4}
    {5}        ; jmp if limit switch enabled
continue:
    mov y isr
    push noblock
    mov isr y
delay_loop:
    jmp y-- delay_loop
    jmp x-- step_loop
    jmp reload
stall:
    jmp stall
 """


class Stepper:
    PIO_FREQ = 4e6
    DIR_BIT = False  # set direction in asm?
    DELAY_BIT_LIMIT = 16
    PIO_DELAY = 21  # additional asm clock cycles per step
    STEPS_PER_REV = 200
    MM_PER_REV = 5
    ENC_TICKS_PER_MM = 500

    def __init__(
        self,
        step_pin,
        max_velocity=2000,
        acceleration=0.5,
        delays="linear",
        encoder_pins=None,
        count_pin=None,
        dir_pin=None,
        enable_pin=None,
    ):
        if delays == "scurve":
            from scurve_delay import SCurve as Delay
        else:
            from linear_delay import Linear as Delay
        self.delays = Delay(self, delay_bits=self.DELAY_BIT_LIMIT, dir_bit=self.DIR_BIT)
        self.step_pin = step_pin
        self.max_velocity = max_velocity  # velocity in Hz (1 / delay)
        self.acceleration = acceleration  # time to max_velocity in sec.
        self.micro_steps = 8  # set to stepper driver microstepping

        self.direction = 0  # forward or backward direction

        if dir_pin:
            if not self.DIR_BIT:
                self.dir_pin = digitalio.DigitalInOut(dir_pin)
                self.dir_pin.switch_to_output()
            else:
                self.dir_pin = dir_pin
        else:
            self.dir_pin = None
        self.dir_active = "HIGH"

        if enable_pin:
            self.enable_pin = digitalio.DigitalInOut(dir_pin)
            self.enable_pin.switch_to_output()
        else:
            self.enable_pin = None
        self.dir_active = "HIGH"

        self.jmp_pin = None  # used for stepper limit switch
        self.jmp_active = "LOW"
        self.step_active = "HIGH"
        self._jmp_delay = 0

        if count_pin:
            if self.step_active == "HIGH":
                edge = countio.Edge.RISE
            else:
                edge = countio.Edge.FALL
            self.counter = countio.Counter(count_pin, edge=edge)
            self.counter.reset()
        else:
            self.counter = None

        if encoder_pins:
            self.encoder = rotaryio.IncrementalEncoder(*encoder_pins)
        else:
            self.encoder = None

        self._steps = 0
        self._setup_sm()

    def _setup_sm(self):
        try:
            self._sm.deinit()
        except:
            pass
        if self.DIR_BIT:  # asm dir control
            _dir_asm = "out pins 1"
            dir_pin = self.dir_pin
        else:  # general digital I/O dir control
            _dir_asm = ""
            dir_pin = None

        # setup pin bits for step pin active high/low
        bits = ("0b1", "0b0")
        if self.step_active == "HIGH":
            step_pos, step_neg = bits

        else:
            step_neg, step_pos = bits

        if self.jmp_pin:  # add a jmp command to stall at limit
            # note: activating jmp pin will require sm restart
            if self.jmp_active == "HIGH":
                _jmp_asm = "jmp pin stall"
                _jmp_pull = digitalio.Pull.DOWN
            if self.jmp_active == "LOW":
                _jmp_asm = "jmp pin continue\n    jmp stall"
                _jmp_pull = digitalio.Pull.UP
            self._jmp_delay = 1
        else:
            _jmp_asm = ""
            self._jmp_delay = 0
            _jmp_pull = None

        self.pio_delay = self.PIO_DELAY + self._jmp_delay

        STEP_LIMIT = 32 - self.DELAY_BIT_LIMIT - int(self.DIR_BIT)

        # fill in the stepper_asm variables
        _asm = _asm_str.format(
            _dir_asm, self.DELAY_BIT_LIMIT, STEP_LIMIT, step_pos, step_neg, _jmp_asm
        )
        _stepper_asm = Program(_asm)
        self._sm = StateMachine(
            _stepper_asm.assembled,
            auto_pull=True,
            pull_threshold=32,
            first_set_pin=self.step_pin,
            initial_set_pin_state=int(step_neg),
            first_out_pin=dir_pin,
            jmp_pin=self.jmp_pin,
            jmp_pin_pull=_jmp_pull,
            exclusive_pin_use=False,
            out_shift_right=False,
            frequency=int(self.PIO_FREQ),
            **_stepper_asm.pio_kwargs,
        )

    def _set_dir(self, direction):
        self.direction = direction
        if self.dir_pin:
            self.dir_pin.value = direction ^ (self.dir_active == "LOW")

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

    @property
    def velocity(self):
        if self.stopped():
            return 0
        self._sm.clear_rxfifo()
        _timeout = time.monotonic()
        while not self._sm.in_waiting:
            if (time.monotonic() - _timeout) > 0.1:
                return 0
        delay_bits = array.array("L", [0])
        self._sm.readinto(delay_bits)
        delay = delay_bits[0] + self.pio_delay
        sign = 1 - 2 * self.direction
        return sign * self._sm.frequency / self.micro_steps / delay

    @velocity.setter
    def velocity(self, value):  # accelerate to contiuous rotation
        _ = self.steps  # flush counter
        if value * self.velocity < 0:  # changing direction
            self.stop()
            while self.velocity:
                time.sleep(0.01)
        self._set_dir(int(value < 0))
        value = abs(value)
        velocity = abs(self.velocity)
        acc_delays = self.delays._change_velocity(velocity, value)
        full_speed = array.array("L", [acc_delays[-1]])
        self._sm.background_write(once=acc_delays, loop=full_speed)
        self._sm.clear_txstall()

    @property
    def steps(self):
        if self.counter:
            sign = 1 - 2 * self.direction
            if self.velocity != 0:
                return self._steps + sign * self.counter.count
            else:
                self._steps += sign * self.counter.count
                self.counter.reset()
        return int(self._steps / self.micro_steps)

    @steps.setter
    def steps(self, value):
        if self.velocity != 0:  # not while stepper is moving
            return
        _ = self.steps  # make sure counter is flushed
        self._set_dir(int(value < 0))
        self._sm.background_write(self.delays.gen_delays(abs(value)))
        self._sm.clear_txstall()
        if self.counter is None:
            sign = 1 - 2 * self.direction
            self._steps += sign * abs(value) * self.micro_steps

    def stop(self):  # gracefully decelerate
        v = abs(self.velocity)
        dec_delays = array.array(
            "L", list(self.delays._accel_delays(velocity=v)[:-1])[::-1]
        )
        self._sm.background_write(dec_delays)
        self._sm.clear_txstall()

    def stopped(self):  # check to see if buffer is done
        return self._sm.txstall

