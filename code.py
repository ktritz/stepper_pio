import stepper
import board

s = stepper.Stepper(
    board.D0, limit_pins=(board.D7, board.D7), count_pin=board.D2, dir_pin=board.D6
)
s.dir_active = "HIGH"
s._setup_sm()

