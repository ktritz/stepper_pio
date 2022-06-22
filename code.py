import stepper
import board

s = stepper.Stepper(
    board.D6, limit_pins=(board.D0, board.D1), dir_pin=board.D4, encoder_pis=(board.D2, board.D3)
)
s.dir_active = "HIGH"
s._setup_sm()

