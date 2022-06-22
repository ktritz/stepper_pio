import board
import stepper_mcu

s = stepper_mcu.Stepper(board.GP0, delays="scurve")
