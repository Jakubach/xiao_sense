import time
import board
import digitalio
from time import sleep
from seeed_xiao_nrf52840 import IMU

motor = digitalio.DigitalInOut(board.D0)
motor.direction = digitalio.Direction.OUTPUT

while True:
    motor.value = 1
    sleep(1)
    motor.value = 0
    sleep(1)




