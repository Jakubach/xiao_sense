from time import sleep
from seeed_xiao_nrf52840 import IMU

while True:
    with IMU() as imu:
        print("Acceleration:", imu.acceleration)
        sleep(1)
