from time import sleep
from seeed_xiao_nrf52840 import IMU
import board
import digitalio
import supervisor
from adafruit_lsm6ds import Rate
#led = digitalio.DigitalInOut(board.LED)
#led.direction = digitalio.Direction.OUTPUT

    
previous_msecs = supervisor.ticks_ms()
loop_rate = 10 #[Hz]
imu = IMU()
imu.gyro_data_rate = Rate.RATE_26_HZ
imu.accelerometer_data_rate = Rate.RATE_26_HZ

while True:
    current_msecs = supervisor.ticks_ms()
    elapsed_time = (current_msecs - previous_msecs)/1000
    if(elapsed_time >= (1.0/loop_rate)):
        print("%.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f" % (imu.acceleration + imu.gyro))
        previous_msecs = current_msecs
