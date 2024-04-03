from time import sleep
from seeed_xiao_nrf52840 import IMU
import board
import digitalio
import supervisor
from adafruit_lsm6ds import Rate
from ulab import numpy as np


def rms(array):
   return np.sqrt(np.mean(array ** 2))
   
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
    
previous_msecs = supervisor.ticks_ms()
loop_rate = 50 #[Hz]
imu = IMU()
imu.gyro_data_rate = Rate.RATE_104_HZ
imu.accelerometer_data_rate = Rate.RATE_104_HZ
   
window_time_size = 1.5 # [s]
window_samples_size = window_time_size * loop_rate # [s] * [1/s]
data_buffer = np.zeros(int(window_samples_size))
data_iter = 0
imu.high_pass_filter = True
previous_value = np.linalg.norm(imu.acceleration) 

while True:
    current_msecs = supervisor.ticks_ms()
    elapsed_time = (current_msecs - previous_msecs)/1000
    if(elapsed_time >= (1.0/loop_rate)):
        data_buffer[data_iter] = np.linalg.norm(imu.acceleration)
        data_iter = (data_iter + 1)%(int(window_samples_size))
        if(data_iter == int(window_samples_size)-1):
            led.value = not led.value
            accel_difference = np.max([0,rms(data_buffer) - previous_value])
            print((accel_difference,accel_difference))
            previous_value = rms(data_buffer)
        previous_msecs = current_msecs
