from time import sleep
from seeed_xiao_nrf52840 import IMU
import board
import digitalio
import supervisor
from adafruit_lsm6ds import Rate
from ulab import numpy as np
#led = digitalio.DigitalInOut(board.LED)
#led.direction = digitalio.Direction.OUTPUT

    
previous_msecs = supervisor.ticks_ms()
loop_rate = 50 #[Hz]
imu = IMU()
imu.gyro_data_rate = Rate.RATE_104_HZ
imu.accelerometer_data_rate = Rate.RATE_104_HZ

def rms(array):
   return np.sqrt(np.mean(array ** 2))
   
window_size = 1.5 * loop_rate #[s] * [1/s]
data_buffer = np.zeros(int(window_size))
data_iter = 0
   
while True:
    current_msecs = supervisor.ticks_ms()
    elapsed_time = (current_msecs - previous_msecs)/1000
    if(elapsed_time >= (1.0/loop_rate)):
        #print("%.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f" % (imu.acceleration + imu.gyro))
        data_buffer[data_iter] = np.linalg.norm(imu.acceleration)
        data_iter = (data_iter + 1)%(int(window_size))
        print(data_buffer[-1])
        previous_msecs = current_msecs
