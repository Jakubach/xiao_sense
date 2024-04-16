from time import sleep
from seeed_xiao_nrf52840 import IMU
import board
import digitalio
import supervisor
from adafruit_lsm6ds import Rate
from ulab import numpy as np
from audiocore import WaveFile
from audiopwmio import PWMAudioOut as AudioOut
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

def rms(array):
   return np.sqrt(np.mean(array ** 2))

class State(object):

    def __init__(self):
        pass

    @property
    def name(self):
        return ''

    def enter(self, machine):
        pass

    def exit(self, machine):
        pass

    def update(self, machine):
        pass

class StateMachine(object):

    def __init__(self):
        self.state = None
        self.states = {}

    def add_state(self, state):
        self.states[state.name] = state

    def go_to_state(self, state_name):
        if self.state:
            print('Exiting %s' % (self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        print('Entering %s' % (self.state.name))
        self.state.enter(self)

    def update(self):
        if self.state:
            #print('Updating %s' % (self.state.name))
            self.state.update(self)

class IdleState(State):

    def __init__(self):
        self.previous_msecs = supervisor.ticks_ms()
        self.loop_rate = 50 #[Hz]
        self.imu = IMU()
        self.imu.gyro_data_rate = Rate.RATE_104_HZ
        self.imu.accelerometer_data_rate = Rate.RATE_104_HZ

        self.window_time_size = 1.5 # [s]
        self.window_samples_size = self.window_time_size * self.loop_rate # [s] * [1/s]
        self.data_buffer = np.zeros(int(self.window_samples_size))
        self.accel_diff_threshold = 0.5 # [m/s^2]
        self.data_iter = 0
        self.imu.high_pass_filter = True
        self.previous_value = np.linalg.norm(self.imu.acceleration)
        self.led = digitalio.DigitalInOut(board.LED)
        self.led.direction = digitalio.Direction.OUTPUT


    @property
    def name(self):
        return 'idle'

    def enter(self, machine):
        State.enter(self, machine)
        self.previous_msecs = supervisor.ticks_ms()
        #self.previous_value = np.linalg.norm(self.imu.acceleration)

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        #if switch.shaked:
        current_msecs = supervisor.ticks_ms()
        elapsed_time = (current_msecs - self.previous_msecs)/1000
        if(elapsed_time >= (1.0/self.loop_rate)):
            # Update timer
            self.previous_msecs = current_msecs
            # Process data
            self.data_buffer[self.data_iter] = np.linalg.norm(self.imu.acceleration)
            self.data_iter = (self.data_iter + 1)%(int(self.window_samples_size))
            if(self.data_iter == int(self.window_samples_size)-1):
                self.led.value = not self.led.value
                accel_difference = np.max([0,rms(self.data_buffer) - self.previous_value])
                self.previous_value = rms(self.data_buffer)
                print((accel_difference,accel_difference))
                if(accel_difference > self.accel_diff_threshold):
                    machine.go_to_state('shorting')

class ShortingState(State):

    def __init__(self):
        self.motor = digitalio.DigitalInOut(board.D0)
        self.motor.direction = digitalio.Direction.OUTPUT
        self.vibration_time = 8 # [s]
        self.previous_msecs = supervisor.ticks_ms()
        self.ble = BLERadio()
        self.uart_connection = None

    @property
    def name(self):
        return 'shorting'

    def enter(self, machine):
        State.enter(self, machine)
        if not self.uart_connection:
            print("Trying to connect...")
            for adv in self.ble.start_scan(ProvideServicesAdvertisement):
                if UARTService in adv.services:
                    self.uart_connection = self.ble.connect(adv)
                    break
            self.ble.stop_scan()
        else:
            self.previous_msecs = supervisor.ticks_ms()
            self.motor.value = True


    def exit(self, machine):
        State.exit(self, machine)


    def update(self, machine):
        if self.uart_connection and self.uart_connection.connected:
            if(supervisor.ticks_ms()  - self.previous_msecs > (self.vibration_time * 1000)):
                self.motor.value = False
                machine.go_to_state('idle')





machine = StateMachine()
machine.add_state(ShortingState())
machine.add_state(IdleState())
machine.go_to_state('idle')

while True:
    machine.update()
