import time
from seeed_xiao_nrf52840 import IMU, Battery
import board
import digitalio
from adafruit_lsm6ds import Rate
from ulab import numpy as np
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
            self.state.update(self)

class IdleState(State):

    def __init__(self):
        self.bat = Battery()
        self.bat.charge_current = self.bat.CHARGE_100MA
        self.previous_msecs = time.monotonic()
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
        self.led_red = digitalio.DigitalInOut(board.LED_RED)
        self.led_red.direction = digitalio.Direction.OUTPUT
        self.led_red.value = True
        self.led_green = digitalio.DigitalInOut(board.LED_GREEN)
        self.led_green.direction = digitalio.Direction.OUTPUT
        self.led_green.value = True
        self.charged = None
        self.discharged_battery_voltage = 3.25
        self.charged_battery_voltage = 3.6


    @property
    def name(self):
        return 'idle'

    def enter(self, machine):
        State.enter(self, machine)
        self.previous_msecs = time.monotonic()
        if(self.charged == None):
            if(self.bat.voltage >= self.discharged_battery_voltage):
                self.charged = True
            else:
                self.charged = False
        #self.previous_value = np.linalg.norm(self.imu.acceleration)

    def exit(self, machine):
        State.exit(self, machine)

    def battery_monitor(self, bat_voltage):
        if(bat_voltage > self.discharged_battery_voltage and bat_voltage < self.discharged_battery_voltage):
            if(self.charged):
                self.led_green.value = False
                self.led_red.value = True
            else:
                self.led_green.value = True
                self.led_red.value = False
        elif(bat_voltage >= self.charged_battery_voltage):
            self.led_green.value = False
            self.led_red.value = True
            self.charged = True
        elif(bat_voltage <= self.discharged_battery_voltage):
            self.led_green.value = True
            self.led_red.value = False
            self.charged = False
            
    def update(self, machine):
        #if switch.shaked:
        current_msecs = time.monotonic()
        elapsed_time = (current_msecs - self.previous_msecs)
        if(elapsed_time >= (1.0/self.loop_rate)):
            # Update timer
            self.previous_msecs = current_msecs
            # Process data
            try:
                self.data_buffer[self.data_iter] = np.linalg.norm(self.imu.acceleration)
                self.data_iter = (self.data_iter + 1)%(int(self.window_samples_size))
                if(self.data_iter == int(self.window_samples_size)-1):
                    print(f"Voltage: {self.bat.voltage}")
                    self.battery_monitor(self.bat.voltage)
                    accel_difference = np.max([0,rms(self.data_buffer) - self.previous_value])
                    self.previous_value = rms(self.data_buffer)
                    print((accel_difference,accel_difference))
                    if(accel_difference > self.accel_diff_threshold):
                        machine.go_to_state('shorting')
            except:
                print('Exception in idle or shorting state')
                pass

class ShortingState(State):

    def __init__(self):
        self.motor = digitalio.DigitalInOut(board.D0)
        self.motor.direction = digitalio.Direction.OUTPUT
        self.vibration_time = 8 # [s]
        self.previous_msecs = time.monotonic()
        self.ble = BLERadio()
        self.uart_connection = None
        self.led_blue = digitalio.DigitalInOut(board.LED_BLUE)
        self.led_blue.direction = digitalio.Direction.OUTPUT
        self.led_blue.value = True

    @property
    def name(self):
        return 'shorting'

    def enter(self, machine):
        State.enter(self, machine)
        if not self.ble.connected:
            print("Trying to connect...")
            for adv in self.ble.start_scan(ProvideServicesAdvertisement,timeout = 1):
                if(adv.complete_name == "Minutnik"):
                    if UARTService in adv.services:
                        self.uart_connection = self.ble.connect(adv)
                        print("Connected to " + adv.complete_name)
                        break
            self.ble.stop_scan()
        if self.ble.connected:
            uart_service = self.uart_connection[UARTService]
            msg = "#cs1*" # command shorting 1
            uart_service.write(msg.encode("utf-8"))
            print("Shorting command has been sent")
            self.led_blue.value = False
        else:
            self.led_blue.value = True
        self.previous_msecs = time.monotonic()
        self.motor.value = True


    def exit(self, machine):
        State.exit(self, machine)


    def update(self, machine):
        if(time.monotonic()  - self.previous_msecs > self.vibration_time):
            self.motor.value = False
            machine.go_to_state('idle')





machine = StateMachine()
machine.add_state(ShortingState())
machine.add_state(IdleState())
machine.go_to_state('idle')

while True:
    machine.update()
