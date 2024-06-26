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
    
    led_blue = digitalio.DigitalInOut(board.LED_BLUE)
    led_blue.direction = digitalio.Direction.OUTPUT
    led_blue.value = True
    # Bugfix: before reading battery voltage a bluetoot-uart should be disconnected
    uart_connection = None
    @classmethod
    def get_uart(cls):
        return cls.uart_connection
    @classmethod
    def set_uart(cls,uart):
        cls.uart_connection = uart
    @classmethod
    def disconnect_uart(cls):
        if cls.uart_connection != None and cls.uart_connection.connected:
            cls.uart_connection.disconnect()
            return True
        return False
        
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
        if(self.uart_connection != None and self.uart_connection.connected):
            self.led_blue.value = False
        else:
            self.led_blue.value = True
        pass

class StateMachine(object):

    def __init__(self):
        self.state = None
        self.states = {}
        self.__previous_state = None

    def add_state(self, state):
        self.states[state.name] = state

    def go_to_state(self, state_name):
        if self.state:
            print('Exiting %s' % (self.state.name))
            if(self.__previous_state != self.state.name):
                self.__previous_state = self.state.name
            self.state.exit(self)
        self.state = self.states[state_name]
        print('Entering %s' % (self.state.name))
        self.state.enter(self)

    def get_previous_state(self):
        return self.__previous_state

    def update(self):
        if self.state:
            self.state.update(self)



class IdleState(State):
    loop_rate = 50 #[Hz]
    led_red = digitalio.DigitalInOut(board.LED_RED)
    led_green = digitalio.DigitalInOut(board.LED_GREEN)
    imu = IMU()
    window_time_size = 1 # [s]
    window_samples_size = window_time_size * loop_rate # [s] * [1/s]
    data_buffer = np.zeros(int(window_samples_size))
    accel_diff_threshold = 0.35 # [m/s^2]
    data_iter = 0
    previous_value = np.linalg.norm(imu.acceleration)
    __previous_msecs = time.monotonic()

    bat = Battery()
    battery_loop_rate = 0.004 # [Hz] every 250s about 4 minutes
    
    #battery_window_time_size = 5 # [s]
    #battery_window_samples_size = battery_window_time_size * battery_loop_rate # [s] * [1/s]
    #battery_data_buffer = np.zeros(int(battery_window_samples_size))
    #battery_data_iter = 0
    charged_battery_voltage = 4.1
    discharged_battery_voltage = 3.65

    def __init__(self):
        super().__init__()
        self.led_red.direction = digitalio.Direction.OUTPUT
        self.led_red.value = True
        self.led_green.direction = digitalio.Direction.OUTPUT
        self.led_green.value = True
        self.imu.gyro_data_rate = Rate.RATE_104_HZ
        self.imu.accelerometer_data_rate = Rate.RATE_104_HZ
        self.imu.high_pass_filter = True
        self.bat.charge_current = self.bat.CHARGE_100MA



    @property
    def name(self):
        return 'idle'

    def enter(self, machine):
        State.enter(self, machine)

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        #if switch.shaked:
        State.update(self,machine)
        current_msecs = time.monotonic()
        elapsed_time = (current_msecs - self.__previous_msecs)
        if(elapsed_time >= (1.0/self.loop_rate)):
            # Update timer
            self.__previous_msecs = current_msecs
            # Process data
            self.data_buffer[self.data_iter] = np.linalg.norm(self.imu.acceleration)
            #self.data_iter = (self.data_iter + 1)%(int(self.window_samples_size))

            if(self.data_iter == int(self.window_samples_size)-1):
                self.data_iter = 0
                accel_difference = np.max([0,rms(self.data_buffer) - self.previous_value])
                self.previous_value = rms(self.data_buffer)
                #print((accel_difference,accel_difference))
                if(accel_difference > self.accel_diff_threshold):
                    machine.go_to_state('shorting')
            else:
                self.data_iter = self.data_iter + 1

class PausedState(IdleState):
    def __init__(self):
        pass

    @property
    def name(self):
        return 'paused'

    def enter(self, machine):
        if(self.bat.voltage <= self.discharged_battery_voltage):
            machine.go_to_state('idle-discharged')
        else:
            machine.go_to_state('idle-charged')

    def exit(self, machine):
        pass

    def update(self, machine):
        pass

class IdleDischargedState(IdleState):
    def __init__(self):
        super().__init__()
        self.__previous_msecs_discharged = time.monotonic()

    @property
    def name(self):
        return 'idle-discharged'

    def enter(self, machine):
        self.led_green.value = True
        self.led_red.value = False

    def exit(self, machine):
        pass

        
    def update(self, machine):
        super().update(machine)
        current_msecs = time.monotonic()
        elapsed_time = (current_msecs - self.__previous_msecs_discharged)
        
        if(elapsed_time >= (1.0/self.battery_loop_rate)):
            # Update timer
            self.__previous_msecs_discharged = current_msecs
            # Process data
            #uart_connection = State.get_uart()
            #if uart_connection != None and uart_connection.connected:
            #    uart_connection.disconnect()
            #    print('Disconnected bluetooth-uart')
            if(State.disconnect_uart() == True):
                print('Disconnected bluetooth-uart')
            bat_voltage = self.bat.voltage
            print("Discharged battery: ", bat_voltage)
            if(bat_voltage > self.charged_battery_voltage):
                machine.go_to_state('idle-charged')


class IdleChargedState(IdleState):
    def __init__(self):
        super().__init__()
        self.__previous_msecs_charged = time.monotonic()

    @property
    def name(self):
        return 'idle-charged'

    def enter(self, machine):
        self.led_green.value = False
        self.led_red.value = True

    def exit(self, machine):
        pass


    def update(self, machine):
        super().update(machine)
        current_msecs = time.monotonic()
        elapsed_time = (current_msecs - self.__previous_msecs_charged)
        if(elapsed_time >= (1.0/self.battery_loop_rate)):
            # Update timer
            self.__previous_msecs_charged = current_msecs
            # Process data
            if(State.disconnect_uart() == True):
                print('Disconnected bluetooth-uart')
            bat_voltage = self.bat.voltage
            print("Charged battery: ", bat_voltage)
            if(bat_voltage < self.discharged_battery_voltage):
                machine.go_to_state('idle-discharged')
            
class ShortingState(State):

    def __init__(self):
        super().__init__()
        self.motor = digitalio.DigitalInOut(board.D0)
        self.motor.direction = digitalio.Direction.OUTPUT
        self.vibration_time = 8 # [s]
        self.previous_msecs = time.monotonic()
        self.ble = BLERadio()
        
    @property
    def name(self):
        return 'shorting'
        
    def enter(self, machine):
        State.enter(self, machine)
        if not self.ble.connected:
            print("Trying to connect...")
            for adv in self.ble.start_scan(ProvideServicesAdvertisement,buffer_size=128, timeout = 1): # ProvideServicesAdvertisement
                if(adv.complete_name == "Minutnik"):
                    if UARTService in adv.services:
                        State.set_uart(self.ble.connect(adv))
                        print("Connected to " + adv.complete_name)
                        break
            self.ble.stop_scan()
        if self.ble.connected:
            uart_connection = State.get_uart()
            if(uart_connection.connected):
                uart_service = uart_connection[UARTService]
                msg = "#cs1*" # command shorting 1
                uart_service.write(msg.encode("utf-8"))
                print("Shorting command has been sent")
                self.led_blue.value = False
            else:
                # TODO
                print('Uart disconnected')
        else:
            self.led_blue.value = True
        self.previous_msecs = time.monotonic()
        self.motor.value = True


    def exit(self, machine):
        State.exit(self, machine)


    def update(self, machine):
        if(time.monotonic()  - self.previous_msecs > self.vibration_time):
            self.motor.value = False
            if(machine.get_previous_state() != None):
                machine.go_to_state(machine.get_previous_state())





machine = StateMachine()
machine.add_state(ShortingState())
machine.add_state(IdleChargedState())
machine.add_state(IdleDischargedState())
machine.add_state(PausedState())
machine.go_to_state('paused')
while True:
    machine.update()
