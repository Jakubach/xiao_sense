from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
import digitalio
import board
import time
from seeed_xiao_nrf52840 import Battery
import analogio

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
    ble = BLERadio()
    uart = UARTService()
    advertisement = ProvideServicesAdvertisement(uart)
    

    loop_rate = 50 #[Hz]
    led_red = digitalio.DigitalInOut(board.LED_RED)
    led_green = digitalio.DigitalInOut(board.LED_GREEN)

    __previous_msecs = time.monotonic()

    bat = Battery()
    battery_loop_rate = 0.004 # [Hz] every 250s about 4 minutes
    charged_battery_voltage = 4.15
    discharged_battery_voltage = 3.55
    


    def __init__(self):
        super().__init__()
        self.led_red.direction = digitalio.Direction.OUTPUT
        self.led_red.value = True
        self.led_green.direction = digitalio.Direction.OUTPUT
        self.led_green.value = True
        self.bat.charge_current = self.bat.CHARGE_100MA
        self.advertisement.complete_name = "Minutnik"
        self.led_blue = digitalio.DigitalInOut(board.LED_BLUE)
        self.led_blue.direction = digitalio.Direction.OUTPUT
        self.led_blue.value = True
        self._ble_connection_flag = False
        
    @property
    def name(self):
        return 'idle'

    def enter(self, machine):
        State.enter(self, machine)
        self.__previous_msecs = time.monotonic()

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if not self.ble.connected:
            self._ble_connection_flag = False
            if not self.ble.advertising:
                self.ble.start_advertising(self.advertisement)
                print("Waiting to connect")
        else:
            if not self._ble_connection_flag:
                print("Connected")
                self.led_blue.value = False
                if self.ble.advertising:
                    self.ble.stop_advertising()
            if self.uart.in_waiting:
                s = self.uart.read(self.uart.in_waiting).decode("utf-8")
                if s == "#cs1*":
                    machine.go_to_state('shorting')
            self._ble_connection_flag = True
        if not self.ble.connected:
            self.led_blue.value = True
            self._ble_connection_flag = False
            
            


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
        self.__previous_msecs_discharged = time.monotonic()

    @property
    def name(self):
        return 'idle-discharged'

    def enter(self, machine):
        self.led_green.value = True
        self.led_red.value = False
        self.__previous_msecs_discharged = time.monotonic()
        #pass

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
            #if(State.disconnect_uart() == True):
            #    print('Disconnected bluetooth-uart')
            print("Discharged battery: ", self.bat.voltage)
            if(self.bat.voltage > self.charged_battery_voltage):
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
        self.__previous_msecs_charged = time.monotonic()

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
            #if(State.disconnect_uart() == True):
            #    print('Disconnected bluetooth-uart')
            print("Charged battery: ", self.bat.voltage)
            if(self.bat.voltage < self.discharged_battery_voltage):
                machine.go_to_state('idle-discharged')
        
class ShortingState(IdleState):
    def __init__(self):
        self._timing_set = analogio.AnalogIn(board.D0)
        self._max_timing = 60 # [s]
        self._ref_voltage = 3.3 #[V]
        self._mosfets = digitalio.DigitalInOut(board.D1)
        self._mosfets.direction = digitalio.Direction.OUTPUT
        self.previous_msecs = time.monotonic() 
        #self.previous_msecs_test = time.monotonic() 

    @property
    def name(self):
        return 'shorting'
        
    @property
    def voltage(self) -> float:
        value = (self._timing_set.value / 65535.0) * 3.3
        return value
    
    @property
    def activation_time(self) -> float:
        return (self._max_timing / self._ref_voltage) * self.voltage
        

    def enter(self, machine):
        self.previous_msecs = time.monotonic()
        self._mosfets.value = True

    def exit(self, machine):
        pass

    def update(self, machine):
        #print(self.activation_time)
        #current_msecs = time.monotonic()
        #elapsed = current_msecs - self.previous_msecs_test
        if(time.monotonic()  - self.previous_msecs > self.activation_time):
            self._mosfets.value = False
            if(machine.get_previous_state() != None):
                machine.go_to_state(machine.get_previous_state())
        #if(elapsed > 1):
        #    print(self.activation_time)
        #    self.previous_msecs_test = current_msecs
        

        
machine = StateMachine()
machine.add_state(ShortingState())
machine.add_state(IdleChargedState())
machine.add_state(IdleDischargedState())
machine.add_state(PausedState())
machine.go_to_state('paused')

while True:
    machine.update()
    

