"""Example for Seeed Studio XIAO nRF52840. Blinks the built-in LED."""
import board
import digitalio
import supervisor

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

    
led_state = False
previous_msecs = supervisor.ticks_ms()
loop_rate = 1 #[Hz]

while True:
    current_msecs = supervisor.ticks_ms()
    elapsed_time = (current_msecs - previous_msecs)/1000
    if(elapsed_time >= (1.0/loop_rate)):
        led_state = not led_state
        led.value = led_state
        previous_msecs = current_msecs




