"""Example for Seeed Studio XIAO nRF52840. Blinks the built-in LED."""
import time
import board
from audiocore import WaveFile
from audiopwmio import PWMAudioOut as AudioOut
        
wave_file = open("StreetChicken.wav", "rb")
wave = WaveFile(wave_file)
audio = AudioOut(board.A0)

while True:
    audio.play(wave)
    # This allows you to do other things while the audio plays!
    t = time.monotonic()
    while time.monotonic() - t < 6:
        pass




