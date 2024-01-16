
"""code.py"""
import board
from time import sleep
from joystick_xl.inputs import Axis, Button, Hat
from joystick_xl.joystick import Joystick
from neopixel import NeoPixel
from keypad import Keys
import random

def rng_clr():
    return random.choice([0xff0000,0x00ff00,0x0000ff,0x008000,0x800080,0x000080])

js = Joystick()
px = NeoPixel(board.NEOPIXEL, 12)
kb = Keys([getattr(board, "KEY%d" % num) for num in range(1,13)], value_when_pressed=False, pull=True)

states = [False]*12

for idx in range(12):
    js.add_input(Button())
    js.button[idx].source_value = 1
js.update()

while True:
    key_event = kb.events.get()
    if key_event:
        if key_event.pressed:
            states[key_event.key_number] = not states[key_event.key_number]
            js.button[key_event.key_number].source_value = 0 if states[key_event.key_number] else 1
        if key_event.released:
            px[key_event.key_number] = rng_clr() if states[key_event.key_number] else 0x000000
    js.update()
