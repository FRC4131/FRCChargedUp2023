# Write your code here :-)

"""code.py"""
import board
from time import sleep
from joystick_xl.inputs import Axis, Button, Hat
from joystick_xl.joystick import Joystick
from neopixel import NeoPixel
from keypad import Keys
import random

js = Joystick()
px = NeoPixel(board.NEOPIXEL, 12)
kb = Keys([getattr(board, "KEY%d" % num)
          for num in range(1, 13)], value_when_pressed=False, pull=True)

for idx in range(12):
    js.add_input(Button())
    js.button[idx].source_value = 0
js.update()

while True:
    key_event = kb.events.get()
    if key_event:
        if key_event.pressed:
            js.button[key_event.key_number].source_value = 1
            px[key_event.key_number] = random.choice(
                [0x00FFFF, 0x0000FF, 0x008000, 0x00FF00, 0x000080, 0x808000, 0x800080, 0x008080,	0xFFFFFF,	0xFFFF00, 0xFF0000])
        else:
            js.button[key_event.key_number].source_value = 0
            px[key_event.key_number] = 0x000000
    js.update()
