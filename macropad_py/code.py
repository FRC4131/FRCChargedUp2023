
"""code.py"""
import board
import time
from joystick_xl.inputs import Axis, Button, Hat
from joystick_xl.joystick import Joystick
from neopixel import NeoPixel
from keypad import Keys
import random
#import socket

NandG_colors = [0xffff00,0x800080,0xffff00,0xffff00,0x800080,0xffff00,0x008000,0x008000,0x008000,0x0000ff,0x00ffff,0x0000ff]
NandG_colors_blue = [0xffff00,0x800080,0xffff00,0xffff00,0x800080,0xffff00,0x008000,0x008000,0x008000,0x0000ff,0x00ffff,0x0000ff]
NandG_colors_red = [0xffff00,0x800080,0xffff00,0xffff00,0x800080,0xffff00,0x008000,0x008000,0x008000,0xff0000,0x80000f,0xff0000]
colors_red = [0xff0000,0xff0000,0xff0000,0xff0000,0xff0000,0xff0000,0xff0000,0xff0000,0xff0000,0xff0000,0xff0000,0xff0000]

def rng_clr():
    return random.choice([0xff0000,0x00ff00,0x0000ff,0x00ffff, 0xffff00,0x800080])

# Device control objects
js = Joystick()
px = NeoPixel(board.NEOPIXEL, 12)
kb = Keys([getattr(board, "KEY%d" % num) for num in range(1,13)], value_when_pressed=False, pull=True)

# State tracking for buttons and LEDs
states = [False]*12
grids = [False]*3
nodes = [False]*9

grid = 0
node = 0

konami = [2,2,8,8,4,6,4,6,9,7,5]
lastInputs = []

red = False

doubletap_cooldown = .15 # seconds

for idx in range(12):
    js.add_input(Button())
    js.button[idx].source_value = 1
js.update()

for i in range(12):
    px[i] = NandG_colors[i]

while True:
    key_event = kb.events.get()

    if key_event:

        if key_event.pressed:
            # Implement Konami code sequence
            # append current event to keypress history
            lastInputs += [key_event.key_number]
            # truncate history
            if (len(lastInputs) > len(konami)):
                lastInputs = lastInputs[-len(konami):]
            # If the history is long enough and all elements match,
            if len(konami) == len(lastInputs) and all(konami[idx]-1 == lastInputs[idx] for idx in range(len(konami))):
                # set everything to NandG_colors[i]
                for i in range(12):
                        px[i] = NandG_colors[i]
                # skip further processing
                continue
            
            # >8 is bottom row of keys
            if key_event.key_number > 8:
                # Implement press-release-press bottom row color swap
                # wait for second tap to come through
                time.sleep(doubletap_cooldown)
                # Flush previous changes to USB HID device (redundant with end-of-loop js.update())
                js.update()
                # Get next two keyboard events:
                # burn(?)
                no = kb.events.get()
                # second keypress
                yes = kb.events.get()
                # If there was a second keypress that came down this path,
                if yes and yes.pressed:
                    # change out the bottom row's colors for the other set
                    NandG_colors = NandG_colors_red if not red else NandG_colors_blue
                    # Toggle which color set is next
                    red = not red
                    # Set the LED colors
                    for i in range(12):
                        px[i] = NandG_colors[i]
                        js.button[i].source_value = 1
                    # reset grid and node selection
                    grid = 0
                    node = 0
                else:
                    # normal keypress
                    # Press all the bottom row buttons on the HID device
                    for i in range(9,12):
                        js.button[i].source_value = 1
                        px[i] = NandG_colors[i]
                    # Un-press the button corresponding with this event
                    js.button[key_event.key_number].source_value = 0
                    # Set the LED to white
                    px[key_event.key_number] = 0xffffff
                    # note the selected key globally as a grid number
                    grid = key_event.key_number - 8
            # top nine keys
            else: 
                # Press all the HID device buttons (in this range), set all LEDs to default colors
                for i in range(9):
                    js.button[i].source_value = 1
                    px[i] = NandG_colors[i]
                # Un-press the joystick button corresponding to the key
                js.button[key_event.key_number].source_value = 0
                # Set the LED to white
                px[key_event.key_number] = 0xffffff
                # note the selected key globally as a 1-based node number
                node = key_event.key_number + 1
        print(f"\n\nGRID {grid} selected\nNODE {node} selcted")
    # Push changes to the HID device
    js.update()
# End of 'while True' loop
