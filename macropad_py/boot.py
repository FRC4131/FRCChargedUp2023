
"""boot.py"""
import usb_hid
from joystick_xl.hid import create_joystick

# Enable USB HID emulation, add 127-button 'joystick' object to USB descriptor
usb_hid.enable((create_joystick(axes=0, buttons=127, hats=0),))
