
The code in this folder is for an Adafruit Macropad RP2040.

This is a 12-key plus a knob keypad with color LEDs and a screen as output.

* Kit: https://www.adafruit.com/product/5128
* Minimum kit: https://www.adafruit.com/product/5100
* Tutorials page: https://learn.adafruit.com/adafruit-macropad-rp2040
* Hardware files: https://learn.adafruit.com/adafruit-macropad-rp2040/downloads

The device enumerates as a USB HID joystick with 12 buttons so the FIRST Driver
Station picks it up as an input device.

The device presents as a USB Mass Storage device (flash drive) that holds the
Python code it runs.

LEDs and screen printouts are used to track which keys have been pressed.

The Konami code is supported.

USB HID Joystick support is provided by the joystick_xl library:

* Docs: https://circuitpython-joystickxl.readthedocs.io/en/latest/intro.html
* Code: https://github.com/fasteddy516/CircuitPython_JoystickXL

# Setup instructions

1. Flash the latest CircuitPython runtime on the device: https://learn.adafruit.com/adafruit-macropad-rp2040/circuitpython
   * This code was written on CircuitPython 8.0.2 for the `adafruit_macropad_rp2040`.
2. Download the matching CircuitPython library set and copy the `lib` and
   `requirements` folders to the device's matching folders.
   * This takes a while, so theoretically someone could enumerate the minimum set
     and list it here.
3. Copy `boot.py` and `code.py` from this folder to the device.
4. After soft boot completes, reset the device with the button by the left edge
   of the screen.
