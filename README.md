# dekacontroller-firmware

Firmware for my [dekaclock](https://github.com/grob6000/dekaclock) - see there for schematics for the board and what it does!

The code is Arduino C++ (might be a bit of AVR GCC thrown in there). Using arduino 1.8.19 & vscode 1.74.1

You'll need the following libraries:
 - [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
 - [Adafruit SH110X Library](https://github.com/adafruit/Adafruit_SH110X)
 - [Adafruit NeoPixel Library](https://github.com/adafruit/Adafruit_NeoPixel)

Note the crystal on the board I'm using is 18.432MHz, not the usual 12MHz - that took some mucking around to configure correctly in vscode. Suggest you stick to 12MHz if you can...

Note that this hogs nearly all the RAM in the ATMEGA328 (2kiB), as 1300kiB or so is used for the graphics buffer alone. I had to strip some things back (like the drift buffer data) to stop it blowing out the stack.

## gpsemulator.py

A tool to emulate a NEMA serial GPS device using your computer. Based on the module I used for this project, a Neo6M.

### Usage:

`python gpsemulator.py COM3`
where COM3 is replaced with the port to emulate on!
