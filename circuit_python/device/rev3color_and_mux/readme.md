# Read Me

## Description

This code will read REV V3 Color sensors on a Raspberry Pico utilizing i2c through a MUX and will write a CSV output containing:

- Red Value
- Blue Value
- Green Value
- Infrared Value
- Proximity Value ( Larger the closer the object )

It is written using circuitpython and libraries from Adafruit.  

## Installation

- Hold down the bootsel button while attaching the raspberry pico to a computer.
- There will be a RPI volume mounted
- If not, use another USB cable.  Some are only useable for charging, and not data.
- Drag a circuitpython image onto the pico.
- The volume will disappear and after a couple of minutes, a CIRCUITPYTHON volume will appear
- Move the boot.py, code.py, and library onto the volume.
- The code should immediately reload after changes, and immediately start producing output.
- The mux board needs to be attached, because the code will check for and error if pullup resistors are not attached.
- GP4/GP5 should be connected to SDA/SCL respectively.
- On the can mux board, SDA is nearest the chip followed by SCL, followed by 3.3V, followed by Ground.
- One color sensor must be attached to the 0 channel on the mux at the start.  Otherwise, there will be an error.
- From the mux board to the sensor, the connections from the chip outward are: GND, 3.3V, SCL, SDA. Following names on rev docs.


## Changing the script

Connecting a pico to any computer will cause the raspberry pico to present a small usb flash drive named "CIRCUITPYTHON" 

You can modify the script to change parameters noted in the Broadcom data sheet.
There are dictionaries with the hexadecimal value required to change behaviours in code.py

Circuit python requires the script be names code.py

You can also change the number of sensors polled.  Currently 3 are defined in code, because 3 were on the robot.
All variables that should be treated as constants are prefixed by k.

## References
Based off of: https://github.com/ThadHouse/picocolorsensor

https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html pinouts and information


https://www.revrobotics.com/rev-31-1557/ REV V3 color sensor
https://docs.revrobotics.com/color-sensor/ - docs and drawings
https://docs.broadcom.com/doc/APDS-9151-DS - Broadcom data sheet for color sensor included in REV robotics V3 color sensor.


https://circuitpython.org - Library including uf2 image for raspberry pico and libraries
https://learn.adafruit.com/welcome-to-circuitpython/installing-mu-editor Editor for circuitpython that also has a serial USB driver
https://learn.adafruit.com/circuitpython-essentials/circuitpython-i2c - Description of how to use i2c with circuitpython
https://docs.circuitpython.org/projects/busdevice/en/latest/_modules/adafruit_bus_device/i2c_device.html detailed documentation
