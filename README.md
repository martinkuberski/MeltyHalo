# MeltyBrian - a MeltyHalo (by swallenhardware) implementation 
An implementation of MeltyHalo, Meltybrain software for Teensy 3.2 by swallenhardware (Spencer Allen) for an accelerometer only robot controlled by Flysky FS-I6 (with FS-iA6B receiver).
Remember to remove unnecessary headers from Adafruit_BusIO library as they are incompatible with i2c - leave only Adafruit_SPIDevice.h and Adafruit_SPIDevice.cpp
Many thanks to the original authors of MeltyHalo!
# Original Description
Arduino code to control my MeltyBrain battlebot

This Arduino project is the control code for Halo, the beetleweight Battlebot designed by Spencer Allen and Pierce Jensen.

The full writeup for this project can be found at https://www.swallenhardware.io/battlebots/


Current functionality:

-Can communicate with the controller two-way over serial

-Can drive the brushless ESCs

-Can operate as a ram bot with standard arcade controls

-Can receive and debounce IR beacon pulses

-Can measure rotational speed using the accelerometer

-Can translate in spin mode using beacon, accelerometer, and hybrid sensing

-Uses the accelerometer to account for failures in the beacon, and vice-versa.

-Automatically falls back to beacon-only motion if the accelerometer fails, and can be set to accelerometer-only if the beacon becomes unreliable.

-Operates a POV display while in spin mode

-Incorporates safety features such as dead-man switch, loss of comms shutdowns, and a watchdog timer.


TODO:

-Finish support for manual trim adjust
