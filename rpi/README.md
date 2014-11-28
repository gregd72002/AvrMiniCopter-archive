# Welcome to the AvrMiniCopter wiki!

AvrMiniCopter is a fully featured, open-source quadcopter controller module designed to work with Raspberry Pi. In the current state it supports quadcopters in X configuration.

The vision throughout the project was to create a controller that can be easily incorporated into any existing project. As such you do not need hacking skills or be an embedded system geek to control and program your quadcopter. 

You will need a module board (i.e. Arduino Pro Mini) and Raspberry Pi or similar (SPI capable) board. The module (AVR) only activity is to read sensors and stabilise the quadcopters while Raspberry Pi is free to do any other tasks. You can play around with Raspberry Pi and program it in any way you want without affecting stability of your quadcopter. You can use any language like python, c/c++, java or even bash scripts without worrying about performance! As an example Raspberry Pi can do 3g communication, video recording, charting and waypoint navigation all in parallel while the module will take care of the quadcopter itself.

## Features
* 2 fly modes - Auto-level & Acro
* Altitude hold
* learner mode (attach 2 controller simultaneously - one for the teacher and one for the learner)
* web-based management portal for adjusting PIDs (use your bluetooth phone/tablet to configure, tune it, play videos and watch pictures, review flight logs & debug)
* log charting
* Real time charting
* ESC calibration
* Raspberry Pi camera support
* Bluetooth and USB controller support (i.e. PS3 gamepad)
* MPU6050 / MPU9150 gyro
* BMP085 / BMP180 barometer

## Roadmap
* support for WIFI controllers (Android & IOS)
* GPS & Waypoint navigation

## Requirements
* AVR board (i.e. Arduino Pro mini 16Mhz, 32kb) - it's a bargain on ebay
* MPU6050 or MPU9150 gyro board
* SPI capable board (tested on: Raspberry Pi and Odroid-W) + optional camera
* BMP085 / BMP180 barometer board
* Bluetooth USB dongle & PS3 controller (you can get away without them and use keyboard if you really want)

## Media
* Video: https://www.youtube.com/watch?v=lyR4wM2C4sA
* Pictures: https://github.com/rpicopter/AvrMiniCopter-images/tree/master/images


## Resources
* Wiki main: https://github.com/rpicopter/AvrMiniCopter/wiki
* How does it work: https://github.com/rpicopter/AvrMiniCopter/wiki/How-does-it-work
* Getting started: https://github.com/rpicopter/AvrMiniCopter/wiki/Getting-started
* Wiring: https://github.com/rpicopter/AvrMiniCopter/wiki/Wiring
* History: https://github.com/rpicopter/AvrMiniCopter/wiki/History
* My quadcopter: https://github.com/rpicopter/AvrMiniCopter/wiki/My-Quadcopter

## Inspiration
* Dr Gareth Owen (http://ghowen.me/build-your-own-quadcopter-autopilot/)
* Vincent Jaunet (https://github.com/vjaunet/QUADCOPTER)

## Build instructions (WIP)
* Please refer to Resources above - Getting started
* RPi folder: issue make
* Arduino foler: you will need Arduino-Makefile (https://github.com/sudar/Arduino-Makefile) and Arduino libraries 1.0.5
* Image folder: work in progress

## License
Copyright (c) 2014 Gregory Dymarek (gregory.dymarek@gmail.com) under MIT License (MIT)

See license file.

