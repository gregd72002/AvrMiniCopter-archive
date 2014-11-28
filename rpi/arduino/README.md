AvrMiniCopter
=============

Arduino based copter driver.

This is just an arduino driver that handles stabilization and executes SPI received commands.

To make it work you will need:
- MPU6050/MPU9150 connected over I2C
- SPI master connected device (like Raspberry Pi) that will send commands to the driver
- 4 ESC connected to PWM ports



This has been tested on Atmega328p (16MHz).
