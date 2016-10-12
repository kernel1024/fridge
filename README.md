# fridge
Controller firmware for fridge thermostat, based on Arduino Nano.

This project based on code from JimmyN ( http://www.homebrewtalk.com/showthread.php?t=426252 ) with few changes and adjustments.

# system requirements
* Arduino Nano with Optiboot bootloader
* DHT22 temperature sensor (mounted inside of fridge in place of old mechanical thermostat)
* 40DA solid state relay (mounted near compressor and power terminal)
* 5V/1A power supply
* two buttons
* 1602 LCD with I2C adaptor

DHT sensors works well with near-zero temperatures with 0.1 precision. Available temperature setpoint range, choosen for this fridge: -10..10°C, with 0.1°C steps.
Also optiboot required for AVR integrated watchdog support.
