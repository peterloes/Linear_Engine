﻿# Linear Actuator

Platform for Animal Manipulation 

This is the firmware for the LinearA_2017 (linear servo, i.e. stepper motor with spindle)
and projects Servo_2015 (standard PWM servo).
Setup and control of the servo is done
via 3 keys, and, for selecting the two end positions, also via two external
inputs, please see below.  The firmware supports soft-start, i.e. the servo
speed will be ramped-up during the start phase.

Supported:
- Nanotec SMC11 Leistungsendstufe
- Nanotec Linear Aktuator L2018S0604-T3,5x1
 

![My image](https://github.com/peterloes/Linear_Engine/blob/master/1_Getting_Started_Tutorial/2_Electronic_board.jpg)


The linear setup procedure consists of 4 steps:
- Adjust end position 1.  For PWM servos this can be the most left or the
  most right end point.  This position will be reached after power-up if
  valid servo data has been found in FLASH.  In contrast, a linear actuator
  doesn't know about its absolute position, therefore it first moves to
  to its physical start position (near the motor) after power-up.  From
  that, position 1 can be adjusted.
- Adjust end position 2.  This is the opposite end position to position 1.
- Adjust servo speed.  The servo permanently moves between the two end
  points.  The speed can be adjusted via S1 and S2.
- Asserting S3 the 4th time stores the servo parameters into FLASH and
  returns to normal operation.

![My image](https://github.com/peterloes/Linear_Engine/blob/master/1_Getting_Started_Tutorial/1_MOMO_Shutter.jpg)

The Linear Actuator features EFM32 ...the world´s most energy friendly microcontrollers

ARM Cortex-M3 EFM32G230F128
