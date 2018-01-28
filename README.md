Driver of Ranging Sensor VL53L0X
================================

Codename: Sumaro(スマロ) from *From the Bridge of No.3*(第3艦橋より)

![Sumaro](docs/sumaro.jpg)

## Target
The firmware runs on an STM32 Nucleo64-F401RE board board.

## Content
The program is based on the demo of ChibiOS/RT port for ARM-Cortex-M4 STM32F401.

* `VL53L0X.c` copied from (vl53l0x-arduino
)[https://github.com/stevemarple/SoftWire] by pololu
* `softi2c.c` copied from (SoftWire for Arduino)[https://github.com/stevemarple/SoftWire] by stevemarple, but current not being used

## Instructions
1. Plug in VL53L0X boards
2. Turn on 3.3V Power Source
3. Plug in the USB of STM32 Nucleo64-F401RE to PC
4. Read from a serial port (Baudrate: 38400), you may use `screen /dev/ttyACM0 38400`

## Known Bug
* It won't automatically restart if you unplug a VL53L0X board and plug it again.
* It rely on external 3.3V power source, so to restart the VL53L0X boards, you have to restart the power source first.

## Build Procedure
The demo has been tested by using the free Codesourcery GCC-based toolchain
and YAGARTO.
Just modify the TRGT line in the makefile in order to use different GCC ports.

## Notes
Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com
