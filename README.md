# arduino-at86rf233
Simple Arduino driver for the AT86RF233 802.15.4 radio module

This is a C++ port of the AT86RF233 driver from [RIOT-OS](https://github.com/RIOT-OS/RIOT)

[arduino-at86rf233.ino](https://github.com/msolters/arduino-at86rf233/blob/master/arduino-at86rf233.ino) demonstrates the simplest example application -- an Arduino sketch that scans for nearby traffic and prints incoming frames over serial (115200 baud).

## Radio Hardware
Designed to be used with ATMEL's AT86RF233 802.15.4 radio, [available as a $30USB radio extension board (REB)](http://www.mouser.com/ProductDetail/Atmel/ATREB233-XPRO/?qs=HVbQlW5zcXX%2FEgqNxRIBfA%3D%3D).

![](http://media.digikey.com/Photos/Atmel%20Photos/ATREB233-XPRO.JPG)

## Connect the AT86RF233 to the Arduino
In this example, we will be using an Arduino Uno.  You are free to use any MCU with a sufficient clock speed, and at least:

*  1 SPI bus
*  VCC (3.3V) & GND
*  3 GPIO ouput
*  1 interruptible GPIO input

Here's the wiring setup.  You may find the Uno pinout reference helpful (included below).

Purpose | AT86RF233 (REB X-Pro Extension Header) | Arduino Uno
---|---|---
Reset | 7 | 8
Interrupt | 9 | 2
Sleep | 10 | 7
CS | 15 | 6
MOSI | 16 | MOSI (ICSP header)
MISO | 17 | MISO (ICSP header)
SCK | 18 | SCK (ICSP header)
Ground | 19 | GND
Power | 20 | VCC (3.3V)

![](http://www.gammon.com.au/images/ArduinoUno_R3_Pinouts.png)

## Driver Configuration
Without further configuration, this library will use the default AT86RF2xx parameters contained in `at86rf2xx-defaults.h`.  Most radio parameters can be updated manually by using the set/getters enumerated in `at86rf2xx-getset.c`.  For example, to change the radio channel to `25`, simply call `set_chan(25)` on the at86rf2xx device after initialization:

```cpp
  at86rf2xx.init(SEL, IRQ, SLP_TR, RESET);
  at86rf2xx.set_chan(25); // set channel to 25
```
