# arduino-at86rf233
Simple Arduino driver for the AT86RF233 802.15.4 radio module

[arduino-at86rf233.ino](https://github.com/msolters/arduino-at86rf233/blob/master/arduino-at86rf233.ino) demonstrates the simplest example application -- an Arduino sketch that scans for nearby traffic and prints incoming frames over serial (115200 baud).

## Radio Hardware
Designed to be used with ATMEL's AT86RF233 802.15.4 radio, available as a $30USB extension board.

![](http://media.digikey.com/Photos/Atmel%20Photos/ATREB233-XPRO.JPG)


## Connect the AT86RF233 to the Arduino
In this example, we will be using an Arduino Uno.  You can use basically any MCU with a sufficient clock speed, and at least:

*  1 SPI bus
*  VCC (3.3V) & GND
*  3 GPIO ouput
*  1 interruptible input

Here's the wiring setup.  You may find the Uno pinout reference helpful.

AT86RF233 (REB) | Arduino Uno
---|---
7 | 8 (digital) (Reset)
9 | 2 (digital) (Interrupt)
10 | 7 (digital) (Sleep)
15 | 6 (digital) (CS)
16 | MOSI (ICSP header)
17 | MISO (ICSP header)
18 | SCK (ICSP header)
19 | GND
20 | VCC (3.3V)

![](http://www.gammon.com.au/images/ArduinoUno_R3_Pinouts.png)
