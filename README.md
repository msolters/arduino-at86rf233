# arduino-at86rf233
Simple Arduino driver for the AT86RF233 802.15.4 radio module

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

AT86RF233 (REB X-Pro Extension Header) | Arduino Uno
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

## Driver Configuration
Without further configuration, this library will use the default AT86RF2xx parameters contained in `at86rf2xx_defaults.h`.  These defaults can be overridden inside `at86rf2xx_config.h`.  For example, to change the radio channel to `25`, you could use the following `at86rf2xx_config.h`:

```c
#define AT86RF2XX_CHANNEL               25  //AT86RF2XX_DEFAULT_CHANNEL
#define AT86RF2XX_TXPOWER               AT86RF2XX_DEFAULT_TXPOWER
#define AT86RF2XX_ADDR_LONG             AT86RF2XX_DEFAULT_ADDR_LONG
#define AT86RF2XX_ADDR_SHORT            AT86RF2XX_DEFAULT_ADDR_SHORT
#define AT86RF2XX_PANID                 AT86RF2XX_DEFAULT_PANID
```

Most radio parameters can also be modified after the fact using the set/getters enumerated in `at86rf2xx_getset.c`.
