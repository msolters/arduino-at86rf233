/*
 * Copyright (C) 2013 Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_at86rf2xx
 * @{
 *
 * @file
 * @brief       Implementation of driver internal functions
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Mark Solters <msolters@gmail.com>
 *
 * @}
 */

#include <SPI.h>

#include "at86rf2xx.h"
#include "at86rf2xx_internal.h"

void at86rf2xx_reg_write(const uint8_t addr,
                         const uint8_t value)
{
    byte writeCommand = addr | AT86RF2XX_ACCESS_REG | AT86RF2XX_ACCESS_WRITE;
    digitalWrite(at86rf2xx_dev.cs_pin, LOW);
    SPI.transfer(writeCommand);
    SPI.transfer(value);
    digitalWrite(at86rf2xx_dev.cs_pin, HIGH);
}

uint8_t at86rf2xx_reg_read(const uint8_t addr)
{
    byte value;
    byte readCommand = addr | AT86RF2XX_ACCESS_REG | AT86RF2XX_ACCESS_READ;
    digitalWrite(at86rf2xx_dev.cs_pin, LOW);
    SPI.transfer(readCommand);
    value = SPI.transfer(0x00);
    digitalWrite(at86rf2xx_dev.cs_pin, HIGH);

    return (uint8_t)value;
}

void at86rf2xx_sram_read(const uint8_t offset,
                         uint8_t *data,
                         const size_t len)
{
    byte readCommand = AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_READ;
    digitalWrite(at86rf2xx_dev.cs_pin, LOW);
    SPI.transfer(readCommand);
    SPI.transfer((char)offset);
    for (int b=0; b<len; b++) {
      data[b] = SPI.transfer(0x00);
    }
    digitalWrite(at86rf2xx_dev.cs_pin, HIGH);
}

void at86rf2xx_sram_write(const uint8_t offset,
                          const uint8_t *data,
                          const size_t len)
{
    byte writeCommand = AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_WRITE;
    digitalWrite(at86rf2xx_dev.cs_pin, LOW);
    SPI.transfer(writeCommand);
    SPI.transfer((char)offset);
    for (int b=0; b<len; b++) {
      SPI.transfer(data[b]);
    }
    digitalWrite(at86rf2xx_dev.cs_pin, HIGH);
}

void at86rf2xx_fb_read(uint8_t *data,
                       const size_t len)
{
    byte readCommand = AT86RF2XX_ACCESS_FB | AT86RF2XX_ACCESS_READ;
    digitalWrite(at86rf2xx_dev.cs_pin, LOW);
    SPI.transfer(readCommand);
    for (int b=0; b<len; b++) {
      data[b] = SPI.transfer(0x00);
    }
    digitalWrite(at86rf2xx_dev.cs_pin, HIGH);
}

uint8_t at86rf2xx_get_status()
{
    /* if sleeping immediately return state */
    if(at86rf2xx_dev.state == AT86RF2XX_STATE_SLEEP)
        return at86rf2xx_dev.state;

    return at86rf2xx_reg_read(AT86RF2XX_REG__TRX_STATUS)
                & AT86RF2XX_TRX_STATUS_MASK__TRX_STATUS;
}

void at86rf2xx_assert_awake()
{
    if(at86rf2xx_get_status() == AT86RF2XX_STATE_SLEEP) {
        /* wake up and wait for transition to TRX_OFF */
        digitalWrite(at86rf2xx_dev.sleep_pin, LOW);
        delayMicroseconds(AT86RF2XX_WAKEUP_DELAY);

        /* update state */
        at86rf2xx_dev.state = at86rf2xx_reg_read(AT86RF2XX_REG__TRX_STATUS)
                         & AT86RF2XX_TRX_STATUS_MASK__TRX_STATUS;
    }
}

void at86rf2xx_hardware_reset()
{
    /* wake up from sleep in case radio is sleeping */
    delayMicroseconds(50); // Arduino seems to hang without some minimum pause here
    at86rf2xx_assert_awake();

    /* trigger hardware reset */

    digitalWrite(at86rf2xx_dev.reset_pin, LOW);
    delayMicroseconds(AT86RF2XX_RESET_PULSE_WIDTH);
    digitalWrite(at86rf2xx_dev.reset_pin, HIGH);
    delayMicroseconds(AT86RF2XX_RESET_DELAY);
}

void at86rf2xx_force_trx_off()
{
    at86rf2xx_reg_write(AT86RF2XX_REG__TRX_STATE,
                        AT86RF2XX_TRX_STATE__FORCE_TRX_OFF);
    while (at86rf2xx_get_status() != AT86RF2XX_STATE_TRX_OFF);
}
