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
 * @brief       Implementation of public functions for AT86RF2xx drivers
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Mark Solters <msolters@gmail.com>
 *
 * @}
 */

#include <Arduino.h>
#include <SPI.h>

#include "ieee802154.h"
#include "at86rf2xx_internal.h"

/*  Declare radio device as globally scoped struct  */
at86rf2xx_t at86rf2xx_dev;

/*  Interrupt flag  */
volatile int at86rf2xx_events = 0;

static void _irq_handler()
{
    at86rf2xx_events++;
    return;
}

int at86rf2xx_init(int cs_pin, int int_pin,
                   int sleep_pin, int reset_pin)
{

    Serial.println("[at86rf2xx] Booting radio device.");

    /* initialize device descriptor */
    at86rf2xx_dev.cs_pin = cs_pin;
    at86rf2xx_dev.int_pin = int_pin;
    at86rf2xx_dev.sleep_pin = sleep_pin;
    at86rf2xx_dev.reset_pin = reset_pin;
    at86rf2xx_dev.idle_state = AT86RF2XX_STATE_TRX_OFF;
    at86rf2xx_dev.state = AT86RF2XX_STATE_SLEEP;

    /* setup GPIOs */
    pinMode(at86rf2xx_dev.reset_pin, OUTPUT);
    pinMode(at86rf2xx_dev.sleep_pin, OUTPUT);
    pinMode(at86rf2xx_dev.int_pin, INPUT);
    pinMode(at86rf2xx_dev.cs_pin, OUTPUT);

    /* initialise SPI */
    //  Set up SPI
    SPI.begin();
    //  Data is transmitted and received MSB first
    SPI.setBitOrder(MSBFIRST);
    //  SPI interface will run at 1MHz if 8MHz chip or 2Mhz if 16Mhz
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    //  Data is clocked on the rising edge and clock is low when inactive
    SPI.setDataMode(SPI_MODE0);

    /*  wait for SPI to be ready  */
    delay(10);

    /*  initialize GPIOs */
    digitalWrite(at86rf2xx_dev.sleep_pin, LOW);
    digitalWrite(at86rf2xx_dev.reset_pin, HIGH);
    digitalWrite(at86rf2xx_dev.cs_pin, HIGH);
    attachInterrupt(digitalPinToInterrupt(at86rf2xx_dev.int_pin), _irq_handler, RISING);

    /* make sure device is not sleeping, so we can query part number */
    at86rf2xx_assert_awake();

    /* test if the SPI is set up correctly and the device is responding */
    byte part_num = at86rf2xx_reg_read(AT86RF2XX_REG__PART_NUM);
    if (part_num != AT86RF233_PARTNUM) {
        Serial.println("[at86rf2xx] Error: unable to read correct part number.");
        return -1;
    }
    Serial.print("[at86rf2xx] Detected part #: 0x");
    Serial.println(part_num, HEX);
    Serial.print("[at86rf2xx] Version: 0x");
    Serial.println(at86rf2xx_reg_read(AT86RF2XX_REG__VERSION_NUM), HEX);

    /* reset device to default values and put it into RX state */
    at86rf2xx_reset();
  
    return 0;
}

void at86rf2xx_reset()
{
    at86rf2xx_hardware_reset();

    /* Reset state machine to ensure a known state */
    at86rf2xx_reset_state_machine();

    /* reset options and sequence number */
    at86rf2xx_dev.seq_nr = 0;
    at86rf2xx_dev.options = 0;

    /* set short and long address */
    at86rf2xx_set_addr_long(AT86RF2XX_ADDR_LONG);
    at86rf2xx_set_addr_short(AT86RF2XX_ADDR_SHORT);

    /* set default PAN id */
    at86rf2xx_set_pan(AT86RF2XX_PANID);

    /* set default channel */
    at86rf2xx_set_chan(AT86RF2XX_CHANNEL);

    /* set default TX power */
    at86rf2xx_set_txpower(AT86RF2XX_TXPOWER);

    /* set default options */
    at86rf2xx_set_option(AT86RF2XX_OPT_PROMISCUOUS, true);
    at86rf2xx_set_option(AT86RF2XX_OPT_AUTOACK, true);
    at86rf2xx_set_option(AT86RF2XX_OPT_CSMA, true);
    at86rf2xx_set_option(AT86RF2XX_OPT_TELL_RX_START, true);
    at86rf2xx_set_option(AT86RF2XX_OPT_TELL_RX_END, true);

    /* enable safe mode (protect RX FIFO until reading data starts) */
    at86rf2xx_reg_write(AT86RF2XX_REG__TRX_CTRL_2,
                        AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE);

//#ifdef MODULE_AT86RF212B
//    at86rf2xx_set_freq(dev, AT86RF2XX_FREQ_915MHZ);
//#endif

    /* don't populate masked interrupt flags to IRQ_STATUS register */
    /*uint8_t tmp = at86rf2xx_reg_read(AT86RF2XX_REG__TRX_CTRL_1);
    tmp &= ~(AT86RF2XX_TRX_CTRL_1_MASK__IRQ_MASK_MODE);
    at86rf2xx_reg_write(AT86RF2XX_REG__TRX_CTRL_1, tmp);*/

    /* disable clock output to save power */
    byte tmp = at86rf2xx_reg_read(AT86RF2XX_REG__TRX_CTRL_0);
    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL);
    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
    tmp |= (AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF);
    at86rf2xx_reg_write(AT86RF2XX_REG__TRX_CTRL_0, tmp);

    /* enable interrupts */
    at86rf2xx_reg_write(AT86RF2XX_REG__IRQ_MASK, AT86RF2XX_IRQ_STATUS_MASK__TRX_END);

    /* clear interrupt flags */
    at86rf2xx_reg_read(AT86RF2XX_REG__IRQ_STATUS);

    /* go into RX state */
    at86rf2xx_set_state(AT86RF2XX_STATE_RX_AACK_ON);

    Serial.println("[at86rf2xx] Reset complete.");
}

bool at86rf2xx_cca()
{
    uint8_t tmp;
    uint8_t status;

    at86rf2xx_assert_awake();

    /* trigger CCA measurment */
    tmp = at86rf2xx_reg_read(AT86RF2XX_REG__PHY_CC_CCA);
    tmp &= AT86RF2XX_PHY_CC_CCA_MASK__CCA_REQUEST;
    at86rf2xx_reg_write(AT86RF2XX_REG__PHY_CC_CCA, tmp);

    /* wait for result to be ready */
    do {
        status = at86rf2xx_reg_read(AT86RF2XX_REG__TRX_STATUS);
    } while (!(status & AT86RF2XX_TRX_STATUS_MASK__CCA_DONE));

    /* return according to measurement */
    if (status & AT86RF2XX_TRX_STATUS_MASK__CCA_STATUS) {
        return true;
    }
    else {
        return false;
    }
}

size_t at86rf2xx_send(uint8_t *data, size_t len)
{
    /* check data length */
    if (len > AT86RF2XX_MAX_PKT_LENGTH) {
        Serial.println("[at86rf2xx] Error: Data to send exceeds max packet size.");
        return 0;
    }
    at86rf2xx_tx_prepare();
    at86rf2xx_tx_load(data, len, 0);
    at86rf2xx_tx_exec();
    return len;
}

void at86rf2xx_tx_prepare()
{
    uint8_t state;

    /* make sure ongoing transmissions are finished */
    do {
        state = at86rf2xx_get_status();
    }
    while (state == AT86RF2XX_STATE_BUSY_TX_ARET);

    /* if receiving cancel */
    if(state == AT86RF2XX_STATE_BUSY_RX_AACK) {
        at86rf2xx_force_trx_off();
        at86rf2xx_dev.idle_state = AT86RF2XX_STATE_RX_AACK_ON;
    } else if (state != AT86RF2XX_STATE_TX_ARET_ON) {
        at86rf2xx_dev.idle_state = state;
    }
    at86rf2xx_set_state(AT86RF2XX_STATE_TX_ARET_ON);
    at86rf2xx_dev.frame_len = IEEE802154_FCS_LEN;
}

size_t at86rf2xx_tx_load(uint8_t *data,
                         size_t len, size_t offset)
{
    at86rf2xx_dev.frame_len += (uint8_t)len;
    at86rf2xx_sram_write(offset + 1, data, len);
    return offset + len;
}

void at86rf2xx_tx_exec()
{
    /* write frame length field in FIFO */
    at86rf2xx_sram_write(0, &(at86rf2xx_dev.frame_len), 1);
    /* trigger sending of pre-loaded frame */
    at86rf2xx_reg_write(AT86RF2XX_REG__TRX_STATE,
                        AT86RF2XX_TRX_STATE__TX_START);
    /*if (at86rf2xx_dev.event_cb && (at86rf2xx_dev.options & AT86RF2XX_OPT_TELL_TX_START)) {
        at86rf2xx_dev.event_cb(NETDEV_EVENT_TX_STARTED, NULL);
    }*/
}

size_t at86rf2xx_rx_len()
{
    uint8_t phr;
    at86rf2xx_fb_read(&phr, 1);

    /* ignore MSB (refer p.80) and substract length of FCS field */
    return (size_t)((phr & 0x7f) - 2);
}

void at86rf2xx_rx_read(uint8_t *data, size_t len, size_t offset)
{
    /* when reading from SRAM, the different chips from the AT86RF2xx family
     * behave differently: the AT86F233, the AT86RF232 and the ATRF86212B return
     * frame length field (PHR) at position 0 and the first data byte at
     * position 1.
     * The AT86RF231 does not return the PHR field and return
     * the first data byte at position 0.
     */
#ifndef MODULE_AT86RF231
    at86rf2xx_sram_read(offset + 1, data, len);
#else
    at86rf2xx_sram_read(offset, data, len);
#endif
}
