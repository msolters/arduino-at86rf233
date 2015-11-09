/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    drivers_at86rf2xx AT86RF2xx based drivers
 * @ingroup     drivers_netdev
 *
 * This module contains drivers for radio devices in Atmel's AT86RF2xx series.
 * The driver is aimed to work with all devices of this series.
 *
 * @{
 *
 * @file
 * @brief       Interface definition for AT86RF2xx based drivers
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */

#ifndef AT86RF2XX_H_
#define AT86RF2XX_H_

#include <Arduino.h>
#include <stdint.h>
#include "ieee802154.h"
#include "at86rf2xx_registers.h"
#include "at86rf2xx_defaults.h"
#include "at86rf2xx_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Device descriptor for AT86RF2XX radio devices
 */
typedef struct {
    int cs_pin;                      /**< chip select pin */
    int sleep_pin;                   /**< sleep pin */
    int reset_pin;                   /**< reset pin */
    int int_pin;                     /**< external interrupt pin */
//    gnrc_nettype_t proto;               /**< protocol the radio expects */
    uint8_t state;                      /**< current state of the radio */
    uint8_t seq_nr;                     /**< sequence number to use next */
    uint8_t frame_len;                  /**< length of the current TX frame */
    uint16_t pan;                       /**< currently used PAN ID */
    uint8_t chan;                       /**< currently used channel */
//#ifdef MODULE_AT86RF212B
//    at86rf2xx_freq_t freq;              /**< currently used frequency */
//#endif
    uint8_t addr_short[2];              /**< the radio's short address */
    uint8_t addr_long[8];               /**< the radio's long address */
    uint16_t options;                   /**< state of used options */
    uint8_t idle_state;                 /**< state to return to after sending */
} at86rf2xx_t;

extern at86rf2xx_t at86rf2xx_dev;
extern volatile int at86rf2xx_events;

/**
 * @brief   Initialize a given AT86RF2xx device
 *
 * @param[in] cs_pin        GPIO pin connected to chip select
 * @param[in] int_pin       GPIO pin connected to the interrupt pin
 * @param[in] sleep_pin     GPIO pin connected to the sleep pin
 * @param[in] reset_pin     GPIO pin connected to the reset pin
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int at86rf2xx_init(int cs_pin, int int_pin,
                   int sleep_pin, int reset_pin);

/**
 * @brief   Trigger a hardware reset and configure radio with default values
 */
void at86rf2xx_reset();

/**
 * @brief   Trigger a clear channel assessment
 *
 * @return                  true if channel is clear
 * @return                  false if channel is busy
 */
bool at86rf2xx_cca();

/**
 * @brief   Get the short address of the given device
 *
 * @return                  the currently set (2-byte) short address
 */
uint16_t at86rf2xx_get_addr_short();

/**
 * @brief   Set the short address of the given device
 *
 * @param[in] addr          (2-byte) short address to set
 */
void at86rf2xx_set_addr_short(uint16_t addr);

/**
 * @brief   Get the configured long address of the given device
 *
 * @return                  the currently set (8-byte) long address
 */
uint64_t at86rf2xx_get_addr_long();

/**
 * @brief   Set the long address of the given device
 *
 * @param[in] addr          (8-byte) long address to set
 */
void at86rf2xx_set_addr_long(uint64_t addr);

/**
 * @brief   Get the configured channel of the given device
 *
 * @return                  the currently set channel
 */
uint8_t at86rf2xx_get_chan();

/**
 * @brief   Set the channel of the given device
 *
 * @param[in] chan          channel to set
 */
void at86rf2xx_set_chan(uint8_t chan);

//#ifdef MODULE_AT86RF212B
/**
 * @brief   Get the configured frequency of the given device
 *
 * @return                  the currently set frequency
 */
//at86rf2xx_freq_t at86rf2xx_get_freq();

/**
 * @brief   Set the frequency of the given device
 * @param[in] chan          frequency to set
 */
//void at86rf2xx_set_freq(at86rf2xx_freq_t freq);
//#endif

/**
 * @brief   Get the configured PAN ID of the given device
 *
 * @return                  the currently set PAN ID
 */
uint16_t at86rf2xx_get_pan();

/**
 * @brief   Set the PAN ID of the given device
 *
 * @param[in] pan           PAN ID to set
 */
void at86rf2xx_set_pan(uint16_t pan);

/**
 * @brief   Get the configured transmission power of the given device [in dBm]
 *
 * @return                  configured transmission power in dBm
 */
int16_t at86rf2xx_get_txpower();

/**
 * @brief   Set the transmission power of the given device [in dBm]
 *
 * If the device does not support the exact dBm value given, it will set a value
 * as close as possible to the given value. If the given value is larger or
 * lower then the maximal or minimal possible value, the min or max value is
 * set, respectively.
 *
 * @param[in] txpower       transmission power in dBm
 */
void at86rf2xx_set_txpower(int16_t txpower);

/**
 * @brief   Get the maximum number of retransmissions
 *
 * @return                  configured number of retransmissions
 */
uint8_t at86rf2xx_get_max_retries();

/**
 * @brief   Set the maximum number of retransmissions
 *
 * This setting specifies the number of attempts to retransmit a frame, when it
 * was not acknowledged by the recipient, before the transaction gets cancelled.
 * The maximum value is 7.
 *
 * @param[in] max           the maximum number of retransmissions
 */
void at86rf2xx_set_max_retries(uint8_t max);

/**
 * @brief   Get the maximum number of channel access attempts per frame (CSMA)
 *
 * @return                  configured number of retries
 */
uint8_t at86rf2xx_get_csma_max_retries();

/**
 * @brief   Set the maximum number of channel access attempts per frame (CSMA)
 *
 * This setting specifies the number of attempts to access the channel to
 * transmit a frame. If the channel is busy @p retries times, then frame
 * transmission fails.
 * Valid values: 0 to 5, -1 means CSMA disabled
 *
 * @param[in] max           the maximum number of retries
 */
void at86rf2xx_set_csma_max_retries(int8_t retries);

/**
 * @brief   Set the min and max backoff exponent for CSMA/CA
 *
 * - Maximum BE: 0 - 8
 * - Minimum BE: 0 - [max]
 *
 * @param[in] min           the minimum BE
 * @param[in] max           the maximum BE
 */
void at86rf2xx_set_csma_backoff_exp(uint8_t min, uint8_t max);

/**
 * @brief   Set seed for CSMA random backoff
 *
 * @param[in] entropy       11 bit of entropy as seed for random backoff
 */
void at86rf2xx_set_csma_seed(uint8_t entropy[2]);

/**
 * @brief   Enable or disable driver specific options
 *
 * @param[in] option        option to enable/disable
 * @param[in] state         true for enable, false for disable
 */
void at86rf2xx_set_option(uint16_t option, bool state);

/**
 * @brief   Set the state of the given device (trigger a state change)
 *
 * @param[in] state         the targeted new state
 */
void at86rf2xx_set_state(uint8_t state);

/**
 * @brief   Reset the internal state machine to TRX_OFF mode.
 *
 * This will force a transition to TRX_OFF regardless of whether the transceiver
 * is currently busy sending or receiving. This function is used to get back to
 * a known state during driver initialization.
 *
 */
void at86rf2xx_reset_state_machine();

/**
 * @brief   Convenience function for simply sending data
 *
 * @note This function ignores the PRELOADING option
 *
 * @param[in] data          data to send (must include IEEE802.15.4 header)
 * @param[in] len           length of @p data
 *
 * @return                  number of bytes that were actually send
 * @return                  0 on error
 */
size_t at86rf2xx_send(uint8_t *data, size_t len);

/**
 * @brief   Prepare for sending of data
 *
 * This function puts the given device into the TX state, so no receiving of
 * data is possible after it was called.
 *
 */
void at86rf2xx_tx_prepare();

/**
 * @brief   Load chunks of data into the transmit buffer of the given device
 *
 * @param[in] data          buffer containing the data to load
 * @param[in] len           number of bytes in @p buffer
 * @param[in] offset        offset used when writing data to internal buffer
 *
 * @return                  offset + number of bytes written
 */
size_t at86rf2xx_tx_load(uint8_t *data, size_t len,
                         size_t offset);

/**
 * @brief   Trigger sending of data previously loaded into transmit buffer
 *
 */
void at86rf2xx_tx_exec();

/**
 * @brief   Read the length of a received packet
 *
 * @return                  overall length of a received packet in byte
 */
size_t at86rf2xx_rx_len();

/**
 * @brief   Read a chunk of data from the receive buffer of the given device
 *
 * @param[out] data         buffer to write data to
 * @param[in]  len          number of bytes to read from device
 * @param[in]  offset       offset in the receive buffer
 */
void at86rf2xx_rx_read(uint8_t *data, size_t len,
                       size_t offset);


#ifdef __cplusplus
}
#endif

#endif /* AT86RF2XX_H_ */
/** @} */
