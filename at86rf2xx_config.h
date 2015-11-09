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

#ifndef AT86RF2XX_CONFIG_H_
#define AT86RF2XX_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * You can override any of the below defaults with your own configuration parameters.
 * Refer to at86rf2xx_defaults.h to review available values and constraints.
 * 
 */
#define AT86RF2XX_CHANNEL               26U//AT86RF2XX_DEFAULT_CHANNEL
#define AT86RF2XX_TXPOWER               AT86RF2XX_DEFAULT_TXPOWER
#define AT86RF2XX_ADDR_LONG             AT86RF2XX_DEFAULT_ADDR_LONG
#define AT86RF2XX_ADDR_SHORT            AT86RF2XX_DEFAULT_ADDR_SHORT
#define AT86RF2XX_PANID                 AT86RF2XX_DEFAULT_PANID

#ifdef __cplusplus
}
#endif

#endif /* AT86RF2XX_CONFIG_H_ */
/** @} */
