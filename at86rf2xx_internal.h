/*
 * Copyright (C) 2013 Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_at86rf2xx
 * @{
 *
 * @file
 * @brief       Internal interfaces for AT86RF2xx drivers
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Mark Solters <msolters@gmail.com>
 */

#ifndef AT86RF2XX_INTERNAL_H_
#define AT86RF2XX_INTERNAL_H_

#include <stdint.h>

#include "at86rf2xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Transition time from SLEEP to TRX_OFF in us, refer figure 7-4, p.42.
 *          For different environments refer figure 13-13, p.201
 */
#define AT86RF2XX_WAKEUP_DELAY          (300U)

/**
 * @brief   Minimum reset pulse width, refer p.190
 */
#define AT86RF2XX_RESET_PULSE_WIDTH     (1U)

/**
 * @brief   Transition time to TRX_OFF after reset pulse in us, refer
 *          figure 7-8, p. 44.
 */
#define AT86RF2XX_RESET_DELAY           (26U)

#ifdef __cplusplus
}
#endif

#endif /* AT86RF2XX_INTERNAL_H_ */
/** @} */
