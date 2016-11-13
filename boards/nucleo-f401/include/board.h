/*
 * Copyright (C) 2015 Lari Lehtomäki
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_nucleo-f401 Nucleo-F401
 * @ingroup     boards
 * @brief       Board specific files for the nucleo-f401 board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the nucleo-f401 board
 *
 * @author      Lari Lehtomäki <lari@lehtomaki.fi>
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "board_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name xtimer configuration
 * @{
 */
#define XTIMER_DEV          TIMER_0
#define XTIMER_CHAN         (0)
#define XTIMER_OVERHEAD     (6)
#define XTIMER_BACKOFF      (5)
/** @} */

#define MRF24J40_PARAM_SPI         (SPI_0)
#define MRF24J40_PARAM_SPI_SPEED   (SPI_SPEED_5MHZ)
#define MRF24J40_PARAM_CS          (GPIO_PIN(0, 8))
#define MRF24J40_PARAM_INT         (GPIO_PIN(1, 10))
#define MRF24J40_PARAM_SLEEP       (GPIO_PIN(2, 7))
#define MRF24J40_PARAM_RESET       (GPIO_PIN(1, 4))


#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
