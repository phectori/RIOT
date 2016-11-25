/*
 * Copyright (C) 2015 Frits Kuipers
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_maple-mini maple-mini
 * @ingroup     boards
 * @brief       Board specific files for the maple-mini board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the maple-mini board
 *
 * @author      Frits Kuipers <frits.kuipers@gmail.com>
 */

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name xtimer configuration
 * @{
 */
#define XTIMER_WIDTH        (16)
#define XTIMER_BACKOFF      5
/** @} */

/**
 * @brief   LED pin definition and handlers
 */
#define LED0_PIN            GPIO_PIN(PORT_B, 1)

/**
 * @brief   User button
 */
#define BTN_B1_PIN          GPIO_PIN(PORT_B, 8)

/**
 * @brief Use the 1st UART for STDIO on this board
 */
#define UART_STDIO_DEV      UART_DEV(0)

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
