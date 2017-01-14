/*
 * Copyright (C) 2016 Kees Bakker, SODAQ
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_bme280
 *
 * @{
 * @file
 * @brief       Default configuration for BME280
 *
 * @author      Kees Bakker <kees@sodaq.com>
 */

#ifndef BME280_PARAMS_H
#define BME280_PARAMS_H

#include "board.h"
#include "bme280.h"
#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Set default configuration parameters for the BME280
 * @{
 */
#ifndef BME280_PARAM_I2C_DEV
#define BME280_PARAM_I2C_DEV         (0)
#endif
#ifndef BME280_PARAM_I2C_ADDR
#define BME280_PARAM_I2C_ADDR        (0x77)
#endif

/* Defaults for Weather Monitoring */
#define BME280_PARAMS_DEFAULT              \
    {                                      \
        .i2c_dev = BME280_PARAM_I2C_DEV,   \
        .i2c_addr = BME280_PARAM_I2C_ADDR, \
        .t_sb = BME280_SB_0_5,             \
        .filter = BME280_FILTER_OFF,       \
        .runMode = BME280_MODE_FORCED,     \
        .tempOverSample = BME280_OSRS_X1,  \
        .pressOverSample = BME280_OSRS_X1, \
        .humidOverSample = BME280_OSRS_X1, \
    }
/**@}*/

/**
 * @brief   Configure BME280
 */
static const bme280_params_t bme280_params[] =
{
#ifdef BME280_PARAMS_BOARD
    BME280_PARAMS_BOARD,
#else
    BME280_PARAMS_DEFAULT,
#endif
};

#ifdef __cplusplus
}
#endif

#endif /* BME280_PARAMS_H */
/** @} */
