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

#define BME280_PARAMS_DEFAULT              \
    {                                      \
        .i2c_dev = BME280_PARAM_I2C_DEV,   \
        .i2c_addr = BME280_PARAM_I2C_ADDR, \
        .t_sb = BME280_SB_250,             \
        .filter = BME280_FILTER_2,         \
        .runMode = BME280_MODE_FORCED,     \
        .tempOverSample = BME280_OSRS_X1,  \
        .pressOverSample = BME280_OSRS_X1, \
        .humidOverSample = BME280_OSRS_X1, \
    }
/**@}*/

/**
 * @brief   Configure BME280
 */
static const bme280_settings_t bme280_params[] =
{
#ifdef BME280_PARAMS_BOARD
    BME280_PARAMS_BOARD,
#else
    BME280_PARAMS_DEFAULT,
#endif
};

/**
 * @brief   Get the number of configured BME280 devices
 */
#define BME280_NUMOF       (sizeof(bme280_params) / sizeof(bme280_params[0]))

#ifdef MODULE_SAUL_REG
/**
 * @brief   Allocate and configure entries to the SAUL registry
 */
saul_reg_t bme280_saul_reg[][2] =
{
    {
        {
            .name = "bme280-temp",
            .driver = &bme280_temperature_saul_driver
        },
        {
            .name = "bme280-press",
            .driver = &bme280_pressure_saul_driver
        },
    }
};
#endif

#ifdef __cplusplus
}
#endif

#endif /* BME280_PARAMS_H */
/** @} */
