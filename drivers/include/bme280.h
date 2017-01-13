/*
 * Copyright (C) 2016 Kees Bakker, SODAQ
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_bme280 BME280
 * @ingroup     drivers_sensors
 * @brief       Device driver interface for the BME280 sensor
 * @{
 *
 * @file
 * @brief       Device driver interface for the BME280 sensor.
 *
 * @author      Kees Bakker <kees@sodaq.com>
 */

#ifndef BME280_H_
#define BME280_H_

#include "periph/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calibration struct for the BME280 sensor
 *
 * This must be read from the device at startup.
 */
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calibration_t;

/**
 * @brief Values for t_sb field of the BME280 config register
 */
typedef enum {
    BME280_SB_0_5 = 0,
    BME280_SB_62_5 = 1,
    BME280_SB_125 = 2,
    BME280_SB_250 = 3,
    BME280_SB_500 = 4,
    BME280_SB_1000 = 5,
    BME280_SB_10 = 6,
    BME280_SB_20 = 7
} bme280_t_sb_t;

/**
 * @brief Values for filter field of the BME280 config register
 */
typedef enum {
    BME280_FILTER_OFF = 0,
    BME280_FILTER_2 = 1,
    BME280_FILTER_4 = 2,
    BME280_FILTER_8 = 3,
    BME280_FILTER_16 = 4,
} bme280_filter_t;

/**
 * @brief Values for mode field of the BME280 ctrl_meas register
 */
typedef enum {
    BME280_MODE_SLEEP = 0,
    BME280_MODE_FORCED = 1,
    BME280_MODE_FORCED2 = 2,    /* Same as FORCED */
    BME280_MODE_NORMAL = 3
} bme280_mode_t;

/**
 * @brief Values for oversampling settings
 *
 * These values are used for:
 *  - osrs_h field of the BME280 ctrl_hum register
 *  - osrs_t field of the BME280 ctrl_meas register
 *  - osrs_p field of the BME280 ctrl_meas register
 */
typedef enum {
    BME280_OSRS_SKIPPED = 0,
    BME280_OSRS_X1 = 1,
    BME280_OSRS_X2 = 2,
    BME280_OSRS_X4 = 3,
    BME280_OSRS_X8 = 4,
    BME280_OSRS_X16 = 5,
} bme280_osrs_t;

/**
 * @brief Settings for the BME280 sensor
 *
 * These parameters are needed to configure the device at startup.
 */
typedef struct {
    /* I2C details */
    i2c_t i2c_dev;                      /**< I2C device which is used */
    uint8_t i2c_addr;                   /**< I2C address */

    /* Config Register */
    bme280_t_sb_t t_sb;
    bme280_filter_t filter;
    uint8_t spi3w_en;

    /* ctrl_meas */
    bme280_mode_t runMode;              /**< ctrl_meas mode */
    bme280_osrs_t tempOverSample;       /**< ctrl_meas osrs_t */
    bme280_osrs_t pressOverSample;      /**< ctrl_meas osrs_p */

    /* ctrl_hum */
    bme280_osrs_t humidOverSample;      /**< ctrl_hum osrs_h */
} bme280_settings_t;

/**
 * @brief Device descriptor for the BME280 sensor
 */
typedef struct {
    bme280_settings_t settings;         /**< Device Settings */
    bme280_calibration_t calibration;   /**< Calibration Data */
} bme280_t;

#if 0
/**
 * @brief Device initialization parameters
 */
typedef struct {
    i2c_t i2c_dev;
    uint8_t i2c_addr;
    bme280_settings_t settings;
} bme280_params_t;
#endif

/**
 * @brief auto-initialize all configured BME280 devices
 */
void bme280_auto_init(void);

/**
 * @brief Initialize the given BME280 device
 *
 * @param[out] dev          Initialized device descriptor of BME280 device
 * @param[in]  settings     The settings for the BME280 device (sampling rate, etc)
 *
 * @return                  0 on success
 * @return                  -1 if given I2C is not enabled in board config
 * @return                  -2 did not detect BME280
 * @return                  -3 could not read calibration data
 */
int bme280_init(bme280_t* dev, const bme280_settings_t* settings);

/**
 * @brief Reset the BME280 device
 *
 * @param[in]  dev          Device descriptor of BME280 device to read from
 *
 * @return                  0 on success
 * @return                  -1 if given I2C is not enabled in board config
 */
int bme280_reset(bme280_t* dev);

/**
 * @brief Read temperature value from the given BME280 device, returned in °C
 *
 * @param[in] dev           Device descriptor of BME280 device to read from
 *
 * @returns                 The temperature in centi Celsius. In case of an error
 *                          it returns INT16_MIN.
 */
int16_t bme280_read_temperature(bme280_t* dev);

/**
 * @brief Read humidity value from the given BME280 device, returned in centi %RH
 *
 * @param[in]  dev          Device descriptor of BME280 device to read from
 *
 * @returns                 Humidity in centi %RH (i.e. the percentage times 100)
 */
uint16_t bme280_read_humidity(bme280_t *dev);

/**
 * @brief Read air pressure value from the given BME280 device, returned in PA
 *
 * @param[in]  dev          Device descriptor of BME280 device to read from
 *
 * @returns                 The air pressure in Pa
 */
uint32_t bme280_read_pressure(bme280_t *dev);

/**
 * @brief Read identification code from the given BME280 device
 *
 * @param[in]  dev          Device descriptor of BME280 device to read from
 * @param[out] buffer       Buffer for the result of the identification code
 * @param[in]  buflen       The size of the buffer
 *
 * @return                  >= 0 on success, number of bytes read
 * @return                  -1 if the code could not be acquired
 */
int bme280_read_ident(bme280_t *dev, uint8_t * buffer, size_t buflen);

#ifdef __cplusplus
}
#endif

#endif /* BME280_H_ */
/** @} */

