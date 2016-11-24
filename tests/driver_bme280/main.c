/*
 * Copyright (C) 2016 Kees Bakker, SODAQ
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the BME280 pressure and temperature sensor
 *
 * @author      Kees Bakker <kees@sodaq.com>
 *
 * @}
 */

#ifndef TEST_I2C
#error "TEST_I2C not defined"
#endif

#ifndef TEST_MEASURE_OVERSAMPLING
#error "TEST_MEASURE_OVERSAMPLING not defined"
#endif

#ifndef TEST_ALTITUDE
#error "TEST_ALTITUDE not defined"
#endif

#include <stdio.h>
#include <inttypes.h>

#include "bme280.h"
#include "xtimer.h"
#include "board.h"

#define SLEEP_2S   (2 * 1000 * 1000u) /* 2 seconds delay between printf */

int main(void)
{
    bme280_t dev;
#if 1
    /* Wheather monitoring */
    bme280_settings_t settings = {
        .i2c_dev = TEST_I2C,
        .i2c_addr = TEST_I2C_ADDR,
        .t_sb = BME280_SB_0_5,
        .filter = BME280_FILTER_OFF,
        .runMode = BME280_MODE_FORCED,
        .tempOverSample = BME280_OSRS_X1,
        .pressOverSample = BME280_OSRS_X1,
        .humidOverSample = BME280_OSRS_X1,
    };
#endif
    float temperature;
    float pressure;
    float humidity;
    //float altitude;
    //float pressure_0;
    int result;

    puts("BME280 test application\n");

    printf("+------------Initializing------------+\n");
    result = bme280_init(&dev, &settings);
    if (result == -1) {
        puts("[Error] The given i2c is not enabled");
        return 1;
    }

    if (result == -2) {
        printf("[Error] The sensor did not answer correctly at address 0x%02X\n", TEST_I2C_ADDR);
        return 1;
    }

    printf("Initialization successful\n\n");

    printf("+------------Calibration------------+\n");
    printf("dig_T1: %u\n", dev.calibration.dig_T1);
    printf("dig_T2: %i\n", dev.calibration.dig_T2);
    printf("dig_T3: %i\n", dev.calibration.dig_T3);

    printf("dig_P1: %u\n", dev.calibration.dig_P1);
    printf("dig_P2: %i\n", dev.calibration.dig_P2);
    printf("dig_P3: %i\n", dev.calibration.dig_P3);
    printf("dig_P4: %i\n", dev.calibration.dig_P4);
    printf("dig_P5: %i\n", dev.calibration.dig_P5);
    printf("dig_P6: %i\n", dev.calibration.dig_P6);
    printf("dig_P7: %i\n", dev.calibration.dig_P7);
    printf("dig_P8: %i\n", dev.calibration.dig_P8);
    printf("dig_P9: %i\n", dev.calibration.dig_P9);

    printf("dig_H1: %u\n", dev.calibration.dig_H1);
    printf("dig_H2: %i\n", dev.calibration.dig_H2);
    printf("dig_H3: %i\n", dev.calibration.dig_H3);
    printf("dig_H4: %i\n", dev.calibration.dig_H4);
    printf("dig_H5: %i\n", dev.calibration.dig_H5);
    printf("dig_H6: %i\n", dev.calibration.dig_H6);

    printf("\n+--------Starting Measurements--------+\n");
    while (1) {
        /* Get temperature in deci degrees celsius */
        bme280_read_temperature(&dev, &temperature);

        /* Get pressure in Pa */
        bme280_read_pressure(&dev, &pressure);

        /* Get pressure in %rH */
        bme280_read_humidity(&dev, &humidity);

        /* Get pressure at sealevel in Pa */
        //bme280_sealevel_pressure(&dev, (int32_t)TEST_ALTITUDE, &pressure_0);

        /* Get altitude in meters */
        //bme280_altitude(&dev, pressure_0, &altitude);

        printf("Temperature [°C]: %.1f\n"
               "Pressure [hPa]: %.2f\n"
               "Humidity [%%rH]: %.2f\n"
               "\n+-------------------------------------+\n",
               temperature,
	       pressure / 100.0,
               humidity);
        xtimer_usleep(SLEEP_2S);
    }

    return 0;
}
