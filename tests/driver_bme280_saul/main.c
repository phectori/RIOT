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
 * @brief       Test application for the BME280 temperature, pressure
 *              and humidity sensor.
 *
 * @author      Kees Bakker <kees@sodaq.com>
 *
 * @}
 */


#include <stdio.h>
#include <inttypes.h>

#include "bme280_params.h"
#include "bme280.h"
#include "xtimer.h"
#include "board.h"

#define MAINLOOP_DELAY  (2 * 1000 * 1000u)      /* 2 seconds delay between printf's */

int main(void)
{
    saul_reg_t * bme_temp;
    saul_reg_t * bme_hum;
    saul_reg_t * bme_pres;
    phydat_t temperature;
    phydat_t pressure;
    phydat_t humidity;
    int dim;

    puts("BME280 SAUL test application");

    /*
     * Get the (first) bme280-temp sensor
     */
    bme_temp = saul_reg_find_name("bme280-temp");
    if (!bme_temp) {
        puts("[Error] BME280 temp sensor not found");
        return 1;
    }

    /*
     * Get the (first) bme280-humidity sensor
     */
    bme_hum = saul_reg_find_name("bme280-humidity");
    if (!bme_hum) {
        puts("[Error] BME280 humidity sensor not found");
        return 1;
    }

    /*
     * Get the (first) bme280-press sensor
     */
    bme_pres = saul_reg_find_name("bme280-press");
    if (!bme_pres) {
        puts("[Error] BME280 pres sensor not found");
        return 1;
    }

    puts("+--------Starting Measurements--------+");
    while (1) {
        puts("\n+-------------------------------------+");

        /* Get temperature in centi degrees Celsius */
        dim = saul_reg_read(bme_temp, &temperature);
        if (dim < 0) {
            puts("[Error] reading BME280 temp sensor");
        }
        phydat_dump(&temperature, dim);
        

        /* Get pressure in Pa */
        dim = saul_reg_read(bme_pres, &pressure);
        if (dim < 0) {
            puts("[Error] reading BME280 pressure sensor");
        }
        phydat_dump(&pressure, dim);

        /* Get pressure in %rH */
        dim = saul_reg_read(bme_hum, &humidity);
        if (dim < 0) {
            puts("[Error] reading BME280 humidity sensor");
        }
        phydat_dump(&humidity, dim);

        xtimer_usleep(MAINLOOP_DELAY);
    }

    return 0;
}
