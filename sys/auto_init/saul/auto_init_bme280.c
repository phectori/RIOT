/*
 * Copyright (C) 2016 Kees Bakker, SODAQ
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     auto_init_saul
 * @{
 *
 * @file
 * @brief       Auto initialization of BME280 driver.
 *
 * @author      Kees Bakker <kees@sodaq.com>
 *
 * @}
 */

#ifdef MODULE_BME280

#include "log.h"
#include "saul_reg.h"

#include "bme280_params.h"
#include "bme280.h"

/**
 * @brief   Define the number of configured sensors
 */
#define BME280_NUMOF    (sizeof(bme280_params) / sizeof(bme280_params[0]))

/**
 * @brief   Allocation of memory for device descriptors
 */
static bme280_t bme280_devs[BME280_NUMOF];

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[BME280_NUMOF * 3];

void auto_init_bme280(void)
{
    size_t se_ix = 0;
    for (int i = 0; i < BME280_NUMOF; i++) {
        int res = bme280_init(&bme280_devs[i], &bme280_params[i]);
        if (res < 0) {
            LOG_ERROR("Unable to initialize BME280 sensor #%i\n", i);
        }
        else {
            /* temperature */
            saul_entries[se_ix].dev = &bme280_devs[i];
            saul_entries[se_ix].name = "bme280-temp";
            saul_entries[se_ix].driver = &bme280_temperature_saul_driver;
            saul_reg_add(&saul_entries[se_ix]);
            se_ix++;

            /* relative humidity */
            saul_entries[se_ix].dev = &bme280_devs[i];
            saul_entries[se_ix].name = "bme280-humidity";
            saul_entries[se_ix].driver = &bme280_relative_humidity_saul_driver;
            saul_reg_add(&saul_entries[se_ix]);
            se_ix++;

            /* pressure */
            saul_entries[se_ix].dev = &bme280_devs[i];
            saul_entries[se_ix].name = "bme280-press";
            saul_entries[se_ix].driver = &bme280_pressure_saul_driver;
            saul_reg_add(&saul_entries[se_ix]);
            se_ix++;
        }
    }
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_BME280 */
