APPLICATION = driver_bme280
include ../Makefile.tests_common

USEMODULE += bme280
USEMODULE += xtimer
USEMODULE += printf_float

# set default device parameters in case they are undefined
TEST_I2C                  ?= I2C_0
TEST_I2C_ADDR             ?= 0x77
TEST_MEASURE_OVERSAMPLING ?= BME280_ULTRALOWPOWER
TEST_ALTITUDE             ?= 158 # altitude in Polytechnique School campus

# export parameters
CFLAGS += -DTEST_I2C=$(TEST_I2C)
CFLAGS += -DTEST_I2C_ADDR=$(TEST_I2C_ADDR)
CFLAGS += -DTEST_MEASURE_OVERSAMPLING=$(TEST_MEASURE_OVERSAMPLING)
CFLAGS += -DTEST_ALTITUDE=$(TEST_ALTITUDE)

include $(RIOTBASE)/Makefile.include
