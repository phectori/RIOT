APPLICATION = driver_bme280
include ../Makefile.tests_common

USEMODULE += bme280
USEMODULE += xtimer

# set default device parameters in case they are undefined
TEST_I2C                  ?= I2C_0
TEST_I2C_ADDR             ?= 0x77

# export parameters
CFLAGS += -DTEST_I2C=$(TEST_I2C)
CFLAGS += -DTEST_I2C_ADDR=$(TEST_I2C_ADDR)

include $(RIOTBASE)/Makefile.include
