APPLICATION = driver_mrf24j40
include ../Makefile.tests_common

FEATURES_REQUIRED = periph_spi periph_gpio

DISABLE_MODULE += auto_init

USEMODULE += od
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps

# include the selected driver
USEMODULE += mrf24j40

include $(RIOTBASE)/Makefile.include
