/*
 * Copyright (C) 2016 Frits Kuipers <frits.kuipers@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief   Common header for mrf24j40 tests
 *
 * @author  Martine Lenders <mlenders@inf.fu-berlin.de>
 * @author  Frits Kuipers <frits.kuipers@gmail.com>
 */
#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

#include "mrf24j40.h"
#include "mrf24j40_params.h"
#include "net/netdev2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Application-internal functions and variables for mrf24j40 tests
 * @internal
 * @{
 */
#define MRF24J40_NUM   (sizeof(mrf24j40_params) / sizeof(mrf24j40_params[0]))

extern mrf24j40_t devs[MRF24J40_NUM];

void recv(netdev2_t *dev);
int ifconfig(int argc, char **argv);
int txtsnd(int argc, char **argv);
void print_addr(uint8_t *addr, size_t addr_len);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H_ */
/** @} */
