/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

#include <stdint.h>
#include <time.h>
#include "esp_eth.h"

#ifdef __cplusplus
extern "C" {
#endif

void ptpd_dm9058_set_eth_handle(esp_eth_handle_t eth);

int ptpd_dm9058_clock_gettime(struct timespec *ts);

int ptpd_dm9058_clock_settime(const struct timespec *ts);

/** Map a nanosecond slew request (from ptpd) into DM9058 frequency adjust (ppb). */
int ptpd_dm9058_clock_adjtime_ns(int64_t adjustment_ns, int period_ms);

#ifdef __cplusplus
}
#endif
