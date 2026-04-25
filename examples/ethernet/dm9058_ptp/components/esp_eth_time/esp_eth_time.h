/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sys/time.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_eth_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLOCK_PTP_SYSTEM         ((clockid_t) 19)

typedef struct {
    esp_eth_handle_t eth_hndl;
} esp_eth_clock_cfg_t;

typedef enum {
    ETH_CLK_ADJ_FREQ_SCALE,
} esp_eth_clock_adj_mode_t;

typedef struct {
    esp_eth_clock_adj_mode_t mode;
    double freq_scale;
} esp_eth_clock_adj_param_t;

typedef bool (*ts_target_exceed_cb_from_isr_t)(esp_eth_mediator_t *eth, void *user_args);

int esp_eth_clock_adjtime(clockid_t clk_id, esp_eth_clock_adj_param_t *adj);
int esp_eth_clock_settime(clockid_t clock_id, const struct timespec *tp);
int esp_eth_clock_gettime(clockid_t clock_id, struct timespec *tp);
int esp_eth_clock_set_target_time(clockid_t clock_id, struct timespec *tp);
int esp_eth_clock_register_target_cb(clockid_t clock_id,
                                     ts_target_exceed_cb_from_isr_t ts_callback);
esp_err_t esp_eth_clock_init(clockid_t clock_id, esp_eth_clock_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
