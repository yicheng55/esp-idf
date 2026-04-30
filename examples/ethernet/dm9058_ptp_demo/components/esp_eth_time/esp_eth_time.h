/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_eth_driver.h"
#include "esp_eth_mac.h"
#include "esp_eth_mac_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Type of callback function invoked under Time Stamp target time exceeded interrupt
 *
 * @param eth: mediator of Ethernet driver
 * @param user_args user specific arguments
 *
 * @return
 *          - TRUE when high priority task has been woken by this function
 *          - FALSE no high priority task was woken by this function
 */
typedef bool (*ts_target_exceed_cb_from_isr_t)(esp_eth_mediator_t *eth, void *user_args);

#define CLOCK_PTP_SYSTEM         ((clockid_t) 19)

/**
 * @brief Configuration of clock during initialization
 *
 */
typedef struct {
    esp_eth_handle_t eth_hndl;
    eth_dm9058_ptp_transport_t transport; /*!< PTP transport type */
} esp_eth_clock_cfg_t;

/**
 * @brief The mode of clock adjustment.
 *
 */
typedef enum {
    ETH_CLK_ADJ_FREQ_SCALE,
} esp_eth_clock_adj_mode_t;

/**
 * @brief Structure containing parameters for adjusting the Ethernet clock.
 *
 */
typedef struct {
    /**
     * @brief The mode of clock adjustment.
     *
     */
    esp_eth_clock_adj_mode_t mode;

    /**
     * @brief The frequency scale factor when in ETH_CLK_ADJ_FREQ_SCALE mode.
     *
     * This value represents the ratio of the desired frequency to the actual
     * frequency. A value greater than 1 increases the frequency, while a value
     * less than 1 decreases the frequency.
     */
    double freq_scale;
} esp_eth_clock_adj_param_t;

/**
 * @brief Adjust the system clock frequency
 *
 * @param clk_id Identifier of the clock to adjust
 * @param buf Pointer to the adjustment parameters
 *
 * @return
 *     - 0: Success
 *     - -1: Failure
 */
int esp_eth_clock_adjtime(clockid_t clk_id, esp_eth_clock_adj_param_t *adj);

/**
 * @brief Set the system clock time
 *
 * @param clk_id Identifier of the clock to set
 * @param tp Pointer to the new time
 *
 * @return
 *     - 0: Success
 *     - -1: Failure
 */
int esp_eth_clock_settime(clockid_t clock_id, const struct timespec *tp);

/**
 * @brief Get the current system clock time
 *
 * @param clk_id Identifier of the clock to query
 * @param tp Pointer to the buffer to store the current time
 *
 * @return
 *     - 0: Success
 *     - -1: Failure
 */
int esp_eth_clock_gettime(clockid_t clock_id, struct timespec *tp);

/**
 * @brief Get the PTP receive time
 *
 * @param eth_handle Ethernet handle
 * @param tp Pointer to the buffer to store the receive time
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - Other ESP_ERR_* codes: Hardware error
 */
esp_err_t esp_eth_clock_get_rx_time(esp_eth_handle_t eth_handle, struct timespec *tp);

/**
 * @brief Set the target time for the system clock.
 *
 * @param clk_id Identifier of the clock to set the target time for
 * @param tp Pointer to the target time
 *
 * @return
 *     - 0: Success
 *     - -1: Failure
 */
int esp_eth_clock_set_target_time(clockid_t clock_id, struct timespec *tp);

/**
 * @brief Register callback invoked when PTP clock reaches the target set by esp_eth_clock_set_target_time()
 *
 * On internal EMAC this runs from the MAC timestamp ISR. On DM9058 (SPI) this stack uses an
 * esp_timer (ESP_TIMER_TASK): the callback is NOT true ISR context; do not use IRAM_ATTR-only APIs
 * that require running from interrupt.
 *
 * @param clock_id Identifier of the clock
 * @param ts_callback callback function to be registered
 * @return
 *     - 0: Success
 *     - -1: Failure
 */
int esp_eth_clock_register_target_cb(clockid_t clock_id,
                                     ts_target_exceed_cb_from_isr_t ts_callback);

/**
 * @brief Initialize the Ethernet clock subsystem
 *
 * @param clk_id Identifier of the clock to initialize
 * @param cfg Pointer to the configuration structure
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_FAIL: Failure
 */
esp_err_t esp_eth_clock_init(clockid_t clock_id, esp_eth_clock_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
