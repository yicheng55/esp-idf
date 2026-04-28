/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include "esp_eth_time.h"
#include "esp_log.h"

static esp_eth_handle_t s_eth_hndl;
static const char *TAG = "esp_eth_time";

static int esp_eth_clock_esp_err_to_errno(esp_err_t esp_err)
{
    switch (esp_err)
    {
    case ESP_ERR_INVALID_ARG:
        return EINVAL;
    case ESP_ERR_INVALID_STATE:
        return EBUSY;
    case ESP_ERR_TIMEOUT:
        return ETIME;
    }
    // default "no err" when error cannot be isolated
    return 0;
}

int esp_eth_clock_adjtime(clockid_t clk_id, esp_eth_clock_adj_param_t *adj)
{
    switch (clk_id) {
    case CLOCK_PTP_SYSTEM:
        if (adj->mode == ETH_CLK_ADJ_FREQ_SCALE) {
            esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_ADJ_PTP_FREQ, &adj->freq_scale);
            if (ret != ESP_OK) {
                errno = esp_eth_clock_esp_err_to_errno(ret);
                return -1;
            }
        } else {
            errno = EINVAL;
            return -1;
        }
        break;
    default:
        errno = EINVAL;
        return -1;
    }
    return 0;
}

int esp_eth_clock_settime(clockid_t clock_id, const struct timespec *tp)
{
    switch (clock_id) {
    case CLOCK_PTP_SYSTEM: {
        if (s_eth_hndl) {
            eth_mac_time_t ptp_time = {
                .seconds = tp->tv_sec,
                .nanoseconds = tp->tv_nsec
            };
            esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_S_PTP_TIME, &ptp_time);
            if (ret != ESP_OK) {
                errno = esp_eth_clock_esp_err_to_errno(ret);
                return -1;
            }
        } else {
            errno = ENODEV;
            return -1;
        }
        break;
    }
    default:
        errno = EINVAL;
        return -1;
    }
    return 0;
}

int esp_eth_clock_gettime(clockid_t clock_id, struct timespec *tp)
{
    switch (clock_id) {
    case CLOCK_PTP_SYSTEM: {
        if (s_eth_hndl) {
            eth_mac_time_t ptp_time;
            // ESP_LOGI(TAG, "gettime s_eth_hndl: %p", (void *)s_eth_hndl);
            esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_G_PTP_TIME, &ptp_time);
            if (ret != ESP_OK) {
                errno = esp_eth_clock_esp_err_to_errno(ret);
                return -1;
            }
            tp->tv_sec = ptp_time.seconds;
            tp->tv_nsec = ptp_time.nanoseconds;
        } else {
            errno = ENODEV;
            return -1;
        }
        break;
    }
    default:
        errno = EINVAL;
        return -1;
    }
    return 0;
}

esp_err_t esp_eth_clock_get_rx_time(esp_eth_handle_t eth_handle, struct timespec *tp)
{
    if (!eth_handle || !tp) {
        return ESP_ERR_INVALID_ARG;
    }

    eth_mac_time_t ptp_rx_time;
    // ESP_LOGI(TAG, "get_rx_time eth_handle: %p", eth_handle);
    esp_err_t ret = esp_eth_ioctl(eth_handle, ETH_MAC_ESP_CMD_G_PTP_RX_TIME, &ptp_rx_time);
    if (ret != ESP_OK) {
        return ret;
    }
    tp->tv_sec = ptp_rx_time.seconds;
    tp->tv_nsec = ptp_rx_time.nanoseconds;
    return ESP_OK;
}

int esp_eth_clock_set_target_time(clockid_t clock_id, struct timespec *tp)
{
    eth_mac_time_t mac_target_time = {
        .seconds = tp->tv_sec,
        .nanoseconds = tp->tv_nsec
    };
    esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_S_TARGET_TIME, &mac_target_time);
    if (ret != ESP_OK) {
        errno = esp_eth_clock_esp_err_to_errno(ret);
        return -1;
    }
    return 0;
}

int esp_eth_clock_register_target_cb(clockid_t clock_id,
                                     ts_target_exceed_cb_from_isr_t ts_callback)
{
    esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_S_TARGET_CB, ts_callback);
    if (ret != ESP_OK) {
        errno = esp_eth_clock_esp_err_to_errno(ret);
        return -1;
    }
    return 0;
}

esp_err_t esp_eth_clock_init(clockid_t clock_id, esp_eth_clock_cfg_t *cfg)
{
    switch (clock_id) {
    case CLOCK_PTP_SYSTEM: {
        // PTP Clock is part of Ethernet system
        esp_eth_ptp_dm9058_enable_config_t ptp_cfg = {
            .enable    = true,
            .transport = cfg->transport,
        };
        ESP_LOGI(TAG, "clock_init cfg->eth_hndl: %p", (void *)cfg->eth_hndl);
        if (esp_eth_ioctl(cfg->eth_hndl, ETH_MAC_ESP_CMD_PTP_ENABLE, &ptp_cfg) != ESP_OK) {
            return ESP_FAIL;
        }
        s_eth_hndl = cfg->eth_hndl;
        ESP_LOGI(TAG, "clock_init s_eth_hndl set to: %p", (void *)s_eth_hndl);
        break;
    }
    default:
        return ESP_FAIL;
    }
    return ESP_OK;
}
