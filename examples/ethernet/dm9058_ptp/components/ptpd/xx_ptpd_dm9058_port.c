/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "ptpd_dm9058_port.h"
#include "esp_eth_mac_spi.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "ptpd_dm9058";

static esp_eth_handle_t s_eth;

void ptpd_dm9058_set_eth_handle(esp_eth_handle_t eth)
{
    s_eth = eth;
}

int ptpd_dm9058_clock_gettime(struct timespec *ts)
{
    if (s_eth == NULL || ts == NULL) {
        return -1;
    }
    eth_dm9058_ptp_time_t t;
    if (esp_eth_ioctl(s_eth, ETH_MAC_DM9058_CMD_G_PTP_TIME, &t) != ESP_OK) {
        return -1;
    }
    ts->tv_sec = (time_t)t.seconds;
    ts->tv_nsec = (long)t.nanoseconds;
    return 0;
}

int ptpd_dm9058_clock_settime(const struct timespec *ts)
{
    if (s_eth == NULL || ts == NULL) {
        return -1;
    }
    eth_dm9058_ptp_time_t t = {
        .seconds = (uint32_t)ts->tv_sec,
        .nanoseconds = (uint32_t)ts->tv_nsec,
    };
    if (esp_eth_ioctl(s_eth, ETH_MAC_DM9058_CMD_S_PTP_TIME, &t) != ESP_OK) {
        return -1;
    }
    return 0;
}

int ptpd_dm9058_clock_adjtime_ns(int64_t adjustment_ns, int period_ms)
{
    if (s_eth == NULL || period_ms <= 0) {
        return -1;
    }
    /* First-order map: nanoseconds to correct over `period_ms` => ppb */
    double ppb = (double)adjustment_ns * 1000.0 / (double)period_ms;
    if (ppb > 500000.0) {
        ppb = 500000.0;
    } else if (ppb < -500000.0) {
        ppb = -500000.0;
    }
    int32_t adj = (int32_t)lrint(ppb);
    esp_err_t err = esp_eth_ioctl(s_eth, ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ, &adj);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADJ_PTP_FREQ failed: %s", esp_err_to_name(err));
        return -1;
    }
    return 0;
}
