/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include "esp_eth_time.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static esp_eth_handle_t s_eth_hndl;
static const char *TAG = "esp_eth_time";

static ts_target_exceed_cb_from_isr_t s_target_cb;
static esp_timer_handle_t s_target_timer;
static struct timespec s_armed_target;
static portMUX_TYPE s_target_spin = portMUX_INITIALIZER_UNLOCKED;

static int esp_eth_clock_esp_err_to_errno(esp_err_t esp_err)
{
    switch (esp_err) {
    case ESP_ERR_INVALID_ARG:
        return EINVAL;
    case ESP_ERR_INVALID_STATE:
        return EBUSY;
    case ESP_ERR_TIMEOUT:
        return ETIME;
    default:
        return 0;
    }
}

/** @return microseconds (later - earlier), may be negative */
static int64_t timespec_diff_us(const struct timespec *later, const struct timespec *earlier)
{
    int64_t s = (int64_t)later->tv_sec - (int64_t)earlier->tv_sec;
    int64_t ns = (int64_t)later->tv_nsec - (int64_t)earlier->tv_nsec;
    while (ns < 0) {
        s--;
        ns += 1000000000LL;
    }
    while (ns >= 1000000000LL) {
        s++;
        ns -= 1000000000LL;
    }
    return s * 1000000LL + ns / 1000LL;
}

static int timespec_cmp(const struct timespec *a, const struct timespec *b)
{
    if (a->tv_sec < b->tv_sec) {
        return -1;
    }
    if (a->tv_sec > b->tv_sec) {
        return 1;
    }
    if (a->tv_nsec < b->tv_nsec) {
        return -1;
    }
    if (a->tv_nsec > b->tv_nsec) {
        return 1;
    }
    return 0;
}

static void target_timer_trampoline(void *arg)
{
    (void)arg;
    if (s_eth_hndl == NULL) {
        return;
    }

    struct timespec now;
    if (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &now) != 0) {
        return;
    }

    struct timespec target;
    portENTER_CRITICAL(&s_target_spin);
    target = s_armed_target;
    portEXIT_CRITICAL(&s_target_spin);

    if (timespec_cmp(&now, &target) < 0) {
        int64_t rem_us = timespec_diff_us(&target, &now);
        if (rem_us < 1) {
            rem_us = 1;
        }
        /* Avoid huge single-shot internal limits: chunk long waits */
        const uint64_t max_chunk_us = 10ULL * 1000000ULL;
        uint64_t wait_us = (uint64_t)rem_us;
        if (wait_us > max_chunk_us) {
            wait_us = max_chunk_us;
        }
        esp_timer_start_once(s_target_timer, wait_us);
        return;
    }

    if (s_target_cb) {
        (void)s_target_cb(NULL, NULL);
    }
}

static esp_err_t ensure_target_timer_created(void)
{
    if (s_target_timer) {
        return ESP_OK;
    }
    const esp_timer_create_args_t args = {
        .callback = &target_timer_trampoline,
        .arg = NULL,
        .name = "ptp_tgt",
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = true,
    };
    return esp_timer_create(&args, &s_target_timer);
}

int esp_eth_clock_adjtime(clockid_t clk_id, esp_eth_clock_adj_param_t *adj)
{
    switch (clk_id) {
    case CLOCK_PTP_SYSTEM:
        if (adj->mode == ETH_CLK_ADJ_FREQ_SCALE) {
            int32_t adj_ppb = (int32_t)adj->freq_scale;
            esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ, &adj_ppb);
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
            eth_dm9058_ptp_time_t ptp_time = {
                .seconds = (uint32_t)tp->tv_sec,
                .nanoseconds = (uint32_t)tp->tv_nsec
            };
            esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_DM9058_CMD_S_PTP_TIME, &ptp_time);
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
            eth_dm9058_ptp_time_t ptp_time;
            esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_DM9058_CMD_G_PTP_TIME, &ptp_time);
            if (ret != ESP_OK) {
                errno = esp_eth_clock_esp_err_to_errno(ret);
                return -1;
            }
            tp->tv_sec = (time_t)ptp_time.seconds;
            tp->tv_nsec = (long)ptp_time.nanoseconds;
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

    (void)eth_handle;
    (void)tp;
    return ESP_ERR_NOT_SUPPORTED;
}

int esp_eth_clock_set_target_time(clockid_t clock_id, struct timespec *tp)
{
    if (clock_id != CLOCK_PTP_SYSTEM || tp == NULL) {
        errno = EINVAL;
        return -1;
    }
    if (s_eth_hndl == NULL) {
        errno = ENODEV;
        return -1;
    }
    if (ensure_target_timer_created() != ESP_OK) {
        errno = ENOMEM;
        return -1;
    }

    struct timespec now;
    if (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &now) != 0) {
        errno = EIO;
        return -1;
    }

    portENTER_CRITICAL(&s_target_spin);
    s_armed_target = *tp;
    portEXIT_CRITICAL(&s_target_spin);

    esp_timer_stop(s_target_timer);

    int64_t wait_us = timespec_diff_us(tp, &now);
    if (wait_us < 1) {
        wait_us = 1;
    }
    /* esp_timer_start_once accepts uint64_t us */
    uint64_t wait_u64 = (uint64_t)wait_us;
    esp_err_t err = esp_timer_start_once(s_target_timer, wait_u64);
    if (err != ESP_OK) {
        errno = esp_eth_clock_esp_err_to_errno(err);
        return -1;
    }
    return 0;
}

int esp_eth_clock_register_target_cb(clockid_t clock_id,
                                     ts_target_exceed_cb_from_isr_t ts_callback)
{
    if (clock_id != CLOCK_PTP_SYSTEM) {
        errno = EINVAL;
        return -1;
    }
    if (s_eth_hndl == NULL) {
        errno = ENODEV;
        return -1;
    }
    if (ts_callback == NULL) {
        errno = EINVAL;
        return -1;
    }
    if (ensure_target_timer_created() != ESP_OK) {
        errno = ENOMEM;
        return -1;
    }
    s_target_cb = ts_callback;
    ESP_LOGI(TAG, "PTP target callback registered (software timer / DM9058)");
    return 0;
}

esp_err_t esp_eth_clock_init(clockid_t clock_id, esp_eth_clock_cfg_t *cfg)
{
    switch (clock_id) {
    case CLOCK_PTP_SYSTEM: {
        ESP_LOGI(TAG, "clock_init cfg->eth_hndl: %p", (void *)cfg->eth_hndl);
        if (esp_eth_ioctl(cfg->eth_hndl, ETH_MAC_DM9058_CMD_S_PTP_TRANSPORT, &cfg->transport) != ESP_OK) {
            return ESP_FAIL;
        }
        bool enable = true;
        if (esp_eth_ioctl(cfg->eth_hndl, ETH_MAC_DM9058_CMD_PTP_ENABLE, &enable) != ESP_OK) {
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
