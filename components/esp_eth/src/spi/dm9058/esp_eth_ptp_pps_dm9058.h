/**
 ******************************************************************************
 * @file     esp_eth_ptp_pps_dm9058.h
 * @brief    DM9058 PTP 1PPS signal output driver for ESP-IDF
 *
 * @details  Ported from ptp_gpio_generate.c (MCU vendor driver) to ESP-IDF
 *           architecture. Uses esp_eth_ptp_dm9058_t register-access callbacks
 *           instead of HAL_write_reg / HAL_read_reg.
 *
 *           Implements GP1 (GPIO0) or GP2 (GPIO1) single-pulse output using
 *           DM9058 PTP hardware trigger registers, with automatic re-arm and
 *           timeout protection.
 *
 *           Typical usage:
 *
 *             esp_eth_ptp_pps_dm9058_t pps = {};
 *             esp_eth_ptp_pps_dm9058_config_t cfg =
 *                 ESP_ETH_PTP_PPS_DM9058_1PPS_CONFIG_DEFAULT();
 *             ESP_ERROR_CHECK(esp_eth_ptp_pps_dm9058_init(&pps, &ptp, &cfg));
 *
 *             // In main loop / FreeRTOS task (call every ≤ 100 ms):
 *             esp_eth_ptp_pps_dm9058_update(&pps);
 *
 *             // esp_eth_ptp_dm9058_set_time() automatically triggers
 *             // notify_restart() via the registered on_restart hook.
 *             // Manual call is only needed for edge cases (e.g. external reset).
 *
 *           Reference:
 *             ptp_1588_gpio_both_porting_guide.md (9-step register sequence)
 *             ptp_gpio_generate.c (guard / timeout / restart-protection logic)
 ******************************************************************************
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_eth_ptp_dm9058.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * GPIO-trigger register addresses (0x6A–0x6F are not defined in dm9058.h)
 * ---------------------------------------------------------------------------
 * Addresses 0x60/0x61/0x68 reuse the DM9058_PTP_CR / DM9058_PTP_ENR /
 * DM9058_PTP_DATA macros already defined in dm9058.h.
 * --------------------------------------------------------------------------- */
#define DM9058_PPS_REG_PGCR    0x6A  /**< PTP GPIO Control Register                  */
#define DM9058_PPS_REG_PGTER   0x6B  /**< PTP GPIO Trigger / Edge Register           */
#define DM9058_PPS_REG_PA_LW   0x6C  /**< Asserted pulse width low  byte             */
#define DM9058_PPS_REG_PA_HW   0x6D  /**< Asserted pulse width high byte + unit      */
#define DM9058_PPS_REG_PD_LW   0x6E  /**< Deasserted pulse width low  byte           */
#define DM9058_PPS_REG_PD_HW   0x6F  /**< Deasserted pulse width high byte + unit    */

/* REG[0x60] trigger-done flags (Write-1-to-Clear) */
#define DM9058_PPS_GPIO0_DONE_BIT  (1u << 5)  /**< bit5: GP1 (GPIO0) trigger done */
#define DM9058_PPS_GPIO1_DONE_BIT  (1u << 7)  /**< bit7: GP2 (GPIO1) trigger done */

/* REG[0x62] start-trigger command bits.
 * NOTE: IRQ-enable bit (0x80) is intentionally omitted to avoid conflict with
 *       the ptpd PTP interrupt used by esp_eth_ptp_dm9058.c.               */
#define DM9058_PPS_GP1_TRIGGER     0x10u  /**< Start GPIO0 (GP1) trigger */
#define DM9058_PPS_GP2_TRIGGER     0x20u  /**< Start GPIO1 (GP2) trigger */

/* REG[0x6B] trigger-mode bit-field values (bit[3:2]) */
#define DM9058_PPS_PGTER_EDGE      0x00u  /**< Edge output              */
#define DM9058_PPS_PGTER_TOGGLE    0x04u  /**< Toggle on each trigger   */
#define DM9058_PPS_PGTER_SINGLE    0x08u  /**< Single pulse (default)   */
#define DM9058_PPS_PGTER_PERIODIC  0x0Cu  /**< Periodic pulse           */
#define DM9058_PPS_PGTER_ACT_HIGH  0x02u  /**< Active-High polarity     */

/* Pulse-width unit values for REG[0x6D/0x6F] bit[7:6] */
#define DM9058_PPS_UNIT_120NS      0x00u  /**< 120 ns per count  */
#define DM9058_PPS_UNIT_1US        0x40u  /**< 1 µs  per count   */
#define DM9058_PPS_UNIT_1MS        0x80u  /**< 1 ms  per count   */

/* ---------------------------------------------------------------------------
 * Timeout and restart-guard constants (milliseconds)
 * --------------------------------------------------------------------------- */

/** Re-arm timeout: if no done-flag arrives within this window, force re-arm.
 *  Should be >= sec_offset * 1000 + 500 ms.  Default sec_offset=1 → 1500 ms,
 *  but 3000 ms gives extra margin.                                          */
#define DM9058_PPS_REARM_TIMEOUT_MS   3000u

/** Post-PTP-restart guard: after esp_eth_ptp_dm9058_set_time() the hardware
 *  clears all GPIO trigger settings.  Wait this long before re-arming to let
 *  the PTP clock stabilise and prevent an arm→clear→arm dead-loop.         */
#define DM9058_PPS_RESTART_GUARD_MS   3000u

/* ---------------------------------------------------------------------------
 * esp_eth_ptp_pps_dm9058_config_t — per-channel configuration
 * --------------------------------------------------------------------------- */
typedef struct {
    uint8_t  gpio_pin;           /**< GPIO channel: 0 = GP1 (GPIO0), 1 = GP2 (GPIO1) */
    uint8_t  trig_por;           /**< Polarity: 0 = Active Low, 1 = Active High       */
    uint8_t  sec_offset;         /**< Trigger at PTP_sec + sec_offset (≥ 1)           */

    struct {
        uint8_t asserted_unit;   /**< DM9058_PPS_UNIT_120NS / 1US / 1MS              */
        uint8_t asserted_high;   /**< High-byte of count, bit[5:0] (0–63)            */
        uint8_t asserted_low;    /**< Low-byte  of count (0–255)                     */
        uint8_t deasserted_unit; /**< DM9058_PPS_UNIT_120NS / 1US / 1MS              */
        uint8_t deasserted_high;
        uint8_t deasserted_low;
    } pulse;                     /**< Pulse-width parameters (SINGLE / PERIODIC)      */

    uint32_t rearm_timeout_ms;   /**< 0 → use DM9058_PPS_REARM_TIMEOUT_MS            */
    uint32_t restart_guard_ms;   /**< 0 → use DM9058_PPS_RESTART_GUARD_MS            */
} esp_eth_ptp_pps_dm9058_config_t;

/**
 * @brief Default 1PPS configuration macro.
 *
 *   GP1 (GPIO0), Active High, SINGLE mode, sec_offset = 1
 *   Asserted   :  (0×256 + 100) × 1ms = 100 ms
 *   Deasserted :  (3×256 + 132) × 1ms = 900 ms
 *   Total period ≈ 1000 ms  →  1 Hz PPS signal aligned to PTP second boundary
 */
#define ESP_ETH_PTP_PPS_DM9058_1PPS_CONFIG_DEFAULT() {          \
    .gpio_pin        = 0,                                        \
    .trig_por        = 1,                                        \
    .sec_offset      = 1,                                        \
    .pulse = {                                                   \
        .asserted_unit   = DM9058_PPS_UNIT_1MS,                 \
        .asserted_high   = 0,                                    \
        .asserted_low    = 100,                                  \
        .deasserted_unit = DM9058_PPS_UNIT_1MS,                 \
        .deasserted_high = 3,                                    \
        .deasserted_low  = 132,                                  \
    },                                                           \
    .rearm_timeout_ms  = DM9058_PPS_REARM_TIMEOUT_MS,           \
    .restart_guard_ms  = DM9058_PPS_RESTART_GUARD_MS,           \
}

/* ---------------------------------------------------------------------------
 * esp_eth_ptp_pps_dm9058_t — driver context
 *
 * Declare as a local / global variable; no heap allocation required.
 * Initialise to zero before calling esp_eth_ptp_pps_dm9058_init().
 * --------------------------------------------------------------------------- */
typedef struct {
    esp_eth_ptp_dm9058_t            *ptp;             /**< PTP hardware context   */
    esp_eth_ptp_pps_dm9058_config_t  cfg;             /**< Channel configuration  */
    volatile uint8_t                 ptp_restarted;   /**< Set by notify_restart  */
    uint32_t                         last_arm_tick;   /**< Tick at last arm (ms)  */
    uint32_t                         last_restart_tick; /**< Tick at last restart */
    uint8_t                          done_pending;    /**< Done flag seen, pending W1C */
    uint32_t                         done_tick;       /**< Tick when done seen    */
    uint8_t                          initialized;
} esp_eth_ptp_pps_dm9058_t;

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

/**
 * @brief  Initialise the PPS driver and arm the first trigger.
 *
 * @param  pps     Driver context (caller-allocated, zero-initialised).
 * @param  ptp     An already-initialised and enabled PTP context.
 * @param  config  Channel configuration.  Pass NULL to use the 1PPS defaults.
 *
 * @return ESP_OK on success.
 *         ESP_ERR_INVALID_ARG   if pps or ptp is NULL.
 *         ESP_ERR_INVALID_STATE if ptp is not yet enabled.
 *         ESP_ERR_TIMEOUT       if the SPI bus lock timed out.
 *
 * @note   Call this after PTP is enabled (i.e. after esp_eth_ptp_dm9058_enable()).
 */
esp_err_t esp_eth_ptp_pps_dm9058_init(esp_eth_ptp_pps_dm9058_t *pps,
                                       esp_eth_ptp_dm9058_t *ptp,
                                       const esp_eth_ptp_pps_dm9058_config_t *config);

/**
 * @brief  Notify the PPS driver that the PTP clock was reset.
 *
 * Call this immediately after esp_eth_ptp_dm9058_set_time() (or any path that
 * writes PTP_CR_RESTART to REG[0x60]).  The driver defers re-arming by
 * restart_guard_ms to let the clock stabilise.
 *
 * @note   Safe to call from any task or ISR context.
 */
void esp_eth_ptp_pps_dm9058_notify_restart(esp_eth_ptp_pps_dm9058_t *pps);

/**
 * @brief  Polling update — detect trigger-done and re-arm automatically.
 *
 * Call this periodically from your main loop or a dedicated FreeRTOS task.
 * Recommended call interval: ≤ 100 ms.
 *
 * @param  pps  Driver context previously initialised with
 *              esp_eth_ptp_pps_dm9058_init().
 */
void esp_eth_ptp_pps_dm9058_update(esp_eth_ptp_pps_dm9058_t *pps);

#ifdef __cplusplus
}
#endif
