/**
 ******************************************************************************
 * @file     esp_eth_ptp_pps_dm9058.c
 * @brief    DM9058 PTP 1PPS signal output driver for ESP-IDF
 *
 * @details  ESP-IDF port of ptp_gpio_generate.c.
 *           Register access uses esp_eth_ptp_dm9058_t callback ops instead of
 *           vendor HAL_write_reg / HAL_read_reg.
 *
 *           Register layout (addresses match DM9058 and DM9051 hardware):
 *             REG[0x60]  DM9058_PTP_CR   — GPIO pin select + done flags (W1C)
 *             REG[0x61]  DM9058_PTP_ENR  — PTP clock control / index reset
 *             REG[0x62]  DM9058_PTP_TXCR — GPIO trigger start command
 *             REG[0x68]  DM9058_PTP_DATA — 8-byte time register (auto-increment)
 *             REG[0x6A]  DM9058_PPS_REG_PGCR  — GPIO control mode
 *             REG[0x6B]  DM9058_PPS_REG_PGTER — trigger type + polarity
 *             REG[0x6C–0x6F]              — pulse-width registers
 *
 *           Reference:
 *             ptp_1588_gpio_both_porting_guide.md   (9-step register sequence)
 *             ptp_gpio_generate.c                   (guard/timeout/restart logic)
 ******************************************************************************
 */

#include <string.h>
#include <inttypes.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "dm9058.h"
#include "esp_eth_ptp_pps_dm9058.h"

static const char *TAG = "dm9058.pps";

/* Uncomment to enable verbose register-level debug output */
/* #define DM9058_PPS_DEBUG */

#ifdef DM9058_PPS_DEBUG
#  define PPS_DBG(fmt, ...)  ESP_LOGD(TAG, fmt, ##__VA_ARGS__)
#else
#  define PPS_DBG(fmt, ...)  do {} while (0)
#endif

/* ---------------------------------------------------------------------------
 * Private helpers
 * --------------------------------------------------------------------------- */

/** Return current millisecond tick (wraps at ~49.7 days, same as HAL_GetTick). */
static inline uint32_t pps_tick_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

/** Acquire the SPI bus lock if provided. Returns true on success. */
static bool pps_lock(esp_eth_ptp_pps_dm9058_t *pps)
{
    if (pps->ptp->ops.lock != NULL) {
        return pps->ptp->ops.lock(pps->ptp->io_ctx);
    }
    return true;
}

/** Release the SPI bus lock if it was acquired. */
static void pps_unlock(esp_eth_ptp_pps_dm9058_t *pps, bool locked)
{
    if (locked && pps->ptp->ops.unlock != NULL) {
        pps->ptp->ops.unlock(pps->ptp->io_ctx);
    }
}

/** Write a single byte to a DM9058 register. */
static inline esp_err_t pps_wr(esp_eth_ptp_pps_dm9058_t *pps, uint8_t reg, uint8_t val)
{
    return pps->ptp->ops.reg_write(pps->ptp->io_ctx, reg, val);
}

/** Read a single byte from a DM9058 register. */
static inline esp_err_t pps_rd(esp_eth_ptp_pps_dm9058_t *pps, uint8_t reg, uint8_t *val)
{
    return pps->ptp->ops.reg_read(pps->ptp->io_ctx, reg, val);
}

/**
 * @brief  Burst-write len bytes to an auto-increment register (REG[0x68]).
 *         Uses reg_burst_write if available, otherwise falls back to
 *         repeated reg_write calls (hardware auto-increments the index).
 */
static esp_err_t pps_burst_wr(esp_eth_ptp_pps_dm9058_t *pps,
                               uint8_t reg, const uint8_t *buf, size_t len)
{
    if (pps->ptp->ops.reg_burst_write != NULL) {
        return pps->ptp->ops.reg_burst_write(pps->ptp->io_ctx, reg, buf, len);
    }
    for (size_t i = 0; i < len; i++) {
        esp_err_t ret = pps->ptp->ops.reg_write(pps->ptp->io_ctx, reg, buf[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

/**
 * @brief  Burst-read len bytes from an auto-increment register (REG[0x68]).
 *         Uses reg_burst_read if available, otherwise falls back to
 *         repeated reg_read calls.
 */
static esp_err_t pps_burst_rd(esp_eth_ptp_pps_dm9058_t *pps,
                               uint8_t reg, uint8_t *buf, size_t len)
{
    if (pps->ptp->ops.reg_burst_read != NULL) {
        return pps->ptp->ops.reg_burst_read(pps->ptp->io_ctx, reg, buf, len);
    }
    for (size_t i = 0; i < len; i++) {
        esp_err_t ret = pps->ptp->ops.reg_read(pps->ptp->io_ctx, reg, &buf[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

/**
 * @brief  Convert the asserted pulse width in cfg to a millisecond guard time.
 *
 *         The done flag in REG[0x60] may appear while the pulse is still being
 *         driven.  We must wait at least this long after detecting the flag
 *         before issuing the W1C clear and re-arming.
 *
 * @return Guard time in ms (always ≥ 1 ms).
 */
static uint32_t pps_asserted_guard_ms(const esp_eth_ptp_pps_dm9058_config_t *cfg)
{
    uint32_t count = ((uint32_t)(cfg->pulse.asserted_high & 0x3Fu) << 8)
                   | (uint32_t)cfg->pulse.asserted_low;
    if (count == 0u) {
        count = 1u;
    }

    uint32_t guard_ms;
    switch (cfg->pulse.asserted_unit & 0xC0u) {
    case DM9058_PPS_UNIT_1MS:
        guard_ms = count;
        break;
    case DM9058_PPS_UNIT_1US:
    case 0xC0u:   /* bit[7:6]=11 is also treated as 1 µs */
        guard_ms = (count + 999u) / 1000u;
        break;
    case DM9058_PPS_UNIT_120NS:
    default:
        guard_ms = (count * 120u + 999999u) / 1000000u;
        break;
    }
    return guard_ms + 1u;   /* +1 ms margin to avoid edge-case races */
}

/**
 * @brief  Arm the GPIO trigger: execute the full 9-step register sequence.
 *
 *         Ported from gpio_pulse_arm() in ptp_gpio_generate.c.
 *         Acquires the SPI bus lock for the entire sequence.
 *
 * @param  pps   Driver context.
 * @param  tick  Current millisecond tick (used to update last_arm_tick).
 * @return ESP_OK on success, or an error code.
 */
static esp_err_t pps_arm(esp_eth_ptp_pps_dm9058_t *pps, uint32_t tick)
{
    const esp_eth_ptp_pps_dm9058_config_t *cfg = &pps->cfg;
    esp_err_t ret        = ESP_OK;
    bool      locked     = false;
    uint8_t   raw[8];
    uint32_t  current_ns  = 0;
    uint32_t  current_sec = 0;
    uint32_t  trigger_sec = 0;

    /* Acquire SPI bus lock for the full arm sequence */
    locked = pps_lock(pps);
    if (!locked && pps->ptp->ops.lock != NULL) {
        ESP_LOGE(TAG, "GP%u arm: SPI lock timeout", cfg->gpio_pin + 1u);
        return ESP_ERR_TIMEOUT;
    }

    /* ------------------------------------------------------------------
     * Step 1: Ensure PTP clock is enabled — REG[0x61] bit0 = 1
     *
     * ptpd's frequency-adjust and set_time paths may alter REG[0x61];
     * write 0x01 unconditionally before each arm for consistency.
     * ------------------------------------------------------------------ */
    ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PTP_ENR, 0x01u),
                      cleanup, TAG, "Step1 PTP enable failed");
    PPS_DBG("Step1: REG[0x61]=0x01 (PTP enable)");

    /* ------------------------------------------------------------------
     * Step 2: Select GPIO pin — REG[0x60] bit1
     *   GP1 → 0x00,  GP2 → 0x02
     * ------------------------------------------------------------------ */
    ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PTP_CR, cfg->gpio_pin ? 0x02u : 0x00u),
                      cleanup, TAG, "Step2 GPIO pin select failed");
    PPS_DBG("Step2: REG[0x60]=0x%02X (pin select)", cfg->gpio_pin ? 0x02u : 0x00u);

    /* ------------------------------------------------------------------
     * Step 3: GPIO control mode — REG[0x6A]
     *   0x04: IRQ enable | 0x02: trigger enable → 0x06 (output mode)
     * ------------------------------------------------------------------ */
    ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PPS_REG_PGCR, 0x06u),
                      cleanup, TAG, "Step3 PGCR write failed");
    PPS_DBG("Step3: REG[0x6A]=0x06");

    /* ------------------------------------------------------------------
     * Step 4: Trigger mode + polarity — REG[0x6B]
     *   Always SINGLE pulse (0x08) — ensures each pulse is aligned to a
     *   specific PTP second boundary via sec_offset.
     * ------------------------------------------------------------------ */
    {
        uint8_t pgter = DM9058_PPS_PGTER_SINGLE
                      | (cfg->trig_por ? DM9058_PPS_PGTER_ACT_HIGH : 0x00u);
        ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PPS_REG_PGTER, pgter),
                          cleanup, TAG, "Step4 PGTER write failed");
        PPS_DBG("Step4: REG[0x6B]=0x%02X (SINGLE|polarity)", pgter);
    }

    /* ------------------------------------------------------------------
     * Step 5: Pulse-width registers — REG[0x6C–0x6F]
     *   REG[0x6D] = unit[7:6] | asserted_high[5:0]
     *   REG[0x6C] = asserted_low
     *   REG[0x6F] = unit[7:6] | deasserted_high[5:0]
     *   REG[0x6E] = deasserted_low
     * ------------------------------------------------------------------ */
    {
        uint8_t pa_hw = (cfg->pulse.asserted_unit   & 0xC0u)
                      | (cfg->pulse.asserted_high   & 0x3Fu);
        uint8_t pd_hw = (cfg->pulse.deasserted_unit & 0xC0u)
                      | (cfg->pulse.deasserted_high & 0x3Fu);

        ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PPS_REG_PA_HW, pa_hw),
                          cleanup, TAG, "Step5 PA_HW failed");
        ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PPS_REG_PA_LW, cfg->pulse.asserted_low),
                          cleanup, TAG, "Step5 PA_LW failed");
        ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PPS_REG_PD_HW, pd_hw),
                          cleanup, TAG, "Step5 PD_HW failed");
        ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PPS_REG_PD_LW, cfg->pulse.deasserted_low),
                          cleanup, TAG, "Step5 PD_LW failed");
        PPS_DBG("Step5: PA 0x%02X/0x%02X  PD 0x%02X/0x%02X",
                pa_hw, cfg->pulse.asserted_low, pd_hw, cfg->pulse.deasserted_low);
    }

    /* ------------------------------------------------------------------
     * Step 6: Read current PTP time
     *   Write REG[0x61] = 0x84 (RESET_IDX | READ_CLOCK), then burst-read
     *   8 bytes from REG[0x68]:  buf[0..3] = ns (LSB first),
     *                             buf[4..7] = seconds (LSB first)
     * ------------------------------------------------------------------ */
    ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PTP_ENR, 0x84u),
                      cleanup, TAG, "Step6 read-clock cmd failed");
    ESP_GOTO_ON_ERROR(pps_burst_rd(pps, DM9058_PTP_DATA, raw, 8),
                      cleanup, TAG, "Step6 read time failed");

    current_ns  = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8)
                | ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
    current_sec = (uint32_t)raw[4] | ((uint32_t)raw[5] << 8)
                | ((uint32_t)raw[6] << 16) | ((uint32_t)raw[7] << 24);
    PPS_DBG("Step6: PTP time %"PRIu32".%09"PRIu32" s", current_sec, current_ns);

    /* ------------------------------------------------------------------
     * Step 7: Calculate trigger target time
     *   trigger_sec = current_sec + sec_offset
     *   If current nanoseconds >= 500 ms, add an extra second buffer to
     *   prevent the trigger target from being too close to the current time.
     * ------------------------------------------------------------------ */
    trigger_sec = current_sec + cfg->sec_offset;
    if (current_ns >= 500000000u) {
        trigger_sec += 1u;
    }

    /* ------------------------------------------------------------------
     * Step 8: Write trigger target time to REG[0x68]
     *   Write REG[0x61] = 0x80 (RESET_IDX), then burst-write 8 bytes:
     *   ns = 0 (align to second boundary), seconds = trigger_sec LSB first
     * ------------------------------------------------------------------ */
    raw[0] = 0u; raw[1] = 0u; raw[2] = 0u; raw[3] = 0u;
    raw[4] = (uint8_t)(trigger_sec);
    raw[5] = (uint8_t)(trigger_sec >> 8);
    raw[6] = (uint8_t)(trigger_sec >> 16);
    raw[7] = (uint8_t)(trigger_sec >> 24);

    ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PTP_ENR, 0x80u),
                      cleanup, TAG, "Step8 reset index failed");
    ESP_GOTO_ON_ERROR(pps_burst_wr(pps, DM9058_PTP_DATA, raw, 8),
                      cleanup, TAG, "Step8 write trigger time failed");
    PPS_DBG("Step8: trigger target %"PRIu32".000000000 s", trigger_sec);

    /* ------------------------------------------------------------------
     * Step 9: Start GPIO trigger — REG[0x62]
     *   GP1 → 0x10,  GP2 → 0x20
     *   IRQ-enable bit (0x80) is NOT set — avoids conflict with ptpd's
     *   PTP interrupt (mirrors test_1588_gpio_trigger03 behaviour).
     * ------------------------------------------------------------------ */
    {
        uint8_t start_cmd = cfg->gpio_pin ? DM9058_PPS_GP2_TRIGGER
                                          : DM9058_PPS_GP1_TRIGGER;
        ESP_GOTO_ON_ERROR(pps_wr(pps, DM9058_PTP_TXCR, start_cmd),
                          cleanup, TAG, "Step9 start trigger failed");
        PPS_DBG("Step9: REG[0x62]=0x%02X (trigger started)", start_cmd);
    }

cleanup:
    pps_unlock(pps, locked);

    if (ret == ESP_OK) {
        pps->last_arm_tick = tick;
        pps->done_pending  = 0;
        ESP_LOGI(TAG, "GP%u armed: trig_at_sec=%"PRIu32" (+%u s)",
                 cfg->gpio_pin + 1u, trigger_sec, cfg->sec_offset);
    } else {
        ESP_LOGE(TAG, "GP%u arm failed: %s",
                 cfg->gpio_pin + 1u, esp_err_to_name(ret));
    }
    return ret;
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

/** Trampoline registered into ptp->on_restart so set_time() auto-notifies. */
static void pps_on_ptp_restart(void *ctx)
{
    esp_eth_ptp_pps_dm9058_notify_restart((esp_eth_ptp_pps_dm9058_t *)ctx);
}

esp_err_t esp_eth_ptp_pps_dm9058_init(esp_eth_ptp_pps_dm9058_t *pps,
                                       esp_eth_ptp_dm9058_t *ptp,
                                       const esp_eth_ptp_pps_dm9058_config_t *config)
{
    ESP_RETURN_ON_FALSE(pps != NULL && ptp != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "null argument");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled,
                        ESP_ERR_INVALID_STATE, TAG, "PTP not enabled");

    memset(pps, 0, sizeof(*pps));
    pps->ptp = ptp;

    /* Register automatic restart hook — set_time() will call notify_restart() */
    ptp->on_restart     = pps_on_ptp_restart;
    ptp->on_restart_ctx = pps;

    if (config != NULL) {
        pps->cfg = *config;
    } else {
        esp_eth_ptp_pps_dm9058_config_t def =
            ESP_ETH_PTP_PPS_DM9058_1PPS_CONFIG_DEFAULT();
        pps->cfg = def;
    }

    /* Apply built-in defaults for any zero timeout/guard values */
    if (pps->cfg.rearm_timeout_ms == 0u) {
        pps->cfg.rearm_timeout_ms = DM9058_PPS_REARM_TIMEOUT_MS;
    }
    if (pps->cfg.restart_guard_ms == 0u) {
        pps->cfg.restart_guard_ms = DM9058_PPS_RESTART_GUARD_MS;
    }

    ESP_LOGI(TAG, "Initialising GP%u PPS output (sec_offset=%u, "
             "assert=%"PRIu32" ms, deassert=%"PRIu32" ms)",
             pps->cfg.gpio_pin + 1u,
             pps->cfg.sec_offset,
             pps_asserted_guard_ms(&pps->cfg) - 1u,
             (((uint32_t)(pps->cfg.pulse.deasserted_high & 0x3Fu) << 8)
              | pps->cfg.pulse.deasserted_low));

    uint32_t tick = pps_tick_ms();
    esp_err_t ret = pps_arm(pps, tick);
    if (ret != ESP_OK) {
        return ret;
    }

    pps->initialized = 1;
    ESP_LOGI(TAG, "GP%u PPS driver ready", pps->cfg.gpio_pin + 1u);
    return ESP_OK;
}

void esp_eth_ptp_pps_dm9058_notify_restart(esp_eth_ptp_pps_dm9058_t *pps)
{
    if (pps == NULL) {
        return;
    }
    /* Volatile write — safe from any context (task or ISR) */
    pps->ptp_restarted = 1;
}

void esp_eth_ptp_pps_dm9058_update(esp_eth_ptp_pps_dm9058_t *pps)
{
    if (pps == NULL || !pps->initialized) {
        return;
    }

    uint32_t tick     = pps_tick_ms();
    uint8_t  rearm    = 0;
    uint8_t  done_bit = pps->cfg.gpio_pin ? DM9058_PPS_GPIO1_DONE_BIT
                                          : DM9058_PPS_GPIO0_DONE_BIT;

    /* ------------------------------------------------------------------
     * PTP restart detection
     *
     * esp_eth_ptp_dm9058_set_time() writes PTP_CR_RESTART to REG[0x60]
     * which clears all GPIO trigger settings (including an in-progress
     * pulse).  If we re-arm immediately, a subsequent set_time will clear
     * the settings again → arm→clear dead-loop.
     *
     * Correct path:
     *   1. Record restart tick and reset the arm timer.
     *   2. Return without reading REG[0x60] (status is unreliable).
     *   3. Let the timeout-protection path re-arm after restart_guard_ms.
     * ------------------------------------------------------------------ */
    if (pps->ptp_restarted) {
        pps->ptp_restarted     = 0;
        pps->last_restart_tick = tick;
        pps->done_pending      = 0;
        pps->last_arm_tick     = tick;   /* restart the rearm-timeout window */
        ESP_LOGI(TAG, "GP%u PTP restart — rearm deferred %"PRIu32" ms",
                 pps->cfg.gpio_pin + 1u, pps->cfg.restart_guard_ms);
        return;
    }

    /* ------------------------------------------------------------------
     * Read REG[0x60] status (under lock)
     * ------------------------------------------------------------------ */
    uint8_t status = 0;
    {
        bool locked = pps_lock(pps);
        if (!locked && pps->ptp->ops.lock != NULL) {
            return;   /* bus busy — skip this cycle */
        }
        esp_err_t err = pps_rd(pps, DM9058_PTP_CR, &status);
        pps_unlock(pps, locked);
        if (err != ESP_OK) {
            return;
        }
    }

    /* ------------------------------------------------------------------
     * Trigger-done flag detected → start guard window
     * ------------------------------------------------------------------ */
    if (!pps->done_pending && (status & done_bit)) {
        PPS_DBG("GP%u done flag detected, entering guard window",
                pps->cfg.gpio_pin + 1u);
        pps->done_pending = 1;
        pps->done_tick    = tick;
    }

    /* ------------------------------------------------------------------
     * Guard window expired → W1C clear done flag, then re-arm
     *
     * REG[0x60] bit5/bit7 may assert before the pulse has fully completed.
     * Delaying the W1C by asserted_guard_ms ensures the output has settled.
     * ------------------------------------------------------------------ */
    if (pps->done_pending) {
        uint32_t guard_ms = pps_asserted_guard_ms(&pps->cfg);
        if ((tick - pps->done_tick) >= guard_ms) {
            bool locked = pps_lock(pps);
            if (!locked && pps->ptp->ops.lock != NULL) {
                return;   /* bus busy — try again next cycle */
            }
            pps_wr(pps, DM9058_PTP_CR, done_bit);   /* Write-1-to-Clear */
            pps_unlock(pps, locked);

            pps->done_pending = 0;
            rearm = 1;
            PPS_DBG("GP%u done flag cleared after %"PRIu32" ms guard",
                    pps->cfg.gpio_pin + 1u, guard_ms);
        }
    }

    /* ------------------------------------------------------------------
     * Timeout protection
     *
     * If the done flag is never seen (lost interrupt, hardware glitch),
     * force a re-arm after rearm_timeout_ms.  The post-restart guard
     * ensures we do not re-arm while PTP is still converging.
     * ------------------------------------------------------------------ */
    if (!pps->done_pending && !rearm
        && (tick - pps->last_arm_tick)     >= pps->cfg.rearm_timeout_ms
        && (tick - pps->last_restart_tick) >= pps->cfg.restart_guard_ms)
    {
        ESP_LOGW(TAG, "GP%u timeout rearm "
                 "(tick=%"PRIu32" last_arm=%"PRIu32" last_restart=%"PRIu32")",
                 pps->cfg.gpio_pin + 1u,
                 tick, pps->last_arm_tick, pps->last_restart_tick);
        rearm = 1;
    }

    /* ------------------------------------------------------------------
     * Re-arm if requested by either path above
     * ------------------------------------------------------------------ */
    if (rearm) {
        pps_arm(pps, tick);
    }
}
