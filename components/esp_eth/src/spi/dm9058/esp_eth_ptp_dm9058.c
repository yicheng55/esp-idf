/*
 * SPDX-FileCopyrightText: 2019-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "sdkconfig.h"

#if CONFIG_ETH_SPI_ETHERNET_DM9058

#include "esp_check.h"
#include "esp_eth_ptp_dm9058.h"
#include "dm9058.h"

static const char *PTP_TAG = "dm9058.ptp";

/* DM9058 TCR PTP bits (overlap datasheet "reserved" / jabber names in dm9058.h) */
#define DM9058_TCR_TSEN_CAP     (1u << 7)
#define DM9058_TCR_TS1STEP_EMIT (1u << 6)

#define DM9058_PTP_MAX_ADJUSTMENT (0xEFFFFFFFU)

/* 40 ns tick scaling (same as legacy DM9058 PTP reference driver) */
#define V51_ADJ_FREQ_BASE_ADDEND (171.7987)

/* RX status bits in 4-byte RX header (PTP timestamp presence); not identical to RSR register layout */
#define DM9058_PTP_RXHDR_RXTS_EN     (1u << 5)
#define DM9058_PTP_RXHDR_RXTS_PARITY (1u << 3)
#define DM9058_PTP_RXHDR_RXTS_LEN    (1u << 2)
#define DM9058_PTP_RXHDR_PTP_BITS    (DM9058_PTP_RXHDR_RXTS_EN | DM9058_PTP_RXHDR_RXTS_PARITY | DM9058_PTP_RXHDR_RXTS_LEN)
#define DM9058_PTP_RXHDR_ERR_BITS    (RSR_RF | RSR_LCS | RSR_RWTO | RSR_PLE | RSR_AE | RSR_CE | RSR_FOE)

#define ETH_HLEN         14
#define ETH_TYPE_PTP     0x88F7
#define PTP_FLAG_TWO_STEP (1u << 9)

static esp_err_t dm9058_ptp_try_lock(esp_eth_ptp_dm9058_t *ptp, bool *locked)
{
    *locked = false;
    if (ptp->ops.lock != NULL) {
        if (!ptp->ops.lock(ptp->io_ctx)) {
            return ESP_ERR_TIMEOUT;
        }
        *locked = true;
    }
    return ESP_OK;
}

static void dm9058_ptp_unlock_if_needed(esp_eth_ptp_dm9058_t *ptp, bool locked)
{
    if (locked && ptp->ops.unlock != NULL) {
        ptp->ops.unlock(ptp->io_ctx);
    }
}

static inline void dm9058_ptp_delay_us(esp_eth_ptp_dm9058_t *ptp, uint32_t us)
{
    if (ptp->ops.delay_us) {
        ptp->ops.delay_us(us);
    }
}

static inline void dm9058_ptp_delay_ms(esp_eth_ptp_dm9058_t *ptp, uint32_t ms)
{
    if (ptp->ops.delay_ms) {
        ptp->ops.delay_ms(ms);
    } else {
        dm9058_ptp_delay_us(ptp, ms * 1000U);
    }
}

static esp_err_t dm9058_ptp_read_bytes(esp_eth_ptp_dm9058_t *ptp, uint8_t reg, uint8_t *buffer, size_t len)
{
    if (ptp->ops.reg_burst_read) {
        return ptp->ops.reg_burst_read(ptp->io_ctx, reg, buffer, len);
    }
    for (size_t i = 0; i < len; i++) {
        esp_err_t ret = ptp->ops.reg_read(ptp->io_ctx, reg, &buffer[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

static esp_err_t dm9058_ptp_write_bytes(esp_eth_ptp_dm9058_t *ptp, uint8_t reg, const uint8_t *buffer, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        esp_err_t ret = ptp->ops.reg_write(ptp->io_ctx, reg, buffer[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

static inline void dm9058_ptp_encode_time(const esp_eth_ptp_dm9058_time_t *time, uint8_t out[8])
{
    out[0] = (uint8_t)(time->nanoseconds);
    out[1] = (uint8_t)(time->nanoseconds >> 8);
    out[2] = (uint8_t)(time->nanoseconds >> 16);
    out[3] = (uint8_t)(time->nanoseconds >> 24);
    out[4] = (uint8_t)(time->seconds);
    out[5] = (uint8_t)(time->seconds >> 8);
    out[6] = (uint8_t)(time->seconds >> 16);
    out[7] = (uint8_t)(time->seconds >> 24);
}

static inline void dm9058_ptp_decode_time(const uint8_t in[8], esp_eth_ptp_dm9058_time_t *time)
{
    time->nanoseconds = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
    time->seconds = (uint32_t)in[4] | ((uint32_t)in[5] << 8) | ((uint32_t)in[6] << 16) | ((uint32_t)in[7] << 24);
}

esp_err_t esp_eth_ptp_dm9058_init(esp_eth_ptp_dm9058_t *ptp, void *io_ctx, const esp_eth_ptp_dm9058_ops_t *ops)
{
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "null ptp handle");
    ESP_RETURN_ON_FALSE(ops != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "null ptp ops");
    ESP_RETURN_ON_FALSE(ops->reg_read && ops->reg_write, ESP_ERR_INVALID_ARG, PTP_TAG, "missing reg access callbacks");

    memset(ptp, 0, sizeof(*ptp));
    ptp->io_ctx = io_ctx;
    ptp->ops = *ops;
    ptp->initialized = true;
    return ESP_OK;
}

esp_err_t esp_eth_ptp_dm9058_enable(esp_eth_ptp_dm9058_t *ptp, bool enable)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && ptp->initialized, ESP_ERR_INVALID_STATE, PTP_TAG, "ptp not initialized");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    /* Layer 2 (IEEE 802.3 / EtherType 0x88F7) offsets */
    const uint8_t ts_offset = 0x32;
    const uint8_t checksum_offset = 0x20;

    /* App may enable PTP before esp_eth_clock_init(); avoid a second full HW re-init that can fail or race. */
    if (enable && ptp->enabled) {
        return ESP_OK;
    }

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, PTP_TAG, "lock timeout");

    if (!enable) {
        ret = ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, 0x00);
        ESP_GOTO_ON_ERROR(ret, err, PTP_TAG, "disable ptp failed");
        ptp->enabled = false;
        goto err;
    }

    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CR, PTP_CR_RESTART), err, PTP_TAG, "ptp reset assert failed");
    dm9058_ptp_delay_ms(ptp, 1);
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CR, 0x00), err, PTP_TAG, "ptp reset deassert failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, PTP_ENR_ENABLE), err, PTP_TAG, "ptp enable failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_TCR, 0x00), err, PTP_TAG, "clear tx control failed");
    /* Keep PTP RXCR cleared: enabling HW RX timestamps can disturb RX MEM read on some boards (see DM9058 MAC notes). */
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_RXCR, 0x00), err, PTP_TAG, "write PTP_RXCR failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ONESTEP, 0x00), err, PTP_TAG, "disable one-step failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_TSOFF, ts_offset), err, PTP_TAG, "set ts offset failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CSOFF, checksum_offset), err, PTP_TAG, "set checksum offset failed");
    ptp->enabled = true;
    ptp->last_rate = 0;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_get_time(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && time != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, PTP_TAG, "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[8] = {0};

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, PTP_TAG, "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, PTP_ENR_RSTIDX | PTP_ENR_GETTIME), err, PTP_TAG, "prepare get time failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_read_bytes(ptp, DM9058_PTP_DATA, raw, sizeof(raw)), err, PTP_TAG, "read time failed");
    dm9058_ptp_decode_time(raw, time);

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_set_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && time != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, PTP_TAG, "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[8];
    dm9058_ptp_encode_time(time, raw);

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, PTP_TAG, "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CR, PTP_CR_RESTART), err, PTP_TAG, "ptp reset assert failed");
    dm9058_ptp_delay_us(ptp, 2);
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CR, 0x00), err, PTP_TAG, "ptp reset deassert failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, PTP_ENR_RSTIDX), err, PTP_TAG, "reset index failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_write_bytes(ptp, DM9058_PTP_DATA, raw, sizeof(raw)), err, PTP_TAG, "write time failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, PTP_ENR_SETTIME | PTP_ENR_ENABLE), err, PTP_TAG, "apply set time failed");
    ptp->last_rate = 0;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_adj_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *offset)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && offset != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");

    esp_eth_ptp_dm9058_time_t current = {0};
    ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_get_time(ptp, &current), PTP_TAG, "get current time failed");

    int64_t sec = (int64_t)current.seconds + (int32_t)offset->seconds;
    int64_t nsec = (int64_t)current.nanoseconds + (int32_t)offset->nanoseconds;
    while (nsec >= 1000000000LL) {
        sec++;
        nsec -= 1000000000LL;
    }
    while (nsec < 0) {
        sec--;
        nsec += 1000000000LL;
    }
    if (sec < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_eth_ptp_dm9058_time_t updated = {
        .seconds = (uint32_t)sec,
        .nanoseconds = (uint32_t)nsec,
    };
    return esp_eth_ptp_dm9058_set_time(ptp, &updated);
}

esp_err_t esp_eth_ptp_dm9058_adj_freq(esp_eth_ptp_dm9058_t *ptp, int32_t adj_ppb)
{
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, PTP_TAG, "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[4];
    int64_t signed_addend = (int64_t)(adj_ppb * V51_ADJ_FREQ_BASE_ADDEND);
    int64_t delta = signed_addend - ptp->last_rate;
    uint32_t adjustment;
    uint8_t control_value;

    if (delta < 0) {
        adjustment = (uint32_t)(-delta);
        control_value = PTP_ADJUST_SLOWER_CTRL;
    } else {
        adjustment = (uint32_t)delta;
        control_value = PTP_ADJUST_FASTER_CTRL;
    }
    if (adjustment > DM9058_PTP_MAX_ADJUSTMENT) {
        adjustment = DM9058_PTP_MAX_ADJUSTMENT;
    }

    raw[0] = (uint8_t)adjustment;
    raw[1] = (uint8_t)(adjustment >> 8);
    raw[2] = (uint8_t)(adjustment >> 16);
    raw[3] = (uint8_t)(adjustment >> 24);

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, PTP_TAG, "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, PTP_ENR_RSTIDX), err, PTP_TAG, "reset index failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_write_bytes(ptp, DM9058_PTP_DATA, raw, sizeof(raw)), err, PTP_TAG, "write freq adjust failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, control_value), err, PTP_TAG, "apply freq adjust failed");
    ptp->last_rate = signed_addend;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_get_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && time != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, PTP_TAG, "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[8] = {0};

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, PTP_TAG, "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, PTP_ENR_RSTIDX), err, PTP_TAG, "reset index failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_TXCR, PTP_TXCR_READTS), err, PTP_TAG, "set tx timestamp mode failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_read_bytes(ptp, DM9058_PTP_DATA, raw, sizeof(raw)), err, PTP_TAG, "read tx timestamp failed");
    dm9058_ptp_decode_time(raw, time);

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_set_tx_mode(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_tx_mode_t mode)
{
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, PTP_TAG, "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t tx_mode_val = (mode == ESP_ETH_PTP_DM9058_TX_MODE_ONE_STEP) ? 0x01 : 0x00;
    uint8_t tcr_val = 0;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, PTP_TAG, "lock timeout");

    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ONESTEP, tx_mode_val), err, PTP_TAG, "set tx mode failed");

    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_TCR, &tcr_val), err, PTP_TAG, "read tcr failed");
    if (mode == ESP_ETH_PTP_DM9058_TX_MODE_ONE_STEP) {
        tcr_val |= DM9058_TCR_TS1STEP_EMIT;
    } else {
        tcr_val &= ~DM9058_TCR_TS1STEP_EMIT;
    }
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_TCR, tcr_val), err, PTP_TAG, "write tcr failed");

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_enable_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, bool enable)
{
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, PTP_TAG, "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t tcr_val = 0;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, PTP_TAG, "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_TCR, &tcr_val), err, PTP_TAG, "read tcr failed");

    if (enable) {
        tcr_val |= DM9058_TCR_TSEN_CAP;
    } else {
        tcr_val &= ~DM9058_TCR_TSEN_CAP;
    }

    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_TCR, tcr_val), err, PTP_TAG, "write tcr failed");

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

static bool is_valid_ptp_packet_l2(const uint8_t *packet, size_t len)
{
    if (len < ETH_HLEN + 2) {
        return false;
    }
    uint16_t ethertype = ((uint16_t)packet[12] << 8) | packet[13];
    return (ethertype == ETH_TYPE_PTP) && (len >= ETH_HLEN + 34);
}

static bool get_ptp_info_l2(const uint8_t *packet, size_t len, uint8_t *msg_type, bool *two_step_flag)
{
    if (!is_valid_ptp_packet_l2(packet, len)) {
        return false;
    }
    const uint8_t *ptp_hdr = packet + ETH_HLEN;
    *msg_type = ptp_hdr[0] & 0x0F;
    uint16_t flags = ((uint16_t)ptp_hdr[6] << 8) | ptp_hdr[7];
    *two_step_flag = (flags & PTP_FLAG_TWO_STEP) != 0;
    return true;
}

esp_err_t esp_eth_ptp_dm9058_parse_tx_packet(const uint8_t *packet, size_t len, bool two_step_mode,
                                             esp_eth_ptp_dm9058_tx_config_t *config)
{
    ESP_RETURN_ON_FALSE(packet != NULL && config != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");

    config->enable_timestamp_capture = false;
    config->enable_onestep_insert = false;

    uint8_t msg_type;
    bool pkt_two_step_flag;
    (void)pkt_two_step_flag;

    if (!get_ptp_info_l2(packet, len, &msg_type, &pkt_two_step_flag)) {
        return ESP_OK;
    }

    switch (msg_type) {
    case ESP_ETH_PTP_DM9058_MSG_SYNC:
        if (!two_step_mode) {
            config->enable_onestep_insert = true;
        } else {
            config->enable_timestamp_capture = true;
        }
        break;

    case ESP_ETH_PTP_DM9058_MSG_DELAY_REQ:
        config->enable_timestamp_capture = true;
        if (!two_step_mode) {
            config->enable_onestep_insert = true;
        }
        break;

    case ESP_ETH_PTP_DM9058_MSG_PDELAY_REQ:
    case ESP_ETH_PTP_DM9058_MSG_PDELAY_RESP:
        config->enable_timestamp_capture = true;
        break;

    default:
        break;
    }

    return ESP_OK;
}

static esp_err_t esp_eth_ptp_dm9058_prepare_tx_internal(esp_eth_ptp_dm9058_t *ptp, const uint8_t *packet,
                                                      size_t len, bool two_step_mode, bool take_lock)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && packet != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, PTP_TAG, "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    esp_eth_ptp_dm9058_tx_config_t tx_config;
    uint8_t tcr_val = 0;

    ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_parse_tx_packet(packet, len, two_step_mode, &tx_config),
                        PTP_TAG, "parse packet failed");

    if (take_lock) {
        ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, PTP_TAG, "lock timeout");
    }

    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_TCR, &tcr_val), err, PTP_TAG, "read tcr failed");

    if (tx_config.enable_timestamp_capture) {
        tcr_val |= DM9058_TCR_TSEN_CAP;
    } else {
        tcr_val &= ~DM9058_TCR_TSEN_CAP;
    }

    if (tx_config.enable_onestep_insert) {
        tcr_val |= DM9058_TCR_TS1STEP_EMIT;
    } else {
        tcr_val &= ~DM9058_TCR_TS1STEP_EMIT;
    }

    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_TCR, tcr_val), err, PTP_TAG, "write tcr failed");

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_prepare_tx(esp_eth_ptp_dm9058_t *ptp, const uint8_t *packet,
                                        size_t len, bool two_step_mode)
{
    return esp_eth_ptp_dm9058_prepare_tx_internal(ptp, packet, len, two_step_mode, true);
}

esp_err_t esp_eth_ptp_dm9058_prepare_tx_locked(esp_eth_ptp_dm9058_t *ptp, const uint8_t *packet,
                                               size_t len, bool two_step_mode)
{
    return esp_eth_ptp_dm9058_prepare_tx_internal(ptp, packet, len, two_step_mode, false);
}

esp_err_t esp_eth_ptp_dm9058_rx_ready(esp_eth_ptp_dm9058_t *ptp, bool *ready)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && ready != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized, ESP_ERR_INVALID_STATE, PTP_TAG, "ptp not initialized");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t nsr = 0;
    uint8_t rx_flag = 0;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, PTP_TAG, "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_NSR, &nsr), err, PTP_TAG, "read nsr failed");

    if ((nsr & NSR_RXRDY) == 0) {
        *ready = false;
        goto err;
    }

    /* Match esp_eth_mac_dm9058.c: dummy MRCMDX, valid MRCMDX1 */
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_MRCMDX, &rx_flag), err, PTP_TAG, "dummy read MRCMDX failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_MRCMDX1, &rx_flag), err, PTP_TAG, "read MRCMDX1 failed");

    if (rx_flag != 0x01) {
        *ready = false;
        ret = ESP_ERR_INVALID_RESPONSE;
        goto err;
    }

    *ready = true;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_parse_rx_header(const uint8_t *rx_header, size_t rx_header_len,
                                             uint16_t max_packet_len, esp_eth_ptp_dm9058_rx_info_t *info)
{
    ESP_RETURN_ON_FALSE(rx_header != NULL && info != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(rx_header_len >= 4, ESP_ERR_INVALID_ARG, PTP_TAG, "rx header must be 4 bytes");

    uint8_t rx_status = rx_header[1];
    uint16_t packet_len = (uint16_t)rx_header[2] | ((uint16_t)rx_header[3] << 8);

    ESP_RETURN_ON_FALSE((rx_status & (DM9058_PTP_RXHDR_ERR_BITS & ~DM9058_PTP_RXHDR_PTP_BITS)) == 0,
                        ESP_ERR_INVALID_RESPONSE, PTP_TAG, "rx status error");
    ESP_RETURN_ON_FALSE(packet_len <= max_packet_len, ESP_ERR_INVALID_SIZE, PTP_TAG, "rx length too large");

    info->packet_len = packet_len;
    info->rx_status = rx_status;
    info->timestamp_available = (rx_status & DM9058_PTP_RXHDR_RXTS_EN) != 0;
    info->timestamp_len = 0;

    if (info->timestamp_available) {
        info->timestamp_len = (rx_status & DM9058_PTP_RXHDR_RXTS_LEN) ? 8 : 4;
    }

    return ESP_OK;
}

esp_err_t esp_eth_ptp_dm9058_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time)
{
    return esp_eth_ptp_dm9058_get_tx_timestamp(ptp, time);
}

esp_err_t esp_eth_ptp_dm9058_rx_timestamp(const uint8_t *rx_ts_buffer, size_t rx_ts_len, esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(rx_ts_buffer != NULL && time != NULL, ESP_ERR_INVALID_ARG, PTP_TAG, "invalid args");
    ESP_RETURN_ON_FALSE(rx_ts_len == 4 || rx_ts_len == 8, ESP_ERR_INVALID_ARG, PTP_TAG, "rx timestamp length must be 4 or 8");

    memset(time, 0, sizeof(*time));
    time->nanoseconds = (uint32_t)rx_ts_buffer[7] |
                        ((uint32_t)rx_ts_buffer[6] << 8) |
                        ((uint32_t)rx_ts_buffer[5] << 16) |
                        ((uint32_t)rx_ts_buffer[4] << 24);
    if (rx_ts_len == 8) {
        time->seconds = (uint32_t)rx_ts_buffer[3] |
                        ((uint32_t)rx_ts_buffer[2] << 8) |
                        ((uint32_t)rx_ts_buffer[1] << 16) |
                        ((uint32_t)rx_ts_buffer[0] << 24);
    }
    return ESP_OK;
}

#endif // CONFIG_ETH_SPI_ETHERNET_DM9058
