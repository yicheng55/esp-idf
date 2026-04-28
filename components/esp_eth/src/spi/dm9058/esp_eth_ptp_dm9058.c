#include <string.h>
#include "esp_check.h"
#include "esp_eth_ptp_dm9058.h"

static const char *PTP_TAG = "dm9058.ptp";

/* DM9058 PTP registers */
#define DM9058_NSR   (0x01)
#define DM9058_MRCMDX (0x70)
#define DM9058_PTPCR  (0x60)
#define DM9058_PTPCW  (0x61)
#define DM9058_PTPTSM (0x62)
#define DM9058_PTPTX  (0x63)
#define DM9058_PTPMMP (0x64)
#define DM9058_PTPTSO (0x65)
#define DM9058_PTPCSO (0x66)
#define DM9058_PTPTS  (0x68)

/* TX Control Register (0x02) - PTP bits */
#define DM9058_TCR    (0x02)
#define DM9058_TCR_TSEN_CAP      (1 << 7)  /* Enable TX timestamp capture */
#define DM9058_TCR_TS1STEP_EMIT  (1 << 6)  /* Enable one-step timestamp insertion */

#define DM9058_PTP_TCR_ENABLE             (0x01)
#define DM9058_PTP_TCR_RESET_INDEX        (0x80)
#define DM9058_PTP_TCR_READ_CLOCK         (0x84)
#define DM9058_PTP_TCR_APPLY_SET_TIME     (0x09)
#define DM9058_PTP_TCR_APPLY_OFFSET       (0x10)
#define DM9058_PTP_TCR_APPLY_ADJUST_FAST  (0x20)
#define DM9058_PTP_TCR_APPLY_ADJUST_SLOW  (0x60)
#define DM9058_PTP_MAX_ADJUSTMENT         (0xEFFFFFFFU)

/* 2^32 * 40 / 1e9 in Q16 format */
#define V51_ADJ_FREQ_BASE_ADDEND     171.7987 /* Base addend for frequency adjustment */
#define DM9058_PTP_FREQ_BASE_ADDEND_Q16   (11259106)

/* Network protocol constants for packet parsing */
#define ETH_HLEN              14
#define ETH_TYPE_IPV4         0x0800
#define ETH_TYPE_IPV6         0x86DD
#define ETH_TYPE_PTP          0x88F7
#define IP_PROTO_UDP          17
#define PTP_EVENT_PORT        319
#define PTP_GENERAL_PORT      320

/* PTP message flag bits */
#define PTP_FLAG_TWO_STEP     (1 << 9)  /* Bit 1 of flagField (byte 6-7, big-endian) */

/* RX status bits used in DM9058 RX 4-byte header */
#define DM9058_RSR_RF         (1 << 7)
#define DM9058_RSR_MF         (1 << 6)
#define DM9058_RSR_LCS        (1 << 5)
#define DM9058_RSR_RWTO       (1 << 4)
#define DM9058_RSR_PLE        (1 << 3)
#define DM9058_RSR_AE         (1 << 2)
#define DM9058_RSR_CE         (1 << 1)
#define DM9058_RSR_FOE        (1 << 0)

#define DM9058_RSR_RXTS_EN    (1 << 5)
#define DM9058_RSR_RXTS_PARITY (1 << 3)
#define DM9058_RSR_RXTS_LEN   (1 << 2)
#define DM9058_RSR_PTP_BITS   (DM9058_RSR_RXTS_EN | DM9058_RSR_RXTS_PARITY | DM9058_RSR_RXTS_LEN)
#define DM9058_RSR_ERR_BITS   (DM9058_RSR_RF | DM9058_RSR_LCS | DM9058_RSR_RWTO | DM9058_RSR_PLE | DM9058_RSR_AE | DM9058_RSR_CE | DM9058_RSR_FOE)

#define DM9058_NSR_RXRDY      (1 << 0)

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
    // if (ptp->ops.reg_burst_write) {
    //     return ptp->ops.reg_burst_write(ptp->io_ctx, reg, buffer, len);
    // }
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
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "null ptp handle");
    ESP_RETURN_ON_FALSE(ops != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "null ptp ops");
    ESP_RETURN_ON_FALSE(ops->reg_read && ops->reg_write, ESP_ERR_INVALID_ARG, "dm9058.ptp", "missing reg access callbacks");

    memset(ptp, 0, sizeof(*ptp));
    ptp->io_ctx = io_ctx;
    ptp->ops = *ops;
    ptp->initialized = true;
    return ESP_OK;
}

esp_err_t esp_eth_ptp_dm9058_enable(esp_eth_ptp_dm9058_t *ptp, bool enable, esp_eth_ptp_dm9058_transport_t transport)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && ptp->initialized, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not initialized");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t ts_offset = 0x4E;
    uint8_t checksum_offset = 0x3C;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");

    if (!enable) {
        ret = ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, 0x00);
        ESP_GOTO_ON_ERROR(ret, err, "dm9058.ptp", "disable ptp failed");
        ptp->enabled = false;
        goto err;
    }

    switch (transport) {
    case ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6:
        ts_offset = 0x62;
        checksum_offset = 0x50;
        break;

    case ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS:
    case ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3:
        ts_offset = 0x32;
        checksum_offset = 0x20;
        break;
        
    case ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4:
    default:
        break;
    }

    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCR, 0x01), err, "dm9058.ptp", "ptp reset assert failed");
    dm9058_ptp_delay_ms(ptp, 1);
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCR, 0x00), err, "dm9058.ptp", "ptp reset deassert failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_ENABLE), err, "dm9058.ptp", "ptp enable failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, 0x02, 0x00), err, "dm9058.ptp", "clear tx control failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPMMP, 0x12), err, "dm9058.ptp", "set ptp mode failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPTX, 0x00), err, "dm9058.ptp", "disable one-step failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPTSO, ts_offset), err, "dm9058.ptp", "set ts offset failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCSO, checksum_offset), err, "dm9058.ptp", "set checksum offset failed");
    ptp->enabled = true;
    ptp->last_rate = 0;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_get_time(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && time != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[8] = {0};

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_READ_CLOCK), err, "dm9058.ptp", "prepare get time failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_read_bytes(ptp, DM9058_PTPTS, raw, sizeof(raw)), err, "dm9058.ptp", "read time failed");
    dm9058_ptp_decode_time(raw, time);

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_set_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && time != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[8];
    dm9058_ptp_encode_time(time, raw);

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCR, 0x01), err, "dm9058.ptp", "ptp reset assert failed");
    dm9058_ptp_delay_us(ptp, 2);
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCR, 0x00), err, "dm9058.ptp", "ptp reset deassert failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_RESET_INDEX), err, "dm9058.ptp", "reset index failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_write_bytes(ptp, DM9058_PTPTS, raw, sizeof(raw)), err, "dm9058.ptp", "write time failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_APPLY_SET_TIME), err, "dm9058.ptp", "apply set time failed");
    ptp->last_rate = 0;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_adj_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *offset)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && offset != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");

    esp_eth_ptp_dm9058_time_t current = {0};
    ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_get_time(ptp, &current), "dm9058.ptp", "get current time failed");

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
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[4];
    // int64_t signed_addend = ((int64_t)adj_ppb * (int64_t)DM9058_PTP_FREQ_BASE_ADDEND_Q16) >> 16;
    int64_t signed_addend = (int64_t)(adj_ppb * V51_ADJ_FREQ_BASE_ADDEND);
    int64_t delta = signed_addend - ptp->last_rate;
    uint32_t adjustment;
    uint8_t control_value;

    if (delta < 0) {
        adjustment = (uint32_t)(-delta);
        control_value = DM9058_PTP_TCR_APPLY_ADJUST_SLOW;
    } else {
        adjustment = (uint32_t)delta;
        control_value = DM9058_PTP_TCR_APPLY_ADJUST_FAST;
    }
    if (adjustment > DM9058_PTP_MAX_ADJUSTMENT) {
        adjustment = DM9058_PTP_MAX_ADJUSTMENT;
    }

    raw[0] = (uint8_t)adjustment;
    raw[1] = (uint8_t)(adjustment >> 8);
    raw[2] = (uint8_t)(adjustment >> 16);
    raw[3] = (uint8_t)(adjustment >> 24);

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_RESET_INDEX), err, "dm9058.ptp", "reset index failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_write_bytes(ptp, DM9058_PTPTS, raw, sizeof(raw)), err, "dm9058.ptp", "write freq adjust failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, control_value), err, "dm9058.ptp", "apply freq adjust failed");
    ptp->last_rate = signed_addend;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_get_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && time != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[8] = {0};

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_RESET_INDEX), err, "dm9058.ptp", "reset index failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPTSM, 0x01), err, "dm9058.ptp", "set tx timestamp mode failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_read_bytes(ptp, DM9058_PTPTS, raw, sizeof(raw)), err, "dm9058.ptp", "read tx timestamp failed");
    dm9058_ptp_decode_time(raw, time);

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_set_tx_mode(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_tx_mode_t mode)
{
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t tx_mode_val = (mode == ESP_ETH_PTP_DM9058_TX_MODE_ONE_STEP) ? 0x01 : 0x00;
    uint8_t tcr_val = 0;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");

    /* Set PTPTX register (0x63) for hardware one-step mode */
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPTX, tx_mode_val), err, "dm9058.ptp", "set tx mode failed");

    /* Set TCR register (0x02) bit 6 for one-step timestamp insertion control */
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_TCR, &tcr_val), err, "dm9058.ptp", "read tcr failed");
    if (mode == ESP_ETH_PTP_DM9058_TX_MODE_ONE_STEP) {
        tcr_val |= DM9058_TCR_TS1STEP_EMIT;
    } else {
        tcr_val &= ~DM9058_TCR_TS1STEP_EMIT;
    }
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_TCR, tcr_val), err, "dm9058.ptp", "write tcr failed");

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_enable_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, bool enable)
{
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t tcr_val = 0;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_TCR, &tcr_val), err, "dm9058.ptp", "read tcr failed");

    if (enable) {
        tcr_val |= DM9058_TCR_TSEN_CAP;
    } else {
        tcr_val &= ~DM9058_TCR_TSEN_CAP;
    }

    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_TCR, tcr_val), err, "dm9058.ptp", "write tcr failed");

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

/**
 * @brief Helper function to check if packet is valid PTP
 */
static bool is_valid_ptp_packet(const uint8_t *packet, size_t len)
{
    if (len < ETH_HLEN + 2) {
        return false;
    }

    /* Check EtherType */
    uint16_t ethertype = ((uint16_t)packet[12] << 8) | packet[13];

    /* Layer 2 PTP (EtherType 0x88F7) */
    if (ethertype == ETH_TYPE_PTP) {
        return len >= ETH_HLEN + 34;  /* Ethernet + PTP header */
    }

    /* Layer 3/4 PTP over IPv4 UDP */
    if (ethertype == ETH_TYPE_IPV4 && len >= ETH_HLEN + 20 + 8 + 34) {
        uint8_t protocol = packet[ETH_HLEN + 9];
        if (protocol == IP_PROTO_UDP) {
            /* Check UDP destination port (319 or 320) */
            uint16_t dest_port = ((uint16_t)packet[ETH_HLEN + 20 + 2] << 8) | packet[ETH_HLEN + 20 + 3];
            return (dest_port == PTP_EVENT_PORT || dest_port == PTP_GENERAL_PORT);
        }
    }

    return false;
}

/**
 * @brief Get PTP header offset and message type
 */
static bool get_ptp_info(const uint8_t *packet, size_t len, uint8_t *msg_type, bool *two_step_flag)
{
    if (!is_valid_ptp_packet(packet, len)) {
        return false;
    }

    uint16_t ethertype = ((uint16_t)packet[12] << 8) | packet[13];
    const uint8_t *ptp_hdr;

    if (ethertype == ETH_TYPE_PTP) {
        /* Layer 2 PTP */
        ptp_hdr = packet + ETH_HLEN;
    } else if (ethertype == ETH_TYPE_IPV4) {
        /* Layer 3/4 PTP over IPv4 */
        ptp_hdr = packet + ETH_HLEN + 20 + 8;  /* Skip Ethernet + IP + UDP */
    } else {
        return false;
    }

    /* PTP header format:
     * Byte 0: messageType (bits 0-3) and transportSpecific (bits 4-7)
     * Byte 1: reserved and messageLength
     * Bytes 6-7: flagField (big-endian)
     */
    *msg_type = ptp_hdr[0] & 0x0F;

    /* Check two-step flag (bit 1 of flagField, which is bit 9 overall) */
    uint16_t flags = ((uint16_t)ptp_hdr[6] << 8) | ptp_hdr[7];
    *two_step_flag = (flags & PTP_FLAG_TWO_STEP) != 0;

    return true;
}

esp_err_t esp_eth_ptp_dm9058_parse_tx_packet(const uint8_t *packet, size_t len, bool two_step_mode,
                                               esp_eth_ptp_dm9058_tx_config_t *config)
{
    ESP_RETURN_ON_FALSE(packet != NULL && config != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");

    /* Default: no special handling */
    config->enable_timestamp_capture = false;
    config->enable_onestep_insert = false;

    uint8_t msg_type;
    bool pkt_two_step_flag;

    if (!get_ptp_info(packet, len, &msg_type, &pkt_two_step_flag)) {
        /* Not a PTP packet, no timestamp needed */
        return ESP_OK;
    }

    /* Decision logic based on message type and mode:
     *
     * SYNC packet:
     *   - If one-step mode (!two_step_mode): enable one-step insert
     *   - If two-step mode: enable timestamp capture
     *
     * DELAY_REQ, PDELAY_REQ, PDELAY_RESP:
     *   - Always enable timestamp capture
     *   - For DELAY_REQ, optionally enable one-step insert
     */

    switch (msg_type) {
    case ESP_ETH_PTP_DM9058_MSG_SYNC:
        if (!two_step_mode) {
            /* One-step SYNC: hardware inserts timestamp */
            config->enable_onestep_insert = true;
        } else {
            /* Two-step SYNC: capture timestamp for Follow_Up */
            config->enable_timestamp_capture = true;
        }
        break;

    case ESP_ETH_PTP_DM9058_MSG_DELAY_REQ:
        config->enable_timestamp_capture = true;
        /* Optionally enable one-step for slave */
        config->enable_onestep_insert = true;
        break;

    case ESP_ETH_PTP_DM9058_MSG_PDELAY_REQ:
    case ESP_ETH_PTP_DM9058_MSG_PDELAY_RESP:
        /* P2P delay packets need timestamp capture */
        config->enable_timestamp_capture = true;
        break;

    default:
        /* Other message types (FOLLOW_UP, DELAY_RESP, ANNOUNCE, etc.)
         * don't need timestamp */
        break;
    }

    return ESP_OK;
}

static esp_err_t esp_eth_ptp_dm9058_prepare_tx_internal(esp_eth_ptp_dm9058_t *ptp, const uint8_t *packet,
                                                        size_t len, bool two_step_mode, bool take_lock)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && packet != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    esp_eth_ptp_dm9058_tx_config_t tx_config;
    uint8_t tcr_val = 0;

    /* Parse packet to determine TX configuration */
    ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_parse_tx_packet(packet, len, two_step_mode, &tx_config),
                        "dm9058.ptp", "parse packet failed");

    if (take_lock) {
        ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    }

    /* Read current TCR value */
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_TCR, &tcr_val), err, "dm9058.ptp", "read tcr failed");

    /* Configure TCR bits based on packet analysis */
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

    /* Write updated TCR value */
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_TCR, tcr_val), err, "dm9058.ptp", "write tcr failed");

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
    ESP_RETURN_ON_FALSE(ptp != NULL && ready != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not initialized");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t nsr = 0;
    uint8_t rx_flag = 0;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_NSR, &nsr), err, "dm9058.ptp", "read nsr failed");

    if ((nsr & DM9058_NSR_RXRDY) == 0) {
        *ready = false;
        goto err;
    }

    /* Follow cspi_read_rxb flow:
     * - 先讀一次 DM9058_MRCMDX（dummy）
     * - 再讀一次 DM9058_MRCMDX（有效值）
     */
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_MRCMDX, &rx_flag), err, "dm9058.ptp", "dummy read MRCMDX failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_MRCMDX, &rx_flag), err, "dm9058.ptp", "read MRCMDX failed");

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
    ESP_RETURN_ON_FALSE(rx_header != NULL && info != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(rx_header_len >= 4, ESP_ERR_INVALID_ARG, "dm9058.ptp", "rx header must be 4 bytes");

    uint8_t rx_status = rx_header[1];
    uint16_t packet_len = (uint16_t)rx_header[2] | ((uint16_t)rx_header[3] << 8);

    ESP_RETURN_ON_FALSE((rx_status & (DM9058_RSR_ERR_BITS & ~DM9058_RSR_PTP_BITS)) == 0,
                        ESP_ERR_INVALID_RESPONSE, "dm9058.ptp", "rx status error");
    ESP_RETURN_ON_FALSE(packet_len <= max_packet_len, ESP_ERR_INVALID_SIZE, "dm9058.ptp", "rx length too large");

    info->packet_len = packet_len;
    info->rx_status = rx_status;
    info->timestamp_available = (rx_status & DM9058_RSR_RXTS_EN) != 0;
    info->timestamp_len = 0;

    if (info->timestamp_available) {
        info->timestamp_len = (rx_status & DM9058_RSR_RXTS_LEN) ? 8 : 4;
    }

    return ESP_OK;
}

#if 0
esp_err_t esp_eth_ptp_dm9058_parse_rx_packet(esp_eth_ptp_dm9058_t *ptp,
                                             const uint8_t *rx_header, size_t rx_header_len,
                                             const uint8_t *rx_ts_buffer, size_t rx_ts_buffer_len,
                                             uint16_t max_packet_len, esp_eth_ptp_dm9058_rx_info_t *info,
                                             esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && info != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");

    /* Check if RX data is ready (similar to dm9058_rx_ptp checking cspi_rx_ready) */
    bool ready = false;
    ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_rx_ready(ptp, &ready), "dm9058.ptp", "rx ready check failed");

    if (!ready) {
        /* No data available, set packet_len to 0 (similar to dm9058_rx_ptp returning 0) */
        memset(info, 0, sizeof(*info));
        if (time != NULL) {
            memset(time, 0, sizeof(*time));
        }
        return ESP_OK;
    }

    /* Parse RX header */
    ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_parse_rx_header(rx_header, rx_header_len, max_packet_len, info),
                        "dm9058.ptp", "parse rx header failed");

    if (!info->timestamp_available) {
        if (time != NULL) {
            memset(time, 0, sizeof(*time));
        }
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(rx_ts_buffer != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "missing rx timestamp buffer");
    ESP_RETURN_ON_FALSE(rx_ts_buffer_len >= info->timestamp_len, ESP_ERR_INVALID_SIZE,
                        "dm9058.ptp", "rx timestamp buffer too small");

    if (time != NULL) {
        ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_rx_timestamp(rx_ts_buffer, info->timestamp_len, time),
                            "dm9058.ptp", "decode rx timestamp failed");
    }

    return ESP_OK;
}
#endif

esp_err_t esp_eth_ptp_dm9058_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time)
{
    return esp_eth_ptp_dm9058_get_tx_timestamp(ptp, time);
}

esp_err_t esp_eth_ptp_dm9058_rx_timestamp(const uint8_t *rx_ts_buffer, size_t rx_ts_len, esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(rx_ts_buffer != NULL && time != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(rx_ts_len == 4 || rx_ts_len == 8, ESP_ERR_INVALID_ARG, "dm9058.ptp", "rx timestamp length must be 4 or 8");

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
