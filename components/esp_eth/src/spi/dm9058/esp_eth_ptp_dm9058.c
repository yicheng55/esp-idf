#include <string.h>
#include "esp_check.h"
#include "dm9058.h"
#include "esp_eth_ptp_dm9058.h"

#define DM9058_TCR_TSEN_CAP      (1 << 7)
#define DM9058_TCR_TS1STEP_EMIT  (1 << 6)

#define DM9058_PTP_TCR_ENABLE             (0x01)
#define DM9058_PTP_TCR_RESET_INDEX        (0x80)
#define DM9058_PTP_TCR_READ_CLOCK         (0x84)
#define DM9058_PTP_TCR_APPLY_SET_TIME     (0x09)
#define DM9058_PTP_TCR_APPLY_ADJUST_FAST  (0x20)
#define DM9058_PTP_TCR_APPLY_ADJUST_SLOW  (0x60)
#define DM9058_PTP_MAX_ADJUSTMENT         (0xEFFFFFFFU)

#define DM9058_PTP_FREQ_BASE_ADDEND       (171.7987)

#define ETH_HLEN       14
#define ETH_TYPE_IPV4  0x0800
#define ETH_TYPE_IPV6  0x86DD
#define ETH_TYPE_PTP   0x88F7
#define IP_PROTO_UDP   17
#define PTP_EVENT_PORT 319
#define PTP_GENERAL_PORT 320

#define PTP_FLAG_TWO_STEP  (1 << 9)

#define DM9058_RSR_RXTS_EN  (1 << 5)
#define DM9058_RSR_RXTS_PARITY (1 << 3)
#define DM9058_RSR_RXTS_LEN (1 << 2)
#define DM9058_RSR_PTP_BITS (DM9058_RSR_RXTS_EN | DM9058_RSR_RXTS_PARITY | DM9058_RSR_RXTS_LEN)
#define DM9058_RSR_ERR_BITS (RSR_RF | RSR_LCS | RSR_RWTO | RSR_PLE | RSR_AE | RSR_CE | RSR_FOE)

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

static void dm9058_ptp_delay_us(esp_eth_ptp_dm9058_t *ptp, uint32_t us)
{
    if (ptp->ops.delay_us != NULL) {
        ptp->ops.delay_us(us);
    }
}

static void dm9058_ptp_delay_ms(esp_eth_ptp_dm9058_t *ptp, uint32_t ms)
{
    if (ptp->ops.delay_ms != NULL) {
        ptp->ops.delay_ms(ms);
    } else {
        dm9058_ptp_delay_us(ptp, ms * 1000U);
    }
}

static esp_err_t dm9058_ptp_read_bytes(esp_eth_ptp_dm9058_t *ptp, uint8_t reg, uint8_t *buffer, size_t len)
{
    if (ptp->ops.reg_burst_read != NULL) {
        return ptp->ops.reg_burst_read(ptp->io_ctx, reg, buffer, len);
    }
    for (size_t i = 0; i < len; i++) {
        ESP_RETURN_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, reg, &buffer[i]), "dm9058.ptp", "read byte failed");
    }
    return ESP_OK;
}

static esp_err_t dm9058_ptp_write_bytes(esp_eth_ptp_dm9058_t *ptp, uint8_t reg, const uint8_t *buffer, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        ESP_RETURN_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, reg, buffer[i]), "dm9058.ptp", "write byte failed");
    }
    return ESP_OK;
}

static void dm9058_ptp_encode_time(const esp_eth_ptp_dm9058_time_t *time, uint8_t out[8])
{
    out[0] = (uint8_t)time->nanoseconds;
    out[1] = (uint8_t)(time->nanoseconds >> 8);
    out[2] = (uint8_t)(time->nanoseconds >> 16);
    out[3] = (uint8_t)(time->nanoseconds >> 24);
    out[4] = (uint8_t)time->seconds;
    out[5] = (uint8_t)(time->seconds >> 8);
    out[6] = (uint8_t)(time->seconds >> 16);
    out[7] = (uint8_t)(time->seconds >> 24);
}

static void dm9058_ptp_decode_time(const uint8_t in[8], esp_eth_ptp_dm9058_time_t *time)
{
    time->nanoseconds = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
    time->seconds = (uint32_t)in[4] | ((uint32_t)in[5] << 8) | ((uint32_t)in[6] << 16) | ((uint32_t)in[7] << 24);
}

esp_err_t esp_eth_ptp_dm9058_init(esp_eth_ptp_dm9058_t *ptp, void *io_ctx, const esp_eth_ptp_dm9058_ops_t *ops)
{
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "null ptp handle");
    ESP_RETURN_ON_FALSE(ops != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "null ptp ops");
    ESP_RETURN_ON_FALSE(ops->reg_read != NULL && ops->reg_write != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "missing reg ops");

    memset(ptp, 0, sizeof(*ptp));
    ptp->io_ctx = io_ctx;
    ptp->ops = *ops;
    ptp->initialized = true;
    return ESP_OK;
}

esp_err_t esp_eth_ptp_dm9058_enable(esp_eth_ptp_dm9058_t *ptp, bool enable, esp_eth_ptp_dm9058_transport_t transport,
                                    bool hw_one_step_tx)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && ptp->initialized, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not initialized");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t ts_offset = 0x4E;
    uint8_t checksum_offset = 0x3C;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");

    if (!enable) {
        ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, 0), err, "dm9058.ptp", "disable ptp failed");
        ptp->enabled = false;
        goto err;
    }

    if (transport == ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6) {
        ts_offset = 0x62;
        checksum_offset = 0x50;
    } else if (transport == ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3) {
        ts_offset = 0x32;
        checksum_offset = 0x20;
    }

    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CR, PTP_CR_RESTART), err, "dm9058.ptp", "ptp reset assert failed");
    dm9058_ptp_delay_ms(ptp, 1);
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CR, 0), err, "dm9058.ptp", "ptp reset deassert failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, DM9058_PTP_TCR_ENABLE), err, "dm9058.ptp", "ptp enable failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_TCR, 0), err, "dm9058.ptp", "clear tx control failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_RXCR, PTP_RXCR_ENABLE | PTP_RXCR_MCAST), err, "dm9058.ptp", "set rx timestamp mode failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ONESTEP, hw_one_step_tx ? 1U : 0U), err, "dm9058.ptp", "set PTP one-step TX register failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_TSOFF, ts_offset), err, "dm9058.ptp", "set ts offset failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CSOFF, checksum_offset), err, "dm9058.ptp", "set checksum offset failed");
    ptp->enabled = true;
    ptp->last_rate = 0;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_update_hw_one_step_tx(esp_eth_ptp_dm9058_t *ptp, bool hw_one_step_tx)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ONESTEP, hw_one_step_tx ? 1U : 0U), err, "dm9058.ptp", "set PTP one-step TX register failed");

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
    uint8_t raw[8] = { 0 };

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, DM9058_PTP_TCR_READ_CLOCK), err, "dm9058.ptp", "prepare get time failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_read_bytes(ptp, DM9058_PTP_DATA, raw, sizeof(raw)), err, "dm9058.ptp", "read time failed");
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
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CR, PTP_CR_RESTART), err, "dm9058.ptp", "ptp reset assert failed");
    dm9058_ptp_delay_us(ptp, 2);
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_CR, 0), err, "dm9058.ptp", "ptp reset deassert failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, DM9058_PTP_TCR_RESET_INDEX), err, "dm9058.ptp", "reset index failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_write_bytes(ptp, DM9058_PTP_DATA, raw, sizeof(raw)), err, "dm9058.ptp", "write time failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, DM9058_PTP_TCR_APPLY_SET_TIME), err, "dm9058.ptp", "apply set time failed");
    ptp->last_rate = 0;
    /* Notify PPS / any registered restart hook after successful set_time */
    if (ptp->on_restart != NULL) {
        ptp->on_restart(ptp->on_restart_ctx);
    }

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_adj_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *offset)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && offset != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");

    esp_eth_ptp_dm9058_time_t current = { 0 };
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
    ESP_RETURN_ON_FALSE(sec >= 0, ESP_ERR_INVALID_ARG, "dm9058.ptp", "negative time");

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
    int64_t signed_addend = (int64_t)(adj_ppb * DM9058_PTP_FREQ_BASE_ADDEND);
    int64_t delta = signed_addend - ptp->last_rate;
    uint32_t adjustment = delta < 0 ? (uint32_t)-delta : (uint32_t)delta;
    uint8_t control_value = delta < 0 ? DM9058_PTP_TCR_APPLY_ADJUST_SLOW : DM9058_PTP_TCR_APPLY_ADJUST_FAST;

    if (adjustment > DM9058_PTP_MAX_ADJUSTMENT) {
        adjustment = DM9058_PTP_MAX_ADJUSTMENT;
    }
    raw[0] = (uint8_t)adjustment;
    raw[1] = (uint8_t)(adjustment >> 8);
    raw[2] = (uint8_t)(adjustment >> 16);
    raw[3] = (uint8_t)(adjustment >> 24);

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, DM9058_PTP_TCR_RESET_INDEX), err, "dm9058.ptp", "reset index failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_write_bytes(ptp, DM9058_PTP_DATA, raw, sizeof(raw)), err, "dm9058.ptp", "write freq adjust failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, control_value), err, "dm9058.ptp", "apply freq adjust failed");
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
    uint8_t raw[8] = { 0 };

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_ENR, DM9058_PTP_TCR_RESET_INDEX), err, "dm9058.ptp", "reset index failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTP_TXCR, PTP_TXCR_READTS), err, "dm9058.ptp", "set tx timestamp mode failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_read_bytes(ptp, DM9058_PTP_DATA, raw, sizeof(raw)), err, "dm9058.ptp", "read tx timestamp failed");
    dm9058_ptp_decode_time(raw, time);

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

static bool dm9058_ptp_header(const uint8_t *packet, size_t len, const uint8_t **ptp_hdr)
{
    if (len < ETH_HLEN + 1) {
        return false;
    }

    uint16_t ethertype = ((uint16_t)packet[12] << 8) | packet[13];
    if (ethertype == ETH_TYPE_PTP) {
        if (len < ETH_HLEN + 34) {
            return false;
        }
        *ptp_hdr = packet + ETH_HLEN;
        return true;
    }

    if (ethertype == ETH_TYPE_IPV4) {
        if (len < ETH_HLEN + 20 + 8 + 34) {
            return false;
        }
        size_t ip_offset = ETH_HLEN;
        uint8_t ihl = (packet[ip_offset] & 0x0f) * 4;
        if (ihl < 20 || len < ETH_HLEN + ihl + 8 + 34 || packet[ip_offset + 9] != IP_PROTO_UDP) {
            return false;
        }
        size_t udp_offset = ETH_HLEN + ihl;
        uint16_t sport = ((uint16_t)packet[udp_offset] << 8) | packet[udp_offset + 1];
        uint16_t dport = ((uint16_t)packet[udp_offset + 2] << 8) | packet[udp_offset + 3];
        if (sport != PTP_EVENT_PORT && sport != PTP_GENERAL_PORT && dport != PTP_EVENT_PORT && dport != PTP_GENERAL_PORT) {
            return false;
        }
        *ptp_hdr = packet + udp_offset + 8;
        return true;
    }

    if (ethertype == ETH_TYPE_IPV6) {
        if (len < ETH_HLEN + 40 + 8 + 34 || packet[ETH_HLEN + 6] != IP_PROTO_UDP) {
            return false;
        }
        size_t udp_offset = ETH_HLEN + 40;
        uint16_t sport = ((uint16_t)packet[udp_offset] << 8) | packet[udp_offset + 1];
        uint16_t dport = ((uint16_t)packet[udp_offset + 2] << 8) | packet[udp_offset + 3];
        if (sport != PTP_EVENT_PORT && sport != PTP_GENERAL_PORT && dport != PTP_EVENT_PORT && dport != PTP_GENERAL_PORT) {
            return false;
        }
        *ptp_hdr = packet + udp_offset + 8;
        return true;
    }

    return false;
}

esp_err_t esp_eth_ptp_dm9058_parse_tx_packet(const uint8_t *packet, size_t len, bool two_step_mode,
                                             esp_eth_ptp_dm9058_tx_config_t *config)
{
    ESP_RETURN_ON_FALSE(packet != NULL && config != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");

    memset(config, 0, sizeof(*config));
    const uint8_t *ptp_hdr = NULL;
    if (!dm9058_ptp_header(packet, len, &ptp_hdr)) {
        return ESP_OK;
    }

    uint8_t msg_type = ptp_hdr[0] & 0x0f;
    uint16_t flags = ((uint16_t)ptp_hdr[6] << 8) | ptp_hdr[7];
    bool packet_two_step = (flags & PTP_FLAG_TWO_STEP) != 0;
    bool use_two_step = two_step_mode || packet_two_step;

    switch (msg_type) {
    case ESP_ETH_PTP_DM9058_MSG_SYNC:
        if (use_two_step) {
            config->enable_timestamp_capture = true;
        } else {
            config->enable_onestep_insert = true;
        }
        break;
    case ESP_ETH_PTP_DM9058_MSG_DELAY_REQ:
        config->enable_timestamp_capture = true;
        config->enable_onestep_insert = true;
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

esp_err_t esp_eth_ptp_dm9058_prepare_tx(esp_eth_ptp_dm9058_t *ptp, const uint8_t *packet, size_t len,
                                        bool two_step_mode)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && packet != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    esp_eth_ptp_dm9058_tx_config_t tx_config = { 0 };
    uint8_t tcr_val = 0;

    ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_parse_tx_packet(packet, len, two_step_mode, &tx_config),
                        "dm9058.ptp", "parse packet failed");

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_read(ptp->io_ctx, DM9058_TCR, &tcr_val), err, "dm9058.ptp", "read tcr failed");
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
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_TCR, tcr_val), err, "dm9058.ptp", "write tcr failed");

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_parse_rx_header(const uint8_t *rx_header, size_t rx_header_len,
                                             uint16_t max_packet_len, esp_eth_ptp_dm9058_rx_info_t *info)
{
    ESP_RETURN_ON_FALSE(rx_header != NULL && info != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(rx_header_len >= 4, ESP_ERR_INVALID_ARG, "dm9058.ptp", "rx header too short");

    uint8_t rx_status = rx_header[1];
    uint16_t packet_len = (uint16_t)rx_header[2] | ((uint16_t)rx_header[3] << 8);

    ESP_RETURN_ON_FALSE((rx_status & (DM9058_RSR_ERR_BITS & ~DM9058_RSR_PTP_BITS)) == 0,
                        ESP_ERR_INVALID_RESPONSE, "dm9058.ptp", "rx status error");
    ESP_RETURN_ON_FALSE(packet_len <= max_packet_len, ESP_ERR_INVALID_SIZE, "dm9058.ptp", "rx length too large");

    info->packet_len = packet_len;
    info->rx_status = rx_status;
    info->timestamp_available = (rx_status & DM9058_RSR_RXTS_EN) != 0;
    info->timestamp_len = info->timestamp_available ? ((rx_status & DM9058_RSR_RXTS_LEN) ? 8 : 4) : 0;
    return ESP_OK;
}

esp_err_t esp_eth_ptp_dm9058_rx_timestamp(const uint8_t *rx_ts_buffer, size_t rx_ts_len,
                                          esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(rx_ts_buffer != NULL && time != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(rx_ts_len == 4 || rx_ts_len == 8, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid rx timestamp length");

    memset(time, 0, sizeof(*time));
    if (rx_ts_len == 8) {
        time->seconds = ((uint32_t)rx_ts_buffer[0] << 24) | ((uint32_t)rx_ts_buffer[1] << 16) |
                        ((uint32_t)rx_ts_buffer[2] << 8) | rx_ts_buffer[3];
        rx_ts_buffer += 4;
    }
    time->nanoseconds = ((uint32_t)rx_ts_buffer[0] << 24) | ((uint32_t)rx_ts_buffer[1] << 16) |
                        ((uint32_t)rx_ts_buffer[2] << 8) | rx_ts_buffer[3];
    return ESP_OK;
}
