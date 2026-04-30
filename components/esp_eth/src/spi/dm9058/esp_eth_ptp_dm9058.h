#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_eth_mac_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t seconds;
    uint32_t nanoseconds;
} esp_eth_ptp_dm9058_time_t;

typedef enum {
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4 = 0,
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6 = 1,
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3 = 2,
} esp_eth_ptp_dm9058_transport_t;

typedef struct {
    esp_err_t (*reg_read)(void *io_ctx, uint8_t reg, uint8_t *value);
    esp_err_t (*reg_write)(void *io_ctx, uint8_t reg, uint8_t value);
    esp_err_t (*reg_burst_read)(void *io_ctx, uint8_t reg, uint8_t *buffer, size_t len);
    esp_err_t (*reg_burst_write)(void *io_ctx, uint8_t reg, const uint8_t *buffer, size_t len);
    void (*delay_us)(uint32_t us);
    void (*delay_ms)(uint32_t ms);
    bool (*lock)(void *io_ctx);
    void (*unlock)(void *io_ctx);
} esp_eth_ptp_dm9058_ops_t;

typedef struct {
    void *io_ctx;
    esp_eth_ptp_dm9058_ops_t ops;
    bool initialized;
    bool enabled;
    int64_t last_rate;
} esp_eth_ptp_dm9058_t;

typedef enum {
    ESP_ETH_PTP_DM9058_MSG_SYNC        = 0x0,
    ESP_ETH_PTP_DM9058_MSG_DELAY_REQ   = 0x1,
    ESP_ETH_PTP_DM9058_MSG_PDELAY_REQ  = 0x2,
    ESP_ETH_PTP_DM9058_MSG_PDELAY_RESP = 0x3,
} esp_eth_ptp_dm9058_msg_type_t;

typedef struct {
    bool enable_timestamp_capture;
    bool enable_onestep_insert;
} esp_eth_ptp_dm9058_tx_config_t;

typedef struct {
    uint16_t packet_len;
    uint8_t rx_status;
    bool timestamp_available;
    size_t timestamp_len;
} esp_eth_ptp_dm9058_rx_info_t;

esp_err_t esp_eth_ptp_dm9058_init(esp_eth_ptp_dm9058_t *ptp, void *io_ctx, const esp_eth_ptp_dm9058_ops_t *ops);
esp_err_t esp_eth_ptp_dm9058_enable(esp_eth_ptp_dm9058_t *ptp, bool enable, esp_eth_ptp_dm9058_transport_t transport);
esp_err_t esp_eth_ptp_dm9058_get_time(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time);
esp_err_t esp_eth_ptp_dm9058_set_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *time);
esp_err_t esp_eth_ptp_dm9058_adj_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *offset);
esp_err_t esp_eth_ptp_dm9058_adj_freq(esp_eth_ptp_dm9058_t *ptp, int32_t adj_ppb);
esp_err_t esp_eth_ptp_dm9058_get_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time);
esp_err_t esp_eth_ptp_dm9058_parse_tx_packet(const uint8_t *packet, size_t len, bool two_step_mode,
                                             esp_eth_ptp_dm9058_tx_config_t *config);
esp_err_t esp_eth_ptp_dm9058_prepare_tx(esp_eth_ptp_dm9058_t *ptp, const uint8_t *packet, size_t len,
                                        bool two_step_mode);
esp_err_t esp_eth_ptp_dm9058_parse_rx_header(const uint8_t *rx_header, size_t rx_header_len,
                                             uint16_t max_packet_len, esp_eth_ptp_dm9058_rx_info_t *info);
esp_err_t esp_eth_ptp_dm9058_rx_timestamp(const uint8_t *rx_ts_buffer, size_t rx_ts_len,
                                          esp_eth_ptp_dm9058_time_t *time);

#ifdef __cplusplus
}
#endif
