/*
 * SPDX-FileCopyrightText: 2019-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_eth_mac_spi.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PTP transport layer selection for DM9058
 */
typedef enum {
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4 = 0,    /*!< PTP over UDP/IPv4 */
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6 = 1,    /*!< PTP over UDP/IPv6 */
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3 = 2,  /*!< PTP over Ethernet (L2) */
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS = 3, /*!< gPTP over Ethernet */
} esp_eth_ptp_dm9058_transport_t;

/**
 * @brief PTP enable configuration for DM9058
 */
typedef struct {
    bool enable;                                  /*!< Enable or disable PTP */
    esp_eth_ptp_dm9058_transport_t transport;     /*!< PTP transport layer */
} esp_eth_ptp_dm9058_enable_config_t;

/**
 * @brief DM9058 specific configuration
 */
typedef struct {
    int int_gpio_num;                                   /*!< Interrupt GPIO number, set -1 to not use interrupt and to poll rx status periodically */
    uint32_t poll_period_ms;                            /*!< Period in ms to poll rx status when interrupt mode is not used */
    spi_host_device_t spi_host_id;                      /*!< SPI peripheral (this field is invalid when custom SPI driver is defined) */
    spi_device_interface_config_t *spi_devcfg;          /*!< SPI device configuration (this field is invalid when custom SPI driver is defined) */
    eth_spi_custom_driver_config_t custom_spi_driver;   /*!< Custom SPI driver definitions */
} eth_dm9058_config_t;

/**
 * @brief PTP time structure for DM9058
 *
 * Contains seconds and nanoseconds components of PTP time.
 */
typedef struct {
    uint32_t seconds;       /*!< Seconds component */
    uint32_t nanoseconds;   /*!< Nanoseconds component */
} eth_dm9058_ptp_time_t;

/**
 * @brief DM9058 MAC custom ioctl commands
 */
typedef enum {
    ETH_MAC_DM9058_CMD_PTP_ENABLE = 0,          /*!< Enable/disable PTP with transport type */
    ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS = 1,    /*!< Auto-process PTP packets */
    ETH_MAC_DM9058_CMD_S_PTP_TIME = 2,          /*!< Set PTP time */
    ETH_MAC_DM9058_CMD_G_PTP_TIME = 3,          /*!< Get PTP time */
    ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ = 4,        /*!< Adjust PTP frequency (ppb) */
    ETH_MAC_DM9058_CMD_ADJ_PTP_TIME = 5,        /*!< Adjust PTP time offset */
    ETH_MAC_DM9058_CMD_G_PTP_TX_TIME = 6,       /*!< Get TX timestamp */
    ETH_MAC_DM9058_CMD_G_PTP_RX_TIME = 7,       /*!< Get RX timestamp (deprecated) */
    ETH_MAC_DM9058_CMD_S_TARGET_TIME = 8,       /*!< Set target time for interrupt */
    ETH_MAC_DM9058_CMD_S_TARGET_CB = 9,         /*!< Set target time callback */
} eth_mac_dm9058_cmd_t;

/* Command aliases for compatibility with example code */
#define ETH_MAC_ESP_CMD_PTP_ENABLE         ETH_MAC_DM9058_CMD_PTP_ENABLE
#define ETH_MAC_ESP_CMD_PTP_AUTO_PROCESS   ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS
#define ETH_MAC_ESP_CMD_S_PTP_TIME         ETH_MAC_DM9058_CMD_S_PTP_TIME
#define ETH_MAC_ESP_CMD_G_PTP_TIME         ETH_MAC_DM9058_CMD_G_PTP_TIME
#define ETH_MAC_ESP_CMD_ADJ_PTP_FREQ       ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ
#define ETH_MAC_ESP_CMD_ADJ_PTP_TIME       ETH_MAC_DM9058_CMD_ADJ_PTP_TIME
#define ETH_MAC_ESP_CMD_G_PTP_TX_TIME      ETH_MAC_DM9058_CMD_G_PTP_TX_TIME
#define ETH_MAC_ESP_CMD_G_PTP_RX_TIME      ETH_MAC_DM9058_CMD_G_PTP_RX_TIME
#define ETH_MAC_ESP_CMD_S_TARGET_TIME      ETH_MAC_DM9058_CMD_S_TARGET_TIME
#define ETH_MAC_ESP_CMD_S_TARGET_CB        ETH_MAC_DM9058_CMD_S_TARGET_CB

/**
 * @brief Default DM9058 specific configuration
 */
#define ETH_DM9058_DEFAULT_CONFIG(spi_host, spi_devcfg_p) \
    {                                           \
        .int_gpio_num = 4,                      \
        .poll_period_ms = 0,                    \
        .spi_host_id = spi_host,                \
        .spi_devcfg = spi_devcfg_p,             \
        .custom_spi_driver = ETH_DEFAULT_SPI,   \
    }

/**
 * @brief Create DM9058 Ethernet MAC instance
 *
 * @param dm9058_config DM9058 specific configuration
 * @param mac_config Ethernet MAC configuration
 *
 * @return
 *      - instance: create MAC instance successfully
 *      - NULL: create MAC instance failed because some error occurred
 */
esp_eth_mac_t *esp_eth_mac_new_dm9058(const eth_dm9058_config_t *dm9058_config, const eth_mac_config_t *mac_config);

/**
 * @brief Create DM9058 Ethernet PHY instance
 *
 * @param phy_config PHY configuration
 *
 * @return
 *      - instance: create PHY instance successfully
 *      - NULL: create PHY instance failed because some error occurred
 */
esp_eth_phy_t *esp_eth_phy_new_dm9058(const eth_phy_config_t *phy_config);

#ifdef __cplusplus
}
#endif
