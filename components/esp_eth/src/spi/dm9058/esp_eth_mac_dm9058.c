/*
 * SPDX-FileCopyrightText: 2019-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include <inttypes.h>
#include "esp_eth_mac_spi.h"
#include "driver/gpio.h"
#include "esp_private/gpio.h"
#include "soc/io_mux_reg.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "dm9058.h"
#include "sdkconfig.h"
#include "esp_rom_sys.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "esp_rom_crc.h"

static const char *TAG = "dm9058.mac";

#define DM9058_SPI_LOCK_TIMEOUT_MS      (50)
#define DM9058_PHY_OPERATION_TIMEOUT_US (1000)
#define DM9058_MULTI_REG_AXS_TIMEOUT_MS (50)
#define DM9058_RX_MEM_START_ADDR        (3072)
#define DM9058_RX_MEM_MAX_SIZE          (16384)
#define DM9058_RX_HDR_SIZE              (4)

#define DM9058_HASH_FILTER_TABLE_SIZE   (64)

typedef struct {
    uint8_t flag;        // 0 = no frame, 1 = frame received, others = possible memory pointer error
    uint8_t status;      // Events occurred between this and previous frame (the same format as RSR)
    uint8_t length_low;  // Low byte of received frame length
    uint8_t length_high; // High byte of received frame length
} dm9058_rx_header_t;

typedef struct {
    spi_device_handle_t hdl;
    SemaphoreHandle_t lock;
} eth_spi_info_t;

typedef struct {
    void *ctx;
    void *(*init)(const void *spi_config);
    esp_err_t (*deinit)(void *spi_ctx);
    esp_err_t (*read)(void *spi_ctx, uint32_t cmd, uint32_t addr, void *data, uint32_t data_len);
    esp_err_t (*write)(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *data, uint32_t data_len);
} eth_spi_custom_driver_t;

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    eth_spi_custom_driver_t spi;
    TaskHandle_t rx_task_hdl;
    SemaphoreHandle_t multi_reg_axs_mutex;
    uint32_t sw_reset_timeout_ms;
    int int_gpio_num;
    esp_timer_handle_t poll_timer;
    uint32_t poll_period_ms;
    uint8_t addr[ETH_ADDR_LEN];
    bool packets_remain;
    bool flow_ctrl_enabled;
    uint8_t *rx_buffer;
    uint8_t hash_filter_cnt[DM9058_HASH_FILTER_TABLE_SIZE];
    /* PTP related */
    bool ptp_enabled;
    int64_t ptp_last_rate;
    eth_dm9058_ptp_transport_t ptp_transport;
} emac_dm9058_t;

static void *dm9058_spi_init(const void *spi_config)
{
    void *ret = NULL;
    eth_dm9058_config_t *dm9058_config = (eth_dm9058_config_t *)spi_config;
    eth_spi_info_t *spi = calloc(1, sizeof(eth_spi_info_t));
    ESP_GOTO_ON_FALSE(spi, NULL, err, TAG, "no memory for SPI context data");

    /* SPI device init */
    spi_device_interface_config_t spi_devcfg;
    spi_devcfg = *(dm9058_config->spi_devcfg);
    if (dm9058_config->spi_devcfg->command_bits == 0 && dm9058_config->spi_devcfg->address_bits == 0) {
        /* configure default SPI frame format */
        spi_devcfg.command_bits = 1;
        spi_devcfg.address_bits = 7;
    } else {
        ESP_GOTO_ON_FALSE(dm9058_config->spi_devcfg->command_bits == 1 && dm9058_config->spi_devcfg->address_bits == 7,
                          NULL, err, TAG, "incorrect SPI frame format (command_bits/address_bits)");
    }

    ESP_GOTO_ON_FALSE(spi_bus_add_device(dm9058_config->spi_host_id, &spi_devcfg, &spi->hdl) == ESP_OK,
                      NULL, err, TAG, "adding device to SPI host #%i failed", dm9058_config->spi_host_id + 1);

    /* create mutex */
    spi->lock = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(spi->lock, NULL, err, TAG, "create lock failed");

    ret = spi;
    return ret;
err:
    if (spi) {
        if (spi->lock) {
            vSemaphoreDelete(spi->lock);
        }
        free(spi);
    }
    return ret;
}

static esp_err_t dm9058_spi_deinit(void *spi_ctx)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_bus_remove_device(spi->hdl);
    vSemaphoreDelete(spi->lock);

    free(spi);
    return ret;
}

static inline bool dm9058_spi_lock(eth_spi_info_t *spi)
{
    return xSemaphoreTake(spi->lock, pdMS_TO_TICKS(DM9058_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline bool dm9058_spi_unlock(eth_spi_info_t *spi)
{
    return xSemaphoreGive(spi->lock) == pdTRUE;
}

static esp_err_t dm9058_spi_write(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_transaction_t trans = {
        .cmd = cmd,
        .addr = addr,
        .length = 8 * len,
        .tx_buffer = value
    };
    if (dm9058_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        dm9058_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    return ret;
}

static esp_err_t dm9058_spi_read(void *spi_ctx, uint32_t cmd, uint32_t addr, void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_transaction_t trans = {
        .flags = len <= 4 ? SPI_TRANS_USE_RXDATA : 0,
        .cmd = cmd,
        .addr = addr,
        .length = 8 * len,
        .rx_buffer = value
    };
    if (dm9058_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        dm9058_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    if ((trans.flags & SPI_TRANS_USE_RXDATA) && len <= 4) {
        memcpy(value, trans.rx_data, len);
    }
    return ret;
}

static inline bool dm9058_mutex_lock(emac_dm9058_t *emac)
{
    return xSemaphoreTake(emac->multi_reg_axs_mutex, pdMS_TO_TICKS(DM9058_MULTI_REG_AXS_TIMEOUT_MS)) == pdTRUE;
}

static inline bool dm9058_mutex_unlock(emac_dm9058_t *emac)
{
    return xSemaphoreGive(emac->multi_reg_axs_mutex) == pdTRUE;
}

/**
 * @brief write value to dm9058 internal register
 */
static esp_err_t dm9058_register_write(emac_dm9058_t *emac, uint8_t reg_addr, uint8_t value)
{
    return emac->spi.write(emac->spi.ctx, DM9058_SPI_WR, reg_addr, &value, 1);
}

/**
 * @brief read value from dm9058 internal register
 */
static esp_err_t dm9058_register_read(emac_dm9058_t *emac, uint8_t reg_addr, uint8_t *value)
{
    return emac->spi.read(emac->spi.ctx, DM9058_SPI_RD, reg_addr, value, 1);
}

/**
 * @brief write buffer to dm9058 internal memory
 */
static esp_err_t dm9058_memory_write(emac_dm9058_t *emac, uint8_t *buffer, uint32_t len)
{
    return emac->spi.write(emac->spi.ctx, DM9058_SPI_WR, DM9058_MWCMD, buffer, len);
}

/**
 * @brief read buffer from dm9058 internal memory
 */
static esp_err_t dm9058_memory_read(emac_dm9058_t *emac, uint8_t *buffer, uint32_t len)
{
    return emac->spi.read(emac->spi.ctx, DM9058_SPI_RD, DM9058_MRCMD, buffer, len);
}

/**
 * @brief read mac address from internal registers
 */
static esp_err_t dm9058_get_mac_addr(emac_dm9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < ETH_ADDR_LEN; i++) {
        ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_PAR + i, &emac->addr[i]), err, TAG, "read PAR failed");
    }
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief set new mac address to internal registers
 */
static esp_err_t dm9058_set_mac_addr(emac_dm9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < ETH_ADDR_LEN; i++) {
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PAR + i, emac->addr[i]), err, TAG, "write PAR failed");
    }
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief clear multicast hash table
 */
static esp_err_t dm9058_clear_multicast_table(emac_dm9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < 8; i++) {
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_MAR + i, 0x00), err, TAG, "write MAR failed");
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t dm9058_hash_filter_modify(emac_dm9058_t *emac, uint8_t *addr, bool add)
{
    esp_err_t ret = ESP_OK;

    uint32_t crc = ~esp_rom_crc32_le(0, addr, ETH_ADDR_LEN);

    uint8_t hash_value = crc & 0x3F;
    uint8_t hash_group = hash_value / 8;
    uint8_t hash_bit = hash_value % 8;

    uint8_t mar;
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, (DM9058_MAR + hash_group), &mar), err, TAG, "read MAR failed");
    if (add) {
        mar |= (1 << hash_bit);
        emac->hash_filter_cnt[hash_value]++;
    } else {
        emac->hash_filter_cnt[hash_value]--;
        if (emac->hash_filter_cnt[hash_value] == 0) {
            mar &= ~(1 << hash_bit);
        }
    }
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, (DM9058_MAR + hash_group), mar), err, TAG, "write MAR failed");

err:
    return ret;
}

static esp_err_t emac_dm9058_add_mac_filter(esp_eth_mac_t *mac, uint8_t *addr)
{
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    ESP_RETURN_ON_ERROR(dm9058_hash_filter_modify(emac, addr, true), TAG, "modify multicast table failed");
    return ESP_OK;
}

static esp_err_t emac_dm9058_rm_mac_filter(esp_eth_mac_t *mac, uint8_t *addr)
{
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    ESP_RETURN_ON_ERROR(dm9058_hash_filter_modify(emac, addr, false), TAG, "modify multicast table failed");
    return ESP_OK;
}

/**
 * @brief software reset dm9058 internal register
 */
static esp_err_t dm9058_reset(emac_dm9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    /* power on phy */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_GPR, 0x00), err, TAG, "write GPR failed");
    /* mac and phy register won't be accessible within at least 1ms */
    vTaskDelay(pdMS_TO_TICKS(10));
    /* software reset */
    uint8_t ncr = NCR_RST;
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_NCR, ncr), err, TAG, "write NCR failed");
    uint32_t to = 0;
    for (to = 0; to < emac->sw_reset_timeout_ms / 10; to++) {
        ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_NCR, &ncr), err, TAG, "read NCR failed");
        if (!(ncr & NCR_RST)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_GOTO_ON_FALSE(to < emac->sw_reset_timeout_ms / 10, ESP_ERR_TIMEOUT, err, TAG, "reset timeout");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief verify dm9058 chip ID
 */
static esp_err_t dm9058_verify_id(emac_dm9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    uint8_t id[2];
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_VIDL, &id[0]), err, TAG, "read VIDL failed");
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_VIDH, &id[1]), err, TAG, "read VIDH failed");
    ESP_GOTO_ON_FALSE(0x0A == id[1] && 0x46 == id[0], ESP_ERR_INVALID_VERSION, err, TAG, "wrong Vendor ID");
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_PIDL, &id[0]), err, TAG, "read PIDL failed");
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_PIDH, &id[1]), err, TAG, "read PIDH failed");
    ESP_GOTO_ON_FALSE(0x90 == id[1] && 0x58 == id[0], ESP_ERR_INVALID_VERSION, err, TAG, "wrong Product ID");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief default setup for dm9058 internal registers
 */
static esp_err_t dm9058_setup_default(emac_dm9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    /* disable wakeup */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_NCR, 0x00), err, TAG, "write NCR failed");
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_WCR, 0x00), err, TAG, "write WCR failed");
    /* stop transmitting, enable appending pad, crc for packets */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_TCR, 0x00), err, TAG, "write TCR failed");
    /* DM9058 specific: set memory boundary to byte mode */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_MBNDRY, MBNDRY_BYTE), err, TAG, "write MBNDRY failed");
    /* DM9058 specific: configure pause packet control */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PPCR, PPCR_PAUSE_COUNT), err, TAG, "write PPCR failed");
    /* DM9058 specific: configure LED mode */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_LMCR, LMCR_MODE1), err, TAG, "write LMCR failed");
    /* Interrupt pin config: push-pull output, active high (match GPIO_INTR_POSEDGE) */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_INTR, 0x00), err, TAG, "write INTR failed");
    /* Before enabling the RCR feature, make sure to set IMR_PAR in IMR */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_IMR, IMR_PAR), err, TAG, "write IMR failed");
    /* stop receiving, no promiscuous mode, no runt packet(size < 64bytes) */
    /* discard long packet(size > 1522bytes) and crc error packet, enable watchdog */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_RCR, RCR_DIS_LONG | RCR_DIS_CRC), err, TAG, "write RCR failed");
    /* retry late collision packet, at most two transmit command can be issued before transmit complete */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_TCR2, TCR2_RLCP | TCR2_DTU), err, TAG, "write TCR2 failed");
    /* enable auto transmit */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_ATCR, ATCR_AUTO_TX), err, TAG, "write ATCR failed");
    /* do not generate checksum for UDP, TCP and IPv4 packets */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_CSCR, 0x00), err, TAG, "write CSCR failed");
    /* disable check sum for receive packets */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_RCSSR, 0x00), err, TAG, "write RCSSR failed");
    /* no length limitation for rx packets */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_RLENCR, 0x00), err, TAG, "write RLENCR failed");
    /* clear network status: wakeup event, tx complete */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_NSR, NSR_WAKEST), err, TAG, "write NSR failed");
    /* Enable packet length filter of broadcast packet */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_BCASTCR, 0xc0), err, TAG, "write BCASTCR failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t dm9058_enable_flow_ctrl(emac_dm9058_t *emac, bool enable)
{
    esp_err_t ret = ESP_OK;
    if (enable) {
        /* send jam pattern (duration time = 1.15ms) when rx free space < 3k bytes */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_BPTR, 0x3F), err, TAG, "write BPTR failed");
        /* flow control: high water threshold = 3k bytes, low water threshold = 8k bytes */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_FCTR, 0x38), err, TAG, "write FCTR failed");
        /* enable flow control */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_FCR, FCR_FLOW_ENABLE), err, TAG, "write FCR failed");
    } else {
        /* disable flow control */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_FCR, 0), err, TAG, "write FCR failed");
    }
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief PTP Functions for DM9058
 */

static esp_err_t dm9058_ptp_enable(emac_dm9058_t *emac, bool enable)
{
    esp_err_t ret = ESP_OK;
    uint8_t ptp_offset_addr, ptp_checksum_addr;

    if (enable) {
        /* PTP restart sequence */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_CR, PTP_CR_RESTART), err, TAG, "write PTP_CR failed");
        vTaskDelay(pdMS_TO_TICKS(1));
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_CR, 0x00), err, TAG, "write PTP_CR failed");

        /* Enable PTP functionality */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_ENR, PTP_ENR_ENABLE), err, TAG, "write PTP_ENR failed");

        /* Disable TX timestamp capture initially */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_TCR, 0x00), err, TAG, "write TCR failed");

        /* Master/Slave Mode & 1588 Version Register: RX_EN=0x10 | multicast=0x02 */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_RXCR, PTP_RXCR_ENABLE | PTP_RXCR_MCAST), err, TAG, "write PTP_RXCR failed");

        /* TX One Step disabled */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_ONESTEP, 0x00), err, TAG, "write PTP_ONESTEP failed");

        /* Configure 1-step sync packet offsets based on network transport type */
        switch (emac->ptp_transport) {
        case DM9058_PTP_TRANSPORT_UDP_IPV4:
            ptp_offset_addr = 0x4E;
            ptp_checksum_addr = 0x3C;
            break;
        case DM9058_PTP_TRANSPORT_UDP_IPV6:
            ptp_offset_addr = 0x62;
            ptp_checksum_addr = 0x50;
            break;
        case DM9058_PTP_TRANSPORT_IEEE_802_3:
        default:
            ptp_offset_addr = 0x32;
            ptp_checksum_addr = 0x20;
            break;
        }

        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_TSOFF, ptp_offset_addr), err, TAG, "write PTP_TSOFF failed");
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_CSOFF, ptp_checksum_addr), err, TAG, "write PTP_CSOFF failed");

        emac->ptp_enabled = true;
        emac->ptp_last_rate = 0;
        ESP_LOGI(TAG, "PTP enabled with transport type %d", emac->ptp_transport);
    } else {
        /* Disable PTP */
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_ENR, 0x00), err, TAG, "write PTP_ENR failed");
        emac->ptp_enabled = false;
        ESP_LOGI(TAG, "PTP disabled");
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t dm9058_ptp_get_time(emac_dm9058_t *emac, eth_dm9058_ptp_time_t *time)
{
    esp_err_t ret = ESP_OK;
    uint8_t time_buf[8];

    ESP_GOTO_ON_FALSE(emac->ptp_enabled, ESP_ERR_INVALID_STATE, err, TAG, "PTP not enabled");

    /* Setup register to read PTP clock time */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_ENR, PTP_ENR_RSTIDX | PTP_ENR_GETTIME), err, TAG, "write PTP_ENR failed");

    /* Read 8 bytes of time data */
    for (int i = 0; i < 8; i++) {
        ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_PTP_DATA, &time_buf[i]), err, TAG, "read PTP_DATA failed");
    }

    /* Convert to timestamp format (nanoseconds first, then seconds) */
    time->nanoseconds = (uint32_t)time_buf[0] | ((uint32_t)time_buf[1] << 8) |
                        ((uint32_t)time_buf[2] << 16) | ((uint32_t)time_buf[3] << 24);
    time->seconds = (uint32_t)time_buf[4] | ((uint32_t)time_buf[5] << 8) |
                    ((uint32_t)time_buf[6] << 16) | ((uint32_t)time_buf[7] << 24);

    return ESP_OK;
err:
    return ret;
}

static esp_err_t dm9058_ptp_set_time(emac_dm9058_t *emac, const eth_dm9058_ptp_time_t *time)
{
    esp_err_t ret = ESP_OK;
    uint8_t time_buf[8];

    ESP_GOTO_ON_FALSE(emac->ptp_enabled, ESP_ERR_INVALID_STATE, err, TAG, "PTP not enabled");

    /* Convert timestamp to byte array (nanoseconds first, then seconds) */
    time_buf[0] = (uint8_t)(time->nanoseconds & 0xFF);
    time_buf[1] = (uint8_t)((time->nanoseconds >> 8) & 0xFF);
    time_buf[2] = (uint8_t)((time->nanoseconds >> 16) & 0xFF);
    time_buf[3] = (uint8_t)((time->nanoseconds >> 24) & 0xFF);
    time_buf[4] = (uint8_t)(time->seconds & 0xFF);
    time_buf[5] = (uint8_t)((time->seconds >> 8) & 0xFF);
    time_buf[6] = (uint8_t)((time->seconds >> 16) & 0xFF);
    time_buf[7] = (uint8_t)((time->seconds >> 24) & 0xFF);

    /* PTP restart sequence */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_CR, PTP_CR_RESTART), err, TAG, "write PTP_CR failed");
    esp_rom_delay_us(2);
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_CR, 0x00), err, TAG, "write PTP_CR failed");
    emac->ptp_last_rate = 0;

    /* Reset index */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_ENR, PTP_ENR_RSTIDX), err, TAG, "write PTP_ENR failed");

    /* Write 8 bytes of time data */
    for (int i = 0; i < 8; i++) {
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_DATA, time_buf[i]), err, TAG, "write PTP_DATA failed");
    }

    /* Apply time setting: Write PTP Clock Time | PTP Enable */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_ENR, PTP_ENR_SETTIME | PTP_ENR_ENABLE), err, TAG, "write PTP_ENR failed");

    ESP_LOGI(TAG, "PTP time set: %"PRIu32" sec %"PRIu32" ns", time->seconds, time->nanoseconds);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t dm9058_ptp_adj_freq(emac_dm9058_t *emac, int32_t adj_ppb)
{
    esp_err_t ret = ESP_OK;
    uint8_t rate_bytes[4];
    uint8_t control_value;

    ESP_GOTO_ON_FALSE(emac->ptp_enabled, ESP_ERR_INVALID_STATE, err, TAG, "PTP not enabled");

    /* Calculate adjustment value */
    int64_t signed_addend = (int64_t)adj_ppb * (int64_t)PTP_ADJ_FREQ_BASE_ADDEND_Q16 / 65536;
    int64_t delta_rate = signed_addend - emac->ptp_last_rate;

    uint32_t adjust_value;
    if (delta_rate < 0) {
        control_value = PTP_ADJUST_SLOWER_CTRL;
        adjust_value = (uint32_t)(-delta_rate);
    } else {
        control_value = PTP_ADJUST_FASTER_CTRL;
        adjust_value = (uint32_t)delta_rate;
    }

    if (adjust_value > PTP_ADJ_MAX) {
        ESP_LOGW(TAG, "PTP adjustment value 0x%08"PRIx32" exceeds maximum", adjust_value);
        adjust_value = PTP_ADJ_MAX;
    }

    /* Convert to byte array (little-endian) */
    rate_bytes[0] = (uint8_t)(adjust_value & 0xFF);
    rate_bytes[1] = (uint8_t)((adjust_value >> 8) & 0xFF);
    rate_bytes[2] = (uint8_t)((adjust_value >> 16) & 0xFF);
    rate_bytes[3] = (uint8_t)((adjust_value >> 24) & 0xFF);

    /* Reset register index */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_ENR, PTP_ENR_RSTIDX), err, TAG, "write PTP_ENR failed");

    /* Write rate bytes */
    for (int i = 0; i < 4; i++) {
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_DATA, rate_bytes[i]), err, TAG, "write PTP_DATA failed");
    }

    /* Apply adjustment */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_ENR, control_value), err, TAG, "write PTP_ENR failed");

    emac->ptp_last_rate = signed_addend;
    ESP_LOGD(TAG, "PTP freq adjusted by %"PRId32" ppb", adj_ppb);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t dm9058_ptp_adj_time(emac_dm9058_t *emac, const eth_dm9058_ptp_time_t *offset)
{
    esp_err_t ret = ESP_OK;
    eth_dm9058_ptp_time_t current_time, new_time;

    ESP_GOTO_ON_FALSE(emac->ptp_enabled, ESP_ERR_INVALID_STATE, err, TAG, "PTP not enabled");

    /* Get current time */
    ESP_GOTO_ON_ERROR(dm9058_ptp_get_time(emac, &current_time), err, TAG, "get PTP time failed");

    /* Calculate new time by adding offset */
    int64_t total_ns = (int64_t)current_time.nanoseconds + (int64_t)offset->nanoseconds;
    int64_t total_sec = (int64_t)current_time.seconds + (int64_t)offset->seconds;

    /* Handle nanosecond overflow/underflow */
    while (total_ns >= 1000000000LL) {
        total_sec += 1;
        total_ns -= 1000000000LL;
    }
    while (total_ns < 0) {
        total_sec -= 1;
        total_ns += 1000000000LL;
    }

    new_time.seconds = (uint32_t)total_sec;
    new_time.nanoseconds = (uint32_t)total_ns;

    /* Set the new time */
    ESP_GOTO_ON_ERROR(dm9058_ptp_set_time(emac, &new_time), err, TAG, "set PTP time failed");

    ESP_LOGD(TAG, "PTP time adjusted by %"PRIu32"s %"PRIu32"ns", offset->seconds, offset->nanoseconds);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t dm9058_ptp_get_tx_timestamp(emac_dm9058_t *emac, eth_dm9058_ptp_time_t *time)
{
    esp_err_t ret = ESP_OK;
    uint8_t time_buf[8];

    ESP_GOTO_ON_FALSE(emac->ptp_enabled, ESP_ERR_INVALID_STATE, err, TAG, "PTP not enabled");

    /* Reset timestamp read index */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_ENR, PTP_ENR_RSTIDX), err, TAG, "write PTP_ENR failed");

    /* Set TX timestamp read mode */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_PTP_TXCR, PTP_TXCR_READTS), err, TAG, "write PTP_TXCR failed");

    /* Read 8 bytes of timestamp data */
    for (int i = 0; i < 8; i++) {
        ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_PTP_DATA, &time_buf[i]), err, TAG, "read PTP_DATA failed");
    }

    /* Convert to timestamp format */
    time->nanoseconds = (uint32_t)time_buf[0] | ((uint32_t)time_buf[1] << 8) |
                        ((uint32_t)time_buf[2] << 16) | ((uint32_t)time_buf[3] << 24);
    time->seconds = (uint32_t)time_buf[4] | ((uint32_t)time_buf[5] << 8) |
                    ((uint32_t)time_buf[6] << 16) | ((uint32_t)time_buf[7] << 24);

    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_dm9058_custom_ioctl(esp_eth_mac_t *mac, int cmd, void *data)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);

    switch (cmd) {
    case ETH_MAC_DM9058_CMD_PTP_ENABLE: {
        bool *enable = (bool *)data;
        ESP_GOTO_ON_FALSE(enable, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
        ret = dm9058_ptp_enable(emac, *enable);
        break;
    }
    case ETH_MAC_DM9058_CMD_S_PTP_TIME: {
        eth_dm9058_ptp_time_t *time = (eth_dm9058_ptp_time_t *)data;
        ESP_GOTO_ON_FALSE(time, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
        ret = dm9058_ptp_set_time(emac, time);
        break;
    }
    case ETH_MAC_DM9058_CMD_G_PTP_TIME: {
        eth_dm9058_ptp_time_t *time = (eth_dm9058_ptp_time_t *)data;
        ESP_GOTO_ON_FALSE(time, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
        ret = dm9058_ptp_get_time(emac, time);
        break;
    }
    case ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ: {
        int32_t *adj_ppb = (int32_t *)data;
        ESP_GOTO_ON_FALSE(adj_ppb, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
        ret = dm9058_ptp_adj_freq(emac, *adj_ppb);
        break;
    }
    case ETH_MAC_DM9058_CMD_ADJ_PTP_TIME: {
        eth_dm9058_ptp_time_t *offset = (eth_dm9058_ptp_time_t *)data;
        ESP_GOTO_ON_FALSE(offset, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
        ret = dm9058_ptp_adj_time(emac, offset);
        break;
    }
    case ETH_MAC_DM9058_CMD_G_TX_TIMESTAMP: {
        eth_dm9058_ptp_time_t *time = (eth_dm9058_ptp_time_t *)data;
        ESP_GOTO_ON_FALSE(time, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
        ret = dm9058_ptp_get_tx_timestamp(emac, time);
        break;
    }
    case ETH_MAC_DM9058_CMD_S_PTP_TRANSPORT: {
        eth_dm9058_ptp_transport_t *transport = (eth_dm9058_ptp_transport_t *)data;
        ESP_GOTO_ON_FALSE(transport, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
        emac->ptp_transport = *transport;
        break;
    }
    default:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }

err:
    return ret;
}

/**
 * @brief start dm9058: enable interrupt and start receive
 */
static esp_err_t emac_dm9058_start(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    /* reset tx and rx memory pointer */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_MPCR, MPCR_RST_RX | MPCR_RST_TX), err, TAG, "write MPCR failed");
    /* clear interrupt status */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_ISR, ISR_CLR_STATUS), err, TAG, "write ISR failed");
    /* enable only Rx related interrupts as others are processed synchronously */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_IMR, IMR_PAR | IMR_PRI), err, TAG, "write IMR failed");
    /* enable rx */
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_RCR, &rcr), err, TAG, "read RCR failed");
    rcr |= RCR_RXEN;
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_RCR, rcr), err, TAG, "write RCR failed");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief stop dm9058: disable interrupt and stop receive
 */
static esp_err_t emac_dm9058_stop(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    /* IMR_PAR should not be cleared unless Wake-on-LAN (WOL) functionality is required */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_IMR, IMR_PAR), err, TAG, "write IMR failed");
    /* disable rx */
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_RCR, &rcr), err, TAG, "read RCR failed");
    rcr &= ~RCR_RXEN;
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_RCR, rcr), err, TAG, "write RCR failed");
    return ESP_OK;
err:
    return ret;
}

IRAM_ATTR static void dm9058_isr_handler(void *arg)
{
    emac_dm9058_t *emac = (emac_dm9058_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    /* notify dm9058 task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

static void dm9058_poll_timer(void *arg)
{
    emac_dm9058_t *emac = (emac_dm9058_t *)arg;
    xTaskNotifyGive(emac->rx_task_hdl);
}

static esp_err_t emac_dm9058_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(eth, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac's mediator to null");
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    emac->eth = eth;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_dm9058_phy_access_compl(emac_dm9058_t *emac, uint32_t timeout_us)
{
    uint8_t epcr = 0;
    ESP_RETURN_ON_ERROR(dm9058_register_read(emac, DM9058_EPCR, &epcr), TAG, "read EPCR failed");
    uint32_t to = 0;
    if (epcr & EPCR_ERRE) {
        do {
            esp_rom_delay_us(100);
            ESP_RETURN_ON_ERROR(dm9058_register_read(emac, DM9058_EPCR, &epcr), TAG, "read EPCR failed");
            to += 100;
        } while ((epcr & EPCR_ERRE) && to < timeout_us);
        ESP_RETURN_ON_FALSE(!(epcr & EPCR_ERRE), ESP_ERR_TIMEOUT, TAG, "wait for PHY/EEPROM access completion timeouted");
    }
    return ESP_OK;
}

static esp_err_t emac_dm9058_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);

    ESP_RETURN_ON_FALSE(dm9058_mutex_lock(emac), ESP_ERR_TIMEOUT, TAG, "multiple register access mutex timeout");
    ESP_GOTO_ON_ERROR(emac_dm9058_phy_access_compl(emac, DM9058_PHY_OPERATION_TIMEOUT_US), err, TAG, "PHY is busy");
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_EPAR, (uint8_t)(((phy_addr << 6) & 0xFF) | phy_reg)), err, TAG, "write EPAR failed");
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_EPDRL, (uint8_t)(reg_value & 0xFF)), err, TAG, "write EPDRL failed");
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_EPDRH, (uint8_t)((reg_value >> 8) & 0xFF)), err, TAG, "write EPDRH failed");
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_EPCR, EPCR_EPOS | EPCR_ERPRW), err, TAG, "write EPCR failed");
    ESP_GOTO_ON_ERROR(emac_dm9058_phy_access_compl(emac, DM9058_PHY_OPERATION_TIMEOUT_US), err, TAG, "PHY access completion check failed");
err:
    dm9058_mutex_unlock(emac);
    return ret;
}

static esp_err_t emac_dm9058_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(reg_value, ESP_ERR_INVALID_ARG, TAG, "can't set reg_value to null");
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);

    ESP_RETURN_ON_FALSE(dm9058_mutex_lock(emac), ESP_ERR_TIMEOUT, TAG, "multiple register access mutex timeout");
    ESP_GOTO_ON_ERROR(emac_dm9058_phy_access_compl(emac, DM9058_PHY_OPERATION_TIMEOUT_US), err, TAG, "PHY is busy");
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_EPAR, (uint8_t)(((phy_addr << 6) & 0xFF) | phy_reg)), err, TAG, "write EPAR failed");
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_EPCR, EPCR_EPOS | EPCR_ERPRR), err, TAG, "write EPCR failed");
    ESP_GOTO_ON_ERROR(emac_dm9058_phy_access_compl(emac, DM9058_PHY_OPERATION_TIMEOUT_US), err, TAG, "PHY access completion check failed");
    uint8_t value_h = 0;
    uint8_t value_l = 0;
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_EPDRH, &value_h), err, TAG, "read EPDRH failed");
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_EPDRL, &value_l), err, TAG, "read EPDRL failed");
    *reg_value = (value_h << 8) | value_l;
err:
    dm9058_mutex_unlock(emac);
    return ret;
}

static esp_err_t emac_dm9058_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac addr to null");
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    memcpy(emac->addr, addr, 6);
    ESP_GOTO_ON_ERROR(dm9058_set_mac_addr(emac), err, TAG, "set mac address failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_dm9058_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac addr to null");
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    memcpy(addr, emac->addr, 6);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_dm9058_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    switch (link) {
    case ETH_LINK_UP:
        ESP_GOTO_ON_ERROR(mac->start(mac), err, TAG, "dm9058 start failed");
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_start_periodic(emac->poll_timer, emac->poll_period_ms * 1000),
                              err, TAG, "start poll timer failed");
        }
        break;
    case ETH_LINK_DOWN:
        ESP_GOTO_ON_ERROR(mac->stop(mac), err, TAG, "dm9058 stop failed");
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_stop(emac->poll_timer),
                              err, TAG, "stop poll timer failed");
        }
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown link status");
        break;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_dm9058_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
    esp_err_t ret = ESP_OK;
    switch (speed) {
    case ETH_SPEED_10M:
        ESP_LOGD(TAG, "working in 10Mbps");
        break;
    case ETH_SPEED_100M:
        ESP_LOGD(TAG, "working in 100Mbps");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown speed");
        break;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_dm9058_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
{
    esp_err_t ret = ESP_OK;
    switch (duplex) {
    case ETH_DUPLEX_HALF:
        ESP_LOGD(TAG, "working in half duplex");
        break;
    case ETH_DUPLEX_FULL:
        ESP_LOGD(TAG, "working in full duplex");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown duplex");
        break;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_dm9058_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_RCR, &rcr), err, TAG, "read RCR failed");
    if (enable) {
        rcr |= RCR_PRMSC;
    } else {
        rcr &= ~RCR_PRMSC;
    }
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_RCR, rcr), err, TAG, "write RCR failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_dm9058_set_all_multicast(esp_eth_mac_t *mac, bool enable)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_RCR, &rcr), err, TAG, "read RCR failed");
    if (enable) {
        rcr |= RCR_ALL_MCAST;
    } else {
        rcr &= ~RCR_ALL_MCAST;
    }
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_RCR, rcr), err, TAG, "write RCR failed");
err:
    return ret;
}

static esp_err_t emac_dm9058_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)
{
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    emac->flow_ctrl_enabled = enable;
    return ESP_OK;
}

static esp_err_t emac_dm9058_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability)
{
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    if (emac->flow_ctrl_enabled && ability) {
        dm9058_enable_flow_ctrl(emac, true);
    } else {
        dm9058_enable_flow_ctrl(emac, false);
        ESP_LOGD(TAG, "Flow control not enabled for the link");
    }
    return ESP_OK;
}

static esp_err_t emac_dm9058_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);

    ESP_GOTO_ON_FALSE(length <= ETH_MAX_PACKET_SIZE, ESP_ERR_INVALID_ARG, err,
                      TAG, "frame size is too big (actual %" PRIu32 ", maximum %d)", length, ETH_MAX_PACKET_SIZE);

    uint8_t reg_nsr = 0;
    int64_t wait_time = esp_timer_get_time();
    do {
        ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
        reg_nsr &= (NSR_TX2END | NSR_TX1END);
    } while ((!reg_nsr) && ((esp_timer_get_time() - wait_time) < 100));

    if (!reg_nsr) {
        ESP_LOGE(TAG, "last transmit still in progress, cannot send.");
        return ESP_ERR_INVALID_STATE;
    }

    if (reg_nsr == (NSR_TX2END | NSR_TX1END)) {
        ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_MPCR, MPCR_RST_TX), err, TAG, "write MPCR failed");
    }

    /* set tx length */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_TXPLL, length & 0xFF), err, TAG, "write TXPLL failed");
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_TXPLH, (length >> 8) & 0xFF), err, TAG, "write TXPLH failed");
    /* copy data to tx memory */
    ESP_GOTO_ON_ERROR(dm9058_memory_write(emac, buf, length), err, TAG, "write memory failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t dm9058_skip_recv_frame(emac_dm9058_t *emac, uint16_t rx_length)
{
    esp_err_t ret = ESP_OK;
    uint8_t mrrh, mrrl;
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_MRRH, &mrrh), err, TAG, "read MRRH failed");
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_MRRL, &mrrl), err, TAG, "read MRRL failed");
    uint16_t addr = mrrh << 8 | mrrl;
    addr += rx_length;
    if (addr > DM9058_RX_MEM_MAX_SIZE) {
        addr = addr - DM9058_RX_MEM_MAX_SIZE + DM9058_RX_MEM_START_ADDR;
    }
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_MRRL, addr & 0xFF), err, TAG, "write MRRL failed");
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_MRRH, addr >> 8), err, TAG, "write MRRH failed");
err:
    return ret;
}

static esp_err_t dm9058_flush_recv_queue(emac_dm9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(emac->parent.stop(&emac->parent), err, TAG, "stop dm9058 failed");
    /* reset rx fifo pointer */
    ESP_GOTO_ON_ERROR(dm9058_register_write(emac, DM9058_MPCR, MPCR_RST_RX), err, TAG, "write MPCR failed");
    esp_rom_delay_us(10);
    ESP_GOTO_ON_ERROR(emac->parent.start(&emac->parent), err, TAG, "start dm9058 failed");
err:
    return ret;
}

static esp_err_t dm9058_frame_to_rx_buffer(emac_dm9058_t *emac, uint16_t *size)
{
    esp_err_t ret = ESP_OK;
    uint8_t rxbyte = 0;
    __attribute__((aligned(4))) dm9058_rx_header_t header;
    bool try_again = false;

    do {
        *size = 0;
        uint8_t reg_nsr = 0;
        ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
        if (reg_nsr & NSR_RXRDY) {
            /* dummy read, get the most updated data */
            ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
            ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_MRCMDX1, &rxbyte), err, TAG, "read MRCMDX failed");
            if (0x01 != rxbyte) {
                ESP_GOTO_ON_ERROR(dm9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "unexpected rx flag (0x%" PRIx8 "), reset rx fifo pointer", rxbyte);
            }
            ESP_GOTO_ON_ERROR(dm9058_memory_read(emac, (uint8_t *)&header, sizeof(header)), err, TAG, "read rx header failed");
            uint16_t rx_len = header.length_low + (header.length_high << 8);
            /* store the whole frame to preallocated memory */
            if (rx_len <= ETH_MAX_PACKET_SIZE) {
                ESP_GOTO_ON_ERROR(dm9058_memory_read(emac, emac->rx_buffer, rx_len), err, TAG, "read rx data failed");
            } else {
                ESP_GOTO_ON_ERROR(dm9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "invalid frame length, reset rx fifo pointer");
            }
            if (header.status & (RSR_RF | RSR_RWTO | RSR_CE | RSR_FOE)) {
                dm9058_skip_recv_frame(emac, rx_len);
                ESP_LOGE(TAG, "receive status error: %" PRIx8 "H", header.status);
                try_again = true;
            } else {
                *size = rx_len;
            }
        }
    } while (try_again);
err:
    return ret;
}

static esp_err_t emac_dm9058_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    uint16_t byte_count = 0;
    emac->packets_remain = false;

    ESP_GOTO_ON_ERROR(dm9058_frame_to_rx_buffer(emac, &byte_count), err, TAG, "moving data to internal rx_buffer failed");
    if (!byte_count) {
        goto err;
    }
    uint16_t rx_len = byte_count - ETH_CRC_LEN;
    if (buf == emac->rx_buffer) {
        *length = rx_len;
    } else {
        ESP_GOTO_ON_FALSE(buf, ESP_ERR_INVALID_ARG, err, TAG, "buffer can't be NULL");
        uint16_t copy_len = rx_len > *length ? *length : rx_len;
        memcpy(buf, emac->rx_buffer, copy_len);
        *length = copy_len;
    }

    uint8_t reg_nsr = 0;
    ESP_GOTO_ON_ERROR(dm9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
    emac->packets_remain = (reg_nsr & NSR_RXRDY);
    return ESP_OK;
err:
    *length = 0;
    return ret;
}

static esp_err_t emac_dm9058_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    if (emac->int_gpio_num >= 0) {
        gpio_func_sel(emac->int_gpio_num, PIN_FUNC_GPIO);
        gpio_input_enable(emac->int_gpio_num);
        gpio_pulldown_en(emac->int_gpio_num);
        gpio_set_intr_type(emac->int_gpio_num, GPIO_INTR_POSEDGE);
        gpio_intr_enable(emac->int_gpio_num);
        gpio_isr_handler_add(emac->int_gpio_num, dm9058_isr_handler, emac);
    }
    ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL), err, TAG, "lowlevel init failed");
    /* reset dm9058 */
    ESP_GOTO_ON_ERROR(dm9058_reset(emac), err, TAG, "reset dm9058 failed");
    /* verify chip id */
    ESP_GOTO_ON_ERROR(dm9058_verify_id(emac), err, TAG, "verify chip ID failed");
    /* default setup of internal registers */
    ESP_GOTO_ON_ERROR(dm9058_setup_default(emac), err, TAG, "dm9058 default setup failed");
    /* clear multicast hash table */
    ESP_GOTO_ON_ERROR(dm9058_clear_multicast_table(emac), err, TAG, "clear multicast table failed");
    /* get emac address from eeprom */
    ESP_GOTO_ON_ERROR(dm9058_get_mac_addr(emac), err, TAG, "fetch ethernet mac address failed");
    return ESP_OK;
err:
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
}

static esp_err_t emac_dm9058_deinit(esp_eth_mac_t *mac)
{
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    mac->stop(mac);
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
    }
    if (emac->poll_timer && esp_timer_is_active(emac->poll_timer)) {
        esp_timer_stop(emac->poll_timer);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ESP_OK;
}

static void emac_dm9058_task(void *arg)
{
    emac_dm9058_t *emac = (emac_dm9058_t *)arg;
    uint8_t status = 0;
    while (1) {
        if (emac->int_gpio_num >= 0) {
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 &&
                    gpio_get_level(emac->int_gpio_num) == 0) {
                continue;
            }
        } else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        /* clear interrupt status */
        dm9058_register_read(emac, DM9058_ISR, &status);
        dm9058_register_write(emac, DM9058_ISR, status);
        /* packet received */
        if (status & ISR_PR) {
            do {
                uint32_t buf_len;
                if (emac->parent.receive(&emac->parent, emac->rx_buffer, &buf_len) == ESP_OK) {
                    if (buf_len > 0) {
                        uint8_t *buffer = malloc(buf_len);
                        if (buffer == NULL) {
                            ESP_LOGE(TAG, "no mem for receive buffer");
                        } else {
                            memcpy(buffer, emac->rx_buffer, buf_len);
                            ESP_LOGD(TAG, "receive len=%" PRIu32, buf_len);
                            emac->eth->stack_input(emac->eth, buffer, buf_len);
                        }
                    }
                } else {
                    ESP_LOGE(TAG, "frame read from module failed");
                }
            } while (emac->packets_remain);
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t emac_dm9058_del(esp_eth_mac_t *mac)
{
    emac_dm9058_t *emac = __containerof(mac, emac_dm9058_t, parent);
    if (emac->poll_timer) {
        esp_timer_delete(emac->poll_timer);
    }
    vTaskDelete(emac->rx_task_hdl);
    emac->spi.deinit(emac->spi.ctx);
    vSemaphoreDelete(emac->multi_reg_axs_mutex);
    heap_caps_free(emac->rx_buffer);
    free(emac);
    return ESP_OK;
}

esp_eth_mac_t *esp_eth_mac_new_dm9058(const eth_dm9058_config_t *dm9058_config, const eth_mac_config_t *mac_config)
{
    esp_eth_mac_t *ret = NULL;
    emac_dm9058_t *emac = NULL;
    ESP_GOTO_ON_FALSE(dm9058_config, NULL, err, TAG, "can't set dm9058 specific config to null");
    ESP_GOTO_ON_FALSE(mac_config, NULL, err, TAG, "can't set mac config to null");
    ESP_GOTO_ON_FALSE((dm9058_config->int_gpio_num >= 0) != (dm9058_config->poll_period_ms > 0), NULL, err, TAG, "invalid configuration argument combination");
    emac = calloc(1, sizeof(emac_dm9058_t));
    ESP_GOTO_ON_FALSE(emac, NULL, err, TAG, "calloc emac failed");
    /* bind methods and attributes */
    emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->int_gpio_num = dm9058_config->int_gpio_num;
    emac->poll_period_ms = dm9058_config->poll_period_ms;
    emac->parent.set_mediator = emac_dm9058_set_mediator;
    emac->parent.init = emac_dm9058_init;
    emac->parent.deinit = emac_dm9058_deinit;
    emac->parent.start = emac_dm9058_start;
    emac->parent.stop = emac_dm9058_stop;
    emac->parent.del = emac_dm9058_del;
    emac->parent.write_phy_reg = emac_dm9058_write_phy_reg;
    emac->parent.read_phy_reg = emac_dm9058_read_phy_reg;
    emac->parent.set_addr = emac_dm9058_set_addr;
    emac->parent.get_addr = emac_dm9058_get_addr;
    emac->parent.set_speed = emac_dm9058_set_speed;
    emac->parent.set_duplex = emac_dm9058_set_duplex;
    emac->parent.set_link = emac_dm9058_set_link;
    emac->parent.set_promiscuous = emac_dm9058_set_promiscuous;
    emac->parent.set_all_multicast = emac_dm9058_set_all_multicast;
    emac->parent.set_peer_pause_ability = emac_dm9058_set_peer_pause_ability;
    emac->parent.enable_flow_ctrl = emac_dm9058_enable_flow_ctrl;
    emac->parent.transmit = emac_dm9058_transmit;
    emac->parent.receive = emac_dm9058_receive;
    emac->parent.add_mac_filter = emac_dm9058_add_mac_filter;
    emac->parent.rm_mac_filter = emac_dm9058_rm_mac_filter;
    emac->parent.custom_ioctl = emac_dm9058_custom_ioctl;
    /* Initialize PTP state */
    emac->ptp_enabled = false;
    emac->ptp_last_rate = 0;
    emac->ptp_transport = DM9058_PTP_TRANSPORT_IEEE_802_3;

    if (dm9058_config->custom_spi_driver.init != NULL && dm9058_config->custom_spi_driver.deinit != NULL
            && dm9058_config->custom_spi_driver.read != NULL && dm9058_config->custom_spi_driver.write != NULL) {
        ESP_LOGD(TAG, "Using user's custom SPI Driver");
        emac->spi.init = dm9058_config->custom_spi_driver.init;
        emac->spi.deinit = dm9058_config->custom_spi_driver.deinit;
        emac->spi.read = dm9058_config->custom_spi_driver.read;
        emac->spi.write = dm9058_config->custom_spi_driver.write;
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(dm9058_config->custom_spi_driver.config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    } else {
        ESP_LOGD(TAG, "Using default SPI Driver");
        emac->spi.init = dm9058_spi_init;
        emac->spi.deinit = dm9058_spi_deinit;
        emac->spi.read = dm9058_spi_read;
        emac->spi.write = dm9058_spi_write;
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(dm9058_config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    }

    /* create mutex for accessing multiple registers in atomic manner */
    emac->multi_reg_axs_mutex = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(emac->multi_reg_axs_mutex, NULL, err, TAG, "create multi registers access mutex failed");

    /* create dm9058 task */
    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = esp_cpu_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(emac_dm9058_task, "dm9058_tsk", mac_config->rx_task_stack_size, emac,
                                                   mac_config->rx_task_prio, &emac->rx_task_hdl, core_num);
    ESP_GOTO_ON_FALSE(xReturned == pdPASS, NULL, err, TAG, "create dm9058 task failed");

    emac->rx_buffer = heap_caps_malloc(ETH_MAX_PACKET_SIZE, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(emac->rx_buffer, NULL, err, TAG, "RX buffer allocation failed");

    if (emac->int_gpio_num < 0) {
        const esp_timer_create_args_t poll_timer_args = {
            .callback = dm9058_poll_timer,
            .name = "emac_spi_poll_timer",
            .arg = emac,
            .skip_unhandled_events = true
        };
        ESP_GOTO_ON_FALSE(esp_timer_create(&poll_timer_args, &emac->poll_timer) == ESP_OK, NULL, err, TAG, "create poll timer failed");
    }

    return &(emac->parent);

err:
    if (emac) {
        if (emac->poll_timer) {
            esp_timer_delete(emac->poll_timer);
        }
        if (emac->rx_task_hdl) {
            vTaskDelete(emac->rx_task_hdl);
        }
        if (emac->spi.ctx) {
            emac->spi.deinit(emac->spi.ctx);
        }
        if (emac->multi_reg_axs_mutex) {
            vSemaphoreDelete(emac->multi_reg_axs_mutex);
        }
        heap_caps_free(emac->rx_buffer);
        free(emac);
    }
    return ret;
}
