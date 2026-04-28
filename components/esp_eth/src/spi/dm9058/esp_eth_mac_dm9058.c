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
#include "esp_eth_ptp_dm9058.h"

static const char *TAG = "dm9058.mac";
static const char *PTP_TAG = "dm9058.ptp";

typedef bool (*dm9058_ts_target_cb_t)(esp_eth_mediator_t *eth, void *user_args);

#define DM9058_SPI_LOCK_TIMEOUT_MS      (50)
#define DM9058_PHY_OPERATION_TIMEOUT_US (1000)
#define DM9058_MULTI_REG_AXS_TIMEOUT_MS (50)
#define DM9058_RX_MEM_START_ADDR        (3072)
#define DM9058_RX_MEM_MAX_SIZE          (16384)
#define DM9058_RX_HDR_SIZE              (4)
#define DM9058_RSR_RXTS_LEN             (1 << 2)
#define DM9058_RSR_RXTS_EN              (1 << 5)

#define DM9058_ETH_TYPE_IPV4            0x0800
#define DM9058_ETH_TYPE_PTP             0x88F7
#define DM9058_ETH_HEADER_LEN           14
#define DM9058_IPPROTO_UDP              17
#define DM9058_PTP_EVENT_PORT           319
#define DM9058_PTP_GENERAL_PORT         320

#define DM9058_HASH_FILTER_TABLE_SIZE   (64)

typedef struct {
    uint8_t flag;        // 0 = no frame, 1 = frame received, others = possible memory pointer error or tcpip_checksum_offload status flag if enabled
    uint8_t status;      // Events occurred between this and previous frame (the same format as RSR)
    uint8_t length_low;  // Low byte of received frame length
    uint8_t length_high; // High byte of received frame length
} DM9058_rx_header_t;

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
    dm9058_ts_target_cb_t ts_target_exceed_cb_from_isr;
	esp_eth_ptp_dm9058_time_t target_time;
    bool target_time_valid;
    esp_timer_handle_t ptp_timer;
    bool ptp_timer_started;
    esp_eth_ptp_dm9058_t ptp;
    bool ptp_auto_process;
    bool ptp_two_step_mode;
} esp32_DM9058_t;

typedef struct {
    bool is_ptp;
    uint8_t message_type;
} dm9058_ptp_packet_info_t;

static esp_err_t dm9058_parse_packet_info(const uint8_t *packet, size_t len, dm9058_ptp_packet_info_t *info)
{
    ESP_RETURN_ON_FALSE(packet != NULL && info != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid packet info args");
    memset(info, 0, sizeof(*info));

    ESP_RETURN_ON_FALSE(len >= DM9058_ETH_HEADER_LEN, ESP_ERR_INVALID_SIZE, TAG, "frame too short");
    uint16_t ethertype = ((uint16_t)packet[12] << 8) | packet[13];
    const uint8_t *ptp_hdr = NULL;

    if (ethertype == DM9058_ETH_TYPE_PTP) {
        ESP_RETURN_ON_FALSE(len >= DM9058_ETH_HEADER_LEN + 1, ESP_ERR_INVALID_SIZE, TAG, "ptp l2 frame too short");
        ptp_hdr = packet + DM9058_ETH_HEADER_LEN;
    } else if (ethertype == DM9058_ETH_TYPE_IPV4) {
        ESP_RETURN_ON_FALSE(len >= DM9058_ETH_HEADER_LEN + 20 + 8 + 1, ESP_ERR_INVALID_SIZE, TAG, "ipv4 frame too short");
        size_t ip_offset = DM9058_ETH_HEADER_LEN;
        uint8_t ihl = (packet[ip_offset] & 0x0F) * 4;
        ESP_RETURN_ON_FALSE(ihl >= 20, ESP_ERR_INVALID_RESPONSE, TAG, "invalid ipv4 ihl");
        ESP_RETURN_ON_FALSE(len >= DM9058_ETH_HEADER_LEN + ihl + 8 + 1, ESP_ERR_INVALID_SIZE, TAG, "udp payload too short");
        ESP_RETURN_ON_FALSE(packet[ip_offset + 9] == DM9058_IPPROTO_UDP, ESP_ERR_NOT_FOUND, TAG, "not udp ptp packet");

        size_t udp_offset = DM9058_ETH_HEADER_LEN + ihl;
        uint16_t sport = ((uint16_t)packet[udp_offset] << 8) | packet[udp_offset + 1];
        uint16_t dport = ((uint16_t)packet[udp_offset + 2] << 8) | packet[udp_offset + 3];
        bool ptp_port = (sport == DM9058_PTP_EVENT_PORT || sport == DM9058_PTP_GENERAL_PORT ||
                         dport == DM9058_PTP_EVENT_PORT || dport == DM9058_PTP_GENERAL_PORT);
        ESP_RETURN_ON_FALSE(ptp_port, ESP_ERR_NOT_FOUND, TAG, "not ptp udp packet");
        ptp_hdr = packet + udp_offset + 8;
    } else {
        return ESP_ERR_NOT_FOUND;
    }

    info->is_ptp = true;
    info->message_type = ptp_hdr[0] & 0x0F;
    return ESP_OK;
}

static void *DM9058_spi_init(const void *spi_config)
{
    void *ret = NULL;
    eth_dm9058_config_t *DM9058_config = (eth_dm9058_config_t *)spi_config;
    eth_spi_info_t *spi = calloc(1, sizeof(eth_spi_info_t));
    ESP_GOTO_ON_FALSE(spi, NULL, err, TAG, "no memory for SPI context data");

    /* SPI device init */
    spi_device_interface_config_t spi_devcfg;
    spi_devcfg = *(DM9058_config->spi_devcfg);
    if (DM9058_config->spi_devcfg->command_bits == 0 && DM9058_config->spi_devcfg->address_bits == 0) {
        /* configure default SPI frame format */
        spi_devcfg.command_bits = 1;
        spi_devcfg.address_bits = 7;
    } else {
        ESP_GOTO_ON_FALSE(DM9058_config->spi_devcfg->command_bits == 1 && DM9058_config->spi_devcfg->address_bits == 7,
                          NULL, err, TAG, "incorrect SPI frame format (command_bits/address_bits)");
    }
    ESP_GOTO_ON_FALSE(spi_bus_add_device(DM9058_config->spi_host_id, &spi_devcfg, &spi->hdl) == ESP_OK,
                      NULL, err, TAG, "adding device to SPI host #%i failed", DM9058_config->spi_host_id + 1);

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

static esp_err_t DM9058_spi_deinit(void *spi_ctx)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_bus_remove_device(spi->hdl);
    vSemaphoreDelete(spi->lock);

    free(spi);
    return ret;
}

static inline bool DM9058_spi_lock(eth_spi_info_t *spi)
{
    return xSemaphoreTake(spi->lock, pdMS_TO_TICKS(DM9058_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline bool DM9058_spi_unlock(eth_spi_info_t *spi)
{
    return xSemaphoreGive(spi->lock) == pdTRUE;
}

static bool dm9058_time_reached(const esp_eth_ptp_dm9058_time_t *now, const esp_eth_ptp_dm9058_time_t *target)
{
    if (now->seconds > target->seconds) {
        return true;
    }
    if (now->seconds < target->seconds) {
        return false;
    }
    return now->nanoseconds >= target->nanoseconds;
}

static void dm9058_ptp_timer_start(esp32_DM9058_t *emac)
{
    if (!emac->ptp_timer || emac->ptp_timer_started) {
        return;
    }
    if (emac->target_time_valid && emac->ts_target_exceed_cb_from_isr) {
        if (esp_timer_start_periodic(emac->ptp_timer, 1000000) == ESP_OK) { // 1 second
            emac->ptp_timer_started = true;
        }
    }
}

static void dm9058_ptp_timer_stop(esp32_DM9058_t *emac)
{
    if (emac->ptp_timer && emac->ptp_timer_started) {
        esp_timer_stop(emac->ptp_timer);
        emac->ptp_timer_started = false;
    }
}

static esp_err_t DM9058_spi_write(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_transaction_t trans = {
        .cmd = cmd,
        .addr = addr,
        .length = 8 * len,
        .tx_buffer = value
    };
    if (DM9058_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        DM9058_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    return ret;
}

static esp_err_t DM9058_spi_read(void *spi_ctx, uint32_t cmd, uint32_t addr, void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_transaction_t trans = {
        .flags = len <= 4 ? SPI_TRANS_USE_RXDATA : 0, // use direct reads for registers to prevent overwrites by 4-byte boundary writes
        .cmd = cmd,
        .addr = addr,
        .length = 8 * len,
        .rx_buffer = value
    };
    if (DM9058_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        DM9058_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    if ((trans.flags & SPI_TRANS_USE_RXDATA) && len <= 4) {
        memcpy(value, trans.rx_data, len);  // copy register values to output
    }
    return ret;
}

static inline bool DM9058_mutex_lock(esp32_DM9058_t *esp32)
{
    return xSemaphoreTake(esp32->multi_reg_axs_mutex, pdMS_TO_TICKS(DM9058_MULTI_REG_AXS_TIMEOUT_MS)) == pdTRUE;
}

static inline bool DM9058_mutex_unlock(esp32_DM9058_t *esp32)
{
    return xSemaphoreGive(esp32->multi_reg_axs_mutex) == pdTRUE;
}

static esp_err_t DM9058_register_write(esp32_DM9058_t *esp32, uint8_t reg_addr, uint8_t value);
static esp_err_t DM9058_register_read(esp32_DM9058_t *esp32, uint8_t reg_addr, uint8_t *value);

static esp_err_t DM9058_ptp_reg_write(void *io_ctx, uint8_t reg, uint8_t value)
{
    return DM9058_register_write((esp32_DM9058_t *)io_ctx, reg, value);
}

static esp_err_t DM9058_ptp_reg_read(void *io_ctx, uint8_t reg, uint8_t *value)
{
    return DM9058_register_read((esp32_DM9058_t *)io_ctx, reg, value);
}

static esp_err_t DM9058_ptp_reg_burst_write(void *io_ctx, uint8_t reg, const uint8_t *buffer, size_t len)
{
    esp32_DM9058_t *esp32 = (esp32_DM9058_t *)io_ctx;
    return esp32->spi.write(esp32->spi.ctx, DM9058_SPI_WR, reg, buffer, len);
}

static esp_err_t DM9058_ptp_reg_burst_read(void *io_ctx, uint8_t reg, uint8_t *buffer, size_t len)
{
    esp32_DM9058_t *esp32 = (esp32_DM9058_t *)io_ctx;
    return esp32->spi.read(esp32->spi.ctx, DM9058_SPI_RD, reg, buffer, len);
}

static bool DM9058_ptp_lock(void *io_ctx)
{
    return DM9058_mutex_lock((esp32_DM9058_t *)io_ctx);
}

static void DM9058_ptp_unlock(void *io_ctx)
{
    DM9058_mutex_unlock((esp32_DM9058_t *)io_ctx);
}

static void DM9058_ptp_delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

static void DM9058_ptp_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief write value to DM9058 internal register
 */
static esp_err_t DM9058_register_write(esp32_DM9058_t *esp32, uint8_t reg_addr, uint8_t value)
{
    return esp32->spi.write(esp32->spi.ctx, DM9058_SPI_WR, reg_addr, &value, 1);
}

/**
 * @brief read value from DM9058 internal register
 */
static esp_err_t DM9058_register_read(esp32_DM9058_t *esp32, uint8_t reg_addr, uint8_t *value)
{
    return esp32->spi.read(esp32->spi.ctx, DM9058_SPI_RD, reg_addr, value, 1);
}

/**
 * @brief write buffer to DM9058 internal memory
 */
static esp_err_t DM9058_memory_write(esp32_DM9058_t *esp32, uint8_t *buffer, uint32_t len)
{
    return esp32->spi.write(esp32->spi.ctx, DM9058_SPI_WR, DM9058_MWCMD, buffer, len);
}

/**
 * @brief read buffer from DM9058 internal memory
 */
static esp_err_t DM9058_memory_read(esp32_DM9058_t *esp32, uint8_t *buffer, uint32_t len)
{
    return esp32->spi.read(esp32->spi.ctx, DM9058_SPI_RD, DM9058_MRCMD, buffer, len);
}

/**
 * @brief read mac address from internal registers
 */
static esp_err_t DM9058_get_mac_addr(esp32_DM9058_t *esp32)
{
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < ETH_ADDR_LEN; i++) {
        ESP_GOTO_ON_ERROR(DM9058_register_read(esp32, DM9058_PAR + i, &esp32->addr[i]), err, TAG, "read PAR failed");
    }
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief set new mac address to internal registers
 */
static esp_err_t DM9058_set_mac_addr(esp32_DM9058_t *esp32)
{
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < ETH_ADDR_LEN; i++) {
        ESP_GOTO_ON_ERROR(DM9058_register_write(esp32, DM9058_PAR + i, esp32->addr[i]), err, TAG, "write PAR failed");
    }
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief clear multicast hash table
 */
static esp_err_t DM9058_clear_multicast_table(esp32_DM9058_t *esp32)
{
    esp_err_t ret = ESP_OK;
    /* Keep multicast hash table empty and use DM9058_BCASTCR to accept broadcast for in better performance */
    for (int i = 0; i < 8; i++) {
        ESP_GOTO_ON_ERROR(DM9058_register_write(esp32, DM9058_MAR + i, 0x00), err, TAG, "write MAR failed");
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t DM9058_hash_filter_modify(esp32_DM9058_t *esp32, uint8_t *addr, bool add)
{
    esp_err_t ret = ESP_OK;

    // calculate crc32 value of mac address
    uint32_t crc = ~esp_rom_crc32_le(0, addr, ETH_ADDR_LEN);

    uint8_t hash_value = crc & 0x3F;
    uint8_t hash_group = hash_value / 8;
    uint8_t hash_bit = hash_value % 8;

    uint8_t mar;
    ESP_GOTO_ON_ERROR(DM9058_register_read(esp32, (DM9058_MAR + hash_group), &mar), err, TAG, "read MAR failed");
    if (add) {
        // add address to hash table
        mar |= (1 << hash_bit);
        esp32->hash_filter_cnt[hash_value]++;
    } else {
        esp32->hash_filter_cnt[hash_value]--;
        if (esp32->hash_filter_cnt[hash_value] == 0) {
            // remove address from hash table
            mar &= ~(1 << hash_bit);
        }
    }
    ESP_GOTO_ON_ERROR(DM9058_register_write(esp32, (DM9058_MAR + hash_group), mar), err, TAG, "write MAR failed");

err:
    return ret;
}

static esp_err_t esp32_DM9058_add_mac_filter(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    ESP_RETURN_ON_ERROR(DM9058_hash_filter_modify(emac, addr, true), TAG, "modify multicast table failed");
    return ESP_OK;
}

static esp_err_t esp32_DM9058_rm_mac_filter(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    ESP_RETURN_ON_ERROR(DM9058_hash_filter_modify(emac, addr, false), TAG, "modify multicast table failed");
    return ESP_OK;
}

/**
 * @brief software reset DM9058 internal register
 */
static esp_err_t DM9058_reset(esp32_DM9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    /* power on phy */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_GPR, 0x00), err, TAG, "write GPR failed");
    /* mac and phy register won't be accessible within at least 1ms */
    vTaskDelay(pdMS_TO_TICKS(10));
    /* software reset */
    uint8_t ncr = NCR_RST;
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_NCR, ncr), err, TAG, "write NCR failed");
    uint32_t to = 0;
    for (to = 0; to < emac->sw_reset_timeout_ms / 10; to++) {
        ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_NCR, &ncr), err, TAG, "read NCR failed");
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
 * @brief verify DM9058 chip ID
 */
static esp_err_t DM9058_verify_id(esp32_DM9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    uint8_t id[2];
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_VIDL, &id[0]), err, TAG, "read VIDL failed");
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_VIDH, &id[1]), err, TAG, "read VIDH failed");
    ESP_GOTO_ON_FALSE(0x0A == id[1] && 0x46 == id[0], ESP_ERR_INVALID_VERSION, err, TAG, "wrong Vendor ID");
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_PIDL, &id[0]), err, TAG, "read PIDL failed");
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_PIDH, &id[1]), err, TAG, "read PIDH failed");
    ESP_GOTO_ON_FALSE(0x90 == id[1] && 0x58 == id[0], ESP_ERR_INVALID_VERSION, err, TAG, "wrong Product ID");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief default setup for DM9058 internal registers
 */
static esp_err_t DM9058_setup_default(esp32_DM9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    /* disable wakeup */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_NCR, 0x00), err, TAG, "write NCR failed");
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_WCR, 0x00), err, TAG, "write WCR failed");
    /* stop transmitting, enable appending pad, crc for packets */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_TCR, 0x00), err, TAG, "write TCR failed");
    /* Before enabling the RCR feature, make sure to set IMR_PAR in IMR */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_IMR, IMR_PAR), err, TAG, "write DM9058_IMR failed");
    /* stop receiving, no promiscuous mode, no runt packet(size < 64bytes) */
    /* discard long packet(size > 1522bytes) and crc error packet, enable watchdog */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_RCR, RCR_DIS_LONG | RCR_DIS_CRC), err, TAG, "write RCR failed");
    /* retry late collision packet, at most two transmit command can be issued before transmit complete */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_TCR2, TCR2_RLCP), err, TAG, "write TCR2 failed");
    /* enable auto transmit */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_ATCR, ATCR_AUTO_TX), err, TAG, "write ATCR failed");
    /* do not generate checksum for UDP, TCP and IPv4 packets */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_TCSCR, 0x00), err, TAG, "write TCSCR failed");
    /* disable check sum for receive packets */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_RCSCSR, 0x00), err, TAG, "write RCSCSR failed");
    /* interrupt pin config: push-pull output, active high */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_INTCR, 0x00), err, TAG, "write INTCR failed");
    /* Utilize DM9058_INTCKCR to enable edge-triggered interrupts and address level-triggered interrupt loss */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_INTCKCR, 0x83), err, TAG, "write INTCKCR failed");
    /* no length limitation for rx packets */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_RLENCR, 0x00), err, TAG, "write RLENCR failed");
    /* 3K-byte for TX and 13K-byte for RX */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_MEMSCR, 0x00), err, TAG, "write MEMSCR failed");
    /* clear network status: wakeup event, tx complete */
    /* Optimize transmission process using TX2END and TX1END */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_NSR, NSR_WAKEST), err, TAG, "write NSR failed");
    /* Set Link & Traffic LED mode */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_MLEDCR, MLEDCR_MOD3 | MLEDCR_LED_TYPE_01), err, TAG, "write DM9058_MLEDCR failed");
    /* Enable packet length filter of broadcast packet */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_BCASTCR, 0xc0), err, TAG, "write DM9058_BCASTCR failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t DM9058_enable_flow_ctrl(esp32_DM9058_t *emac, bool enable)
{
    esp_err_t ret = ESP_OK;
    if (enable) {
        /* send jam pattern (duration time = 1.15ms) when rx free space < 3k bytes */
        ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_BPTR, 0x3F), err, TAG, "write BPTR failed");
        /* flow control: high water threshold = 3k bytes, low water threshold = 8k bytes */
        ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_FCTR, 0x38), err, TAG, "write FCTR failed");
        /* enable flow control */
        ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_FCR, FCR_FLOW_ENABLE), err, TAG, "write FCR failed");
    } else {
        /* disable flow control */
        ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_FCR, 0), err, TAG, "write FCR failed");
    }
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief start DM9058: enable interrupt and start receive
 */
static esp_err_t esp32_DM9058_start(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    /* reset tx and rx memory pointer */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_MPTRCR, MPTRCR_RST_RX | MPTRCR_RST_TX), err, TAG, "write MPTRCR failed");
    /* clear interrupt status */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_ISR, ISR_CLR_STATUS), err, TAG, "write ISR failed");
    /* enable only Rx related interrupts as others are processed synchronously */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_IMR, IMR_PAR | IMR_PRI), err, TAG, "write IMR failed");
    /* enable rx */
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_RCR, &rcr), err, TAG, "read RCR failed");
    rcr |= RCR_RXEN;
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_RCR, rcr), err, TAG, "write RCR failed");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief stop DM9058: disable interrupt and stop receive
 */
static esp_err_t esp32_DM9058_stop(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    /* IMR_PAR should not be cleared unless Wake-on-LAN (WOL) functionality is required */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_IMR, IMR_PAR), err, TAG, "write IMR failed");
    /* disable rx */
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_RCR, &rcr), err, TAG, "read RCR failed");
    rcr &= ~RCR_RXEN;
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_RCR, rcr), err, TAG, "write RCR failed");
    return ESP_OK;
err:
    return ret;
}

IRAM_ATTR static void DM9058_isr_handler(void *arg)
{
    esp32_DM9058_t *emac = (esp32_DM9058_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    /* notify DM9058 task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

static void DM9058_poll_timer(void *arg)
{
    esp32_DM9058_t *emac = (esp32_DM9058_t *)arg;
    xTaskNotifyGive(emac->rx_task_hdl);
}

static esp_err_t esp32_DM9058_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(eth, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac's mediator to null");
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    emac->eth = eth;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t esp32_DM9058_phy_access_compl(esp32_DM9058_t *emac, uint32_t timeout_us)
{
    uint8_t epcr = 0;
    ESP_RETURN_ON_ERROR(DM9058_register_read(emac, DM9058_EPCR, &epcr), TAG, "read EPCR failed");
    uint32_t to = 0;
    if (epcr & EPCR_ERRE) {
        do {
            esp_rom_delay_us(100);
            ESP_RETURN_ON_ERROR(DM9058_register_read(emac, DM9058_EPCR, &epcr), TAG, "read EPCR failed");
            to += 100;
        } while ((epcr & EPCR_ERRE) && to < timeout_us);
        ESP_RETURN_ON_FALSE(!(epcr & EPCR_ERRE), ESP_ERR_TIMEOUT, TAG, "wait for PHY/EEPROM access completion timeouted");
    }
    return ESP_OK;
}

static esp_err_t esp32_DM9058_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);

    /* The following commands need to be performed in atomic manner */
    ESP_RETURN_ON_FALSE(DM9058_mutex_lock(emac), ESP_ERR_TIMEOUT, TAG, "multiple register access mutex timeout");
    /* check if no PHY/EEPROM access is in progress */
    ESP_GOTO_ON_ERROR(esp32_DM9058_phy_access_compl(emac, DM9058_PHY_OPERATION_TIMEOUT_US), err, TAG, "PHY is busy");
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_EPAR, (uint8_t)(((phy_addr << 6) & 0xFF) | phy_reg)), err, TAG, "write EPAR failed");
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_EPDRL, (uint8_t)(reg_value & 0xFF)), err, TAG, "write EPDRL failed");
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_EPDRH, (uint8_t)((reg_value >> 8) & 0xFF)), err, TAG, "write EPDRH failed");
    /* select PHY and select write operation */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_EPCR, EPCR_EPOS | EPCR_ERPRW), err, TAG, "write EPCR failed");
    /* wait for PHY access completion */
    ESP_GOTO_ON_ERROR(esp32_DM9058_phy_access_compl(emac, DM9058_PHY_OPERATION_TIMEOUT_US), err, TAG, "PHY access completion check failed");
err:
    DM9058_mutex_unlock(emac);
    return ret;
}

static esp_err_t esp32_DM9058_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(reg_value, ESP_ERR_INVALID_ARG, TAG, "can't set reg_value to null");
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);

    /* The following commands need to be performed in atomic manner */
    ESP_RETURN_ON_FALSE(DM9058_mutex_lock(emac), ESP_ERR_TIMEOUT, TAG, "multiple register access mutex timeout");
    /* check if no PHY/EEPROM access is in progress */
    ESP_GOTO_ON_ERROR(esp32_DM9058_phy_access_compl(emac, DM9058_PHY_OPERATION_TIMEOUT_US), err, TAG, "PHY is busy");
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_EPAR, (uint8_t)(((phy_addr << 6) & 0xFF) | phy_reg)), err, TAG, "write EPAR failed");
    /* Select PHY and select read operation */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_EPCR, EPCR_EPOS | EPCR_ERPRR), err, TAG, "write EPCR failed");
    /* wait for PHY access completion */
    ESP_GOTO_ON_ERROR(esp32_DM9058_phy_access_compl(emac, DM9058_PHY_OPERATION_TIMEOUT_US), err, TAG, "PHY access completion check failed");
    uint8_t value_h = 0;
    uint8_t value_l = 0;
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_EPDRH, &value_h), err, TAG, "read EPDRH failed");
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_EPDRL, &value_l), err, TAG, "read EPDRL failed");
    *reg_value = (value_h << 8) | value_l;
err:
    DM9058_mutex_unlock(emac);
    return ret;
}

static esp_err_t esp32_DM9058_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac addr to null");
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    memcpy(emac->addr, addr, 6);
    ESP_GOTO_ON_ERROR(DM9058_set_mac_addr(emac), err, TAG, "set mac address failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t esp32_DM9058_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac addr to null");
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    memcpy(addr, emac->addr, 6);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t esp32_DM9058_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    switch (link) {
    case ETH_LINK_UP:
        ESP_GOTO_ON_ERROR(mac->start(mac), err, TAG, "DM9058 start failed");
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_start_periodic(emac->poll_timer, emac->poll_period_ms * 1000),
                              err, TAG, "start poll timer failed");
        }
        break;
    case ETH_LINK_DOWN:
        ESP_GOTO_ON_ERROR(mac->stop(mac), err, TAG, "DM9058 stop failed");
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

static esp_err_t esp32_DM9058_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
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

static esp_err_t esp32_DM9058_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
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

static esp_err_t esp32_DM9058_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_RCR, &rcr), err, TAG, "read RCR failed");
    if (enable) {
        rcr |= RCR_PRMSC;
    } else {
        rcr &= ~RCR_PRMSC;
    }
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_RCR, rcr), err, TAG, "write RCR failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t esp32_DM9058_set_all_multicast(esp_eth_mac_t *mac, bool enable)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_RCR, &rcr), err, TAG, "read RCR failed");
    if (enable) {
        rcr |= RCR_ALL_MCAST;
    } else {
        rcr &= ~RCR_ALL_MCAST;
    }
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_RCR, rcr), err, TAG, "write RCR failed");
err:
    return ret;
}

static esp_err_t esp32_DM9058_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    emac->flow_ctrl_enabled = enable;
    return ESP_OK;
}

static esp_err_t esp32_DM9058_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    // we want to enable flow control, and peer does support pause function
    // then configure the MAC layer to enable flow control feature
    if (emac->flow_ctrl_enabled && ability) {
        DM9058_enable_flow_ctrl(emac, true);
    } else {
        DM9058_enable_flow_ctrl(emac, false);
        ESP_LOGD(TAG, "Flow control not enabled for the link");
    }
    return ESP_OK;
}

static esp_err_t esp32_DM9058_custom_ioctl(esp_eth_mac_t *mac, int cmd, void *data)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    esp_eth_ptp_dm9058_time_t ptp_time = {0};
    esp_eth_ptp_dm9058_time_t *time = (esp_eth_ptp_dm9058_time_t *)data;

    switch (cmd) {
    case ETH_MAC_DM9058_CMD_PTP_ENABLE: {
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "PTP_ENABLE expects esp_eth_ptp_dm9058_enable_config_t*");
        esp_eth_ptp_dm9058_enable_config_t *cfg = (esp_eth_ptp_dm9058_enable_config_t *)data;
        ESP_LOGD(TAG, "PTP_ENABLE: enable=%d, transport=%d", cfg->enable, cfg->transport);
        return esp_eth_ptp_dm9058_enable(&emac->ptp, cfg->enable, cfg->transport);
    }
    case ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS:
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "PTP_AUTO_PROCESS expects bool*");
        emac->ptp_auto_process = *(bool *)data;
        return ESP_OK;
    case ETH_MAC_DM9058_CMD_S_PTP_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "S_PTP_TIME expects eth_mac_time_t*");
        ptp_time.seconds = time->seconds;
        ptp_time.nanoseconds = time->nanoseconds;
        return esp_eth_ptp_dm9058_set_time(&emac->ptp, &ptp_time);
    case ETH_MAC_DM9058_CMD_G_PTP_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "G_PTP_TIME expects eth_mac_time_t*");
        ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_get_time(&emac->ptp, &ptp_time), TAG, "get ptp time failed");
        time->seconds = ptp_time.seconds;
        time->nanoseconds = ptp_time.nanoseconds;
        return ESP_OK;
    case ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ:
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "ADJ_PTP_FREQ expects int32_t*");
        return esp_eth_ptp_dm9058_adj_freq(&emac->ptp, *(int32_t *)data);
    case ETH_MAC_DM9058_CMD_ADJ_PTP_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "ADJ_PTP_TIME expects eth_mac_time_t*");
        ptp_time.seconds = (uint32_t)((int32_t)time->seconds);
        ptp_time.nanoseconds = (uint32_t)((int32_t)time->nanoseconds);
        return esp_eth_ptp_dm9058_adj_time(&emac->ptp, &ptp_time);
    case ETH_MAC_DM9058_CMD_G_PTP_TX_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "G_PTP_TX_TIME expects eth_mac_time_t*");
        ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_get_tx_timestamp(&emac->ptp, &ptp_time), TAG, "get tx timestamp failed");
        time->seconds = ptp_time.seconds;
        time->nanoseconds = ptp_time.nanoseconds;
        return ESP_OK;
    case ETH_MAC_DM9058_CMD_G_PTP_RX_TIME:
        /* RX timestamp is now delivered inline via stack_input_info; polling via ioctl is no longer supported. */
        return ESP_ERR_NOT_SUPPORTED;
    case ETH_MAC_DM9058_CMD_S_TARGET_TIME:
        // Target time not yet supported in DM9058
        ESP_LOGW(TAG, "Target time feature not yet implemented for DM9058");
        return ESP_ERR_NOT_SUPPORTED;
//		ESP_LOGW(PTP_TAG, "case ETH_MAC_DM9058_CMD_S_TARGET_TIME of DM9058");
//        ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, PTP_TAG, "PTP set target time invalid argument, cant' be NULL");
//        emac->target_time = *(esp_eth_ptp_dm9058_time_t *)data;
//        emac->target_time_valid = true;
//        if (DM9058_mutex_lock(emac)) {
//            esp_eth_ptp_dm9058_time_t now = {0};
//            esp_eth_ptp_dm9058_get_time(&emac->ptp, &now); //dm9058_ptp_get_time(emac, &now);
//            DM9058_mutex_unlock(emac);
//            if (dm9058_time_reached(&now, &emac->target_time)) {
//                emac->target_time_valid = false;
//                if (emac->ts_target_exceed_cb_from_isr) {
//                    emac->ts_target_exceed_cb_from_isr(emac->eth, NULL);
//                }
//                return ESP_OK;
//            }
//        }
//        dm9058_ptp_timer_start(emac);
//        return ESP_OK;
    case ETH_MAC_DM9058_CMD_S_TARGET_CB:
        // Target callback not yet supported in DM9058
        ESP_LOGW(TAG, "Target callback feature not yet implemented for DM9058");
        return ESP_ERR_NOT_SUPPORTED;
//		ESP_LOGW(PTP_TAG, "case ETH_MAC_DM9058_CMD_S_TARGET_CB of DM9058");
//        ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, PTP_TAG, "PTP set target callback invalid argument, cant' be NULL");
//        emac->ts_target_exceed_cb_from_isr = (dm9058_ts_target_cb_t)data;
//        dm9058_ptp_timer_start(emac);
//        return ESP_OK;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t DM9058_wait_tx_pointer(esp32_DM9058_t *emac, uint32_t timeout_us)
{
    uint8_t reg_nsr = 0;
    int64_t start = esp_timer_get_time();

    do {
        ESP_RETURN_ON_ERROR(DM9058_register_read(emac, DM9058_NSR, &reg_nsr), TAG, "read NSR failed");
        reg_nsr &= (NSR_TX2END | NSR_TX1END);
        if (reg_nsr) {
            break;
        }
    } while ((esp_timer_get_time() - start) < timeout_us);

    ESP_RETURN_ON_FALSE(reg_nsr != 0, ESP_ERR_INVALID_STATE, TAG, "last transmit still in progress, cannot send");

    if (reg_nsr == (NSR_TX2END | NSR_TX1END)) {
        ESP_RETURN_ON_ERROR(DM9058_register_write(emac, DM9058_MPTRCR, MPTRCR_RST_TX), TAG, "write MPTRCR failed");
    }
    return ESP_OK;
}

static esp_err_t DM9058_wait_tx_complete(esp32_DM9058_t *emac, uint32_t timeout_us)
{
    uint8_t tcr = 0;
    int64_t start = esp_timer_get_time();

    do {
        ESP_RETURN_ON_ERROR(DM9058_register_read(emac, DM9058_TCR, &tcr), TAG, "read TCR failed");
        if ((tcr & TCR_TXREQ) == 0) {
            return ESP_OK;
        }
    } while ((esp_timer_get_time() - start) < timeout_us);

    return ESP_ERR_TIMEOUT;
}

static esp_err_t DM9058_trigger_tx_request(esp32_DM9058_t *emac)
{
    uint8_t tcr = 0;
    ESP_RETURN_ON_ERROR(DM9058_register_read(emac, DM9058_TCR, &tcr), TAG, "read TCR failed");
    tcr |= TCR_TXREQ;
    ESP_RETURN_ON_ERROR(DM9058_register_write(emac, DM9058_TCR, tcr), TAG, "write TCR failed");
    return ESP_OK;
}

static esp_err_t esp32_DM9058_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    const uint32_t tx_pointer_timeout_us = 500;
    const uint32_t tx_complete_timeout_us = 500;
    esp_err_t ret = ESP_OK;
    bool tx_locked = false;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);

    ESP_GOTO_ON_FALSE(buf != NULL, ESP_ERR_INVALID_ARG, err, TAG, "tx buffer is null");
    ESP_GOTO_ON_FALSE(length <= ETH_MAX_PACKET_SIZE, ESP_ERR_INVALID_ARG, err,
                      TAG, "frame size is too big (actual %" PRIu32 ", maximum %d)", length, ETH_MAX_PACKET_SIZE);

    /* Step 1: wait TX pointer ready (similar to tx_pointer_timeout flow) */
    ESP_GOTO_ON_ERROR(DM9058_wait_tx_pointer(emac, tx_pointer_timeout_us), err, TAG, "wait tx pointer failed");

    ESP_GOTO_ON_FALSE(DM9058_mutex_lock(emac), ESP_ERR_TIMEOUT, err, TAG, "multiple register access mutex timeout");
    tx_locked = true;

    /* Step 2: PTP packet parse/configure (similar to ptp_tx_tstamp_parse_packet flow) */
    if (emac->ptp_auto_process && emac->ptp.enabled) {
        esp_err_t ptp_ret = esp_eth_ptp_dm9058_prepare_tx_locked(&emac->ptp, buf, length, emac->ptp_two_step_mode);
        if (ptp_ret != ESP_OK) {
            ESP_LOGW(TAG, "prepare tx ptp failed: %s", esp_err_to_name(ptp_ret));
        }
    }

    /* Step 3: write frame (similar to cspi_tx_write flow) */
    /* set tx length */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_TXPLL, length & 0xFF), err, TAG, "write TXPLL failed");
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_TXPLH, (length >> 8) & 0xFF), err, TAG, "write TXPLH failed");
    /* copy data to tx memory */
    ESP_GOTO_ON_ERROR(DM9058_memory_write(emac, buf, length), err, TAG, "write memory failed");

    /* Step 4: explicit TX request trigger (equivalent to HAL_write_reg(DM9058_TCR, tcr_wr)) */
    ESP_GOTO_ON_ERROR(DM9058_trigger_tx_request(emac), err, TAG, "trigger tx request failed");

    DM9058_mutex_unlock(emac);
    tx_locked = false;

    /* Step 5: wait tx complete (similar to tx_compl_timeout flow) */
    if (emac->ptp_auto_process && emac->ptp.enabled) {
        esp_err_t wait_ret = DM9058_wait_tx_complete(emac, tx_complete_timeout_us);
        if (wait_ret != ESP_OK) {
            ESP_LOGW(TAG, "wait tx complete timeout: %s", esp_err_to_name(wait_ret));
        }
    }

    return ESP_OK;
err:
    if (tx_locked) {
        DM9058_mutex_unlock(emac);
    }
    return ret;
}

static esp_err_t esp32_DM9058_transmit_ctrl_vargs(esp_eth_mac_t *mac, void *ctrl, uint32_t argc, va_list args)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    uint8_t *buf = va_arg(args, uint8_t *);
    uint32_t length = va_arg(args, uint32_t);
    esp_err_t ret = esp32_DM9058_transmit(mac, buf, length);
    if (ret == ESP_OK && ctrl && emac->ptp_auto_process && emac->ptp.enabled) {
        // Try to get TX timestamp
        esp_eth_ptp_dm9058_time_t ts;
        if (esp_eth_ptp_dm9058_get_tx_timestamp(&emac->ptp, &ts) == ESP_OK) {
            eth_mac_time_t *eth_ts = (eth_mac_time_t *)ctrl;
            eth_ts->seconds = ts.seconds;
            eth_ts->nanoseconds = ts.nanoseconds;
        }
    }
    return ret;
}

static esp_err_t DM9058_skip_recv_frame(esp32_DM9058_t *emac, uint16_t rx_length)
{
    esp_err_t ret = ESP_OK;
    uint8_t mrrh, mrrl;
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_MRRH, &mrrh), err, TAG, "read MDRAH failed");
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_MRRL, &mrrl), err, TAG, "read MDRAL failed");
    uint16_t addr = mrrh << 8 | mrrl;
    addr += rx_length;
    if (addr > DM9058_RX_MEM_MAX_SIZE) {
        addr = addr - DM9058_RX_MEM_MAX_SIZE + DM9058_RX_MEM_START_ADDR;
    }
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_MRRL, addr & 0xFF), err, TAG, "write MDRAL failed");
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_MRRH, addr >> 8), err, TAG, "write MDRAH failed");
err:
    return ret;
}

static esp_err_t DM9058_flush_recv_queue(esp32_DM9058_t *emac)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(emac->parent.stop(&emac->parent), err, TAG, "stop DM9058 failed");
    /* reset rx fifo pointer */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_MPTRCR, MPTRCR_RST_RX), err, TAG, "write MPTRCR failed");
    esp_rom_delay_us(10);
    ESP_GOTO_ON_ERROR(emac->parent.start(&emac->parent), err, TAG, "start DM9058 failed");
err:
    return ret;
}

static esp_err_t DM9058_handle_rx_ptp_timestamp(esp32_DM9058_t *emac, const DM9058_rx_header_t *header,
                                                 esp_eth_ptp_dm9058_time_t *out_ts, bool *out_ts_valid)
{
    if (out_ts_valid) {
        *out_ts_valid = false;
    }

    if (!emac->ptp_auto_process || !emac->ptp.enabled) {
        return ESP_OK;
    }

    uint8_t rx_header_bytes[DM9058_RX_HDR_SIZE] = {
        header->flag,
        header->status,
        header->length_low,
        header->length_high,
    };
    esp_eth_ptp_dm9058_rx_info_t rx_info = {0};
    esp_err_t parse_ret = esp_eth_ptp_dm9058_parse_rx_header(rx_header_bytes, sizeof(rx_header_bytes), ETH_MAX_PACKET_SIZE, &rx_info);

    size_t timestamp_len = 0;
    if (parse_ret == ESP_OK) {
        timestamp_len = rx_info.timestamp_available ? rx_info.timestamp_len : 0;
    } else if (header->status & DM9058_RSR_RXTS_EN) {
        timestamp_len = (header->status & DM9058_RSR_RXTS_LEN) ? 8 : 4;
    }

    if (timestamp_len == 0) {
        return ESP_OK;
    }

    uint8_t ts_buffer[8] = {0};
    ESP_RETURN_ON_FALSE(timestamp_len <= sizeof(ts_buffer), ESP_ERR_INVALID_SIZE, TAG, "timestamp too long");
    ESP_RETURN_ON_ERROR(DM9058_memory_read(emac, ts_buffer, timestamp_len), TAG, "read rx timestamp failed");

    if (parse_ret == ESP_OK && out_ts && out_ts_valid) {
        esp_err_t decode_ret = esp_eth_ptp_dm9058_rx_timestamp(ts_buffer, timestamp_len, out_ts);
        if (decode_ret == ESP_OK) {
            *out_ts_valid = true;
            ESP_LOGD(TAG, "RX PTP timestamp: %lu.%09lu", out_ts->seconds, out_ts->nanoseconds);
        } else {
            ESP_LOGW(TAG, "decode rx timestamp failed: %s", esp_err_to_name(decode_ret));
        }
    }

    return ESP_OK;
}

#define TIMES_TO_RST               10

/**
 * @brief  Process RX buffer fire time
 *
 * @param  histc   History counter array
 * @param  csize   Size of array
 * @param  i       Current index
 * @param  rxb     RX buffer value
 * @return         TIMES_TO_RST if reset needed, 0 otherwise
 */
static uint8_t ret_fire_time(uint8_t *histc, int csize, int i, uint8_t rxb)
{
//   printf(" _dm9058f rxb %02x (times %2d)%c\r\n",
//          rxb, histc[i],
//          (histc[i] == 2) ? '*' : ' ');

  if (histc[i] >= (TIMES_TO_RST / 2))
    ESP_LOGE(TAG, "RX buffer 0x%02x has been received %d times", rxb, histc[i]);

  if (histc[i] >= TIMES_TO_RST)
  {
    // dm9058_show_rxbstatistic(histc, csize);
    histc[i] = 1;
    return TIMES_TO_RST;
  }

  return 0;
}

/**
 * @brief  Evaluate RX buffer status and handle errors
 *
 * @param  rxb  RX buffer value to evaluate
 * @return      0 if successful, error code otherwise
 */
//static
uint16_t env_evaluate_rxb(uint8_t rxb)
{
  int i;
  static uint8_t histc[254] = {0};
  uint8_t times = 1;

  for (i = 0; i < sizeof(histc); i++)
  {
    if (rxb == (i + 2))
    {
      histc[i]++;
      times = ret_fire_time(histc, sizeof(histc), i, rxb);

      if (times == 0)
        return 0;

      return 1;
    }
  }

  return 1;
}

/**
 * @brief Read one RX frame into the internal buffer.
 *
 * @param[out] out_ts Optional RX PTP timestamp output. Pass NULL when the caller
 *                    uses a legacy receive path that cannot propagate metadata.
 * @param[out] out_ts_valid Optional flag paired with out_ts. Pass NULL together
 *                          with out_ts when timestamp metadata is not needed.
 */
static esp_err_t DM9058_frame_to_rx_buffer(esp32_DM9058_t *emac, uint16_t *size,
                                            esp_eth_ptp_dm9058_time_t *out_ts, bool *out_ts_valid)
{
    esp_err_t ret = ESP_OK;
    uint8_t rxbyte = 0;
    __attribute__((aligned(4))) DM9058_rx_header_t header; // SPI driver needs the rx buffer 4 byte align
    bool try_again = false;

    do {
        *size = 0;
        uint8_t reg_nsr = 0;
        ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
        if (reg_nsr & NSR_RXRDY) {
            /* dummy read, get the most updated data */
            ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
            ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
            if (0x01 != rxbyte) {
                if (env_evaluate_rxb(rxbyte)) {
                    ESP_GOTO_ON_ERROR(DM9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                    ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "unexpected rx flag (0x%" PRIx8 "), reset rx fifo pointer", rxbyte);
                }
                return ESP_OK;
            }
            ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, (uint8_t *)&header, sizeof(header)), err, TAG, "read rx header failed");
            uint16_t rx_len = header.length_low + (header.length_high << 8);
            /* store the whole frame to preallocated memory */
            if (rx_len <= ETH_MAX_PACKET_SIZE) {
                ESP_GOTO_ON_ERROR(DM9058_handle_rx_ptp_timestamp(emac, &header, out_ts, out_ts_valid), err, TAG, "handle rx ptp timestamp failed");
                ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, emac->rx_buffer, rx_len), err, TAG, "read rx data failed");
            } else {
                /* we are out of sync or data is corrupted, there is no way how to fix position in rx fifo => flush all */
                ESP_GOTO_ON_ERROR(DM9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "invalid frame length, reset rx fifo pointer");
            }
            if (header.status & (RSR_RF | RSR_RWTO | RSR_CE | RSR_FOE)) {
                /* erroneous frames should not be forwarded by DM9058, however, if it happens, just skip it */
                DM9058_skip_recv_frame(emac, rx_len);
                ESP_LOGE(TAG, "receive status error: %" PRIx8 "H", header.status);
                /* try again to check if other frame is waiting */
                try_again = true;
            } else {
                *size = rx_len;
            }
        }
    } while (try_again);
err:
    return ret;
}

/**
 * @brief Legacy MAC receive callback required by `esp_eth_mac_t`.
 *
 * This path only returns frame bytes to the caller. Any optional RX timestamp
 * metadata is intentionally discarded because the standard MAC receive API has
 * no output channel for it.
 */
static esp_err_t esp32_DM9058_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    uint16_t byte_count = 0;
    emac->packets_remain = false;

    /* `esp_eth_mac_t.receive()` has no channel for RX timestamp metadata, so ignore it here. */
    ESP_GOTO_ON_ERROR(DM9058_frame_to_rx_buffer(emac, &byte_count, NULL, NULL), err, TAG, "moving data to internal rx_buffer failed");
    /* silently return when no frame is waiting */
    if (!byte_count) {
        goto err;
    }
    /* do not include 4 bytes CRC at the end */
    uint16_t rx_len = byte_count - ETH_CRC_LEN;
    if (buf == emac->rx_buffer) {
        /* if we use internal buffer, we are done */
        *length = rx_len;
    } else {
        /* if frame to be copied to external buffer allocated by user */
        ESP_GOTO_ON_FALSE(buf, ESP_ERR_INVALID_ARG, err, TAG, "buffer can't be NULL");
        /* frames larger than expected will be truncated */
        uint16_t copy_len = rx_len > *length ? *length : rx_len;
        memcpy(buf, emac->rx_buffer, copy_len);
        *length = copy_len;
    }

    uint8_t reg_nsr = 0;
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
    emac->packets_remain = (reg_nsr & NSR_RXRDY);
    return ESP_OK;
err:
    *length = 0;
    return ret;
}

static esp_err_t esp32_DM9058_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    if (emac->int_gpio_num >= 0) {
        gpio_func_sel(emac->int_gpio_num, PIN_FUNC_GPIO);
        gpio_input_enable(emac->int_gpio_num);
        gpio_pulldown_en(emac->int_gpio_num);
        gpio_set_intr_type(emac->int_gpio_num, GPIO_INTR_POSEDGE);
        gpio_intr_enable(emac->int_gpio_num);
        gpio_isr_handler_add(emac->int_gpio_num, DM9058_isr_handler, emac);
    }
    ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL), err, TAG, "lowlevel init failed");
    /* reset DM9058 */
    ESP_GOTO_ON_ERROR(DM9058_reset(emac), err, TAG, "reset DM9058 failed");
    /* verify chip id */
    ESP_GOTO_ON_ERROR(DM9058_verify_id(emac), err, TAG, "verify chip ID failed");
    /* default setup of internal registers */
    ESP_GOTO_ON_ERROR(DM9058_setup_default(emac), err, TAG, "DM9058 default setup failed");
    /* clear multicast hash table */
    ESP_GOTO_ON_ERROR(DM9058_clear_multicast_table(emac), err, TAG, "clear multicast table failed");
    /* get emac address from eeprom */
    ESP_GOTO_ON_ERROR(DM9058_get_mac_addr(emac), err, TAG, "fetch ethernet mac address failed");
    return ESP_OK;
err:
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
}

static esp_err_t esp32_DM9058_deinit(esp_eth_mac_t *mac)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
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

/**
 * @brief RX-task helper that preserves optional PTP RX timestamp metadata.
 *
 * Reads one frame into `emac->rx_buffer` and returns the optional PTP hardware
 * timestamp via local output parameters, avoiding any shared state on the emac
 * struct. Unlike the legacy MAC receive callback, this path is used together
 * with `stack_input_info()` so the task can forward both frame bytes and
 * timestamp metadata to the upper layer.
 */
static esp_err_t DM9058_task_receive(esp32_DM9058_t *emac, uint32_t *length,
                                     eth_mac_time_t *ts_out, bool *ts_valid_out)
{
    esp_err_t ret = ESP_OK;
    uint16_t byte_count = 0;
    emac->packets_remain = false;
    *ts_valid_out = false;

    esp_eth_ptp_dm9058_time_t local_ts = {0};
    bool local_ts_valid = false;

    ESP_GOTO_ON_ERROR(DM9058_frame_to_rx_buffer(emac, &byte_count, &local_ts, &local_ts_valid),
                      err, TAG, "moving data to internal rx_buffer failed");
    if (!byte_count) {
        goto err;
    }
    *length = byte_count - ETH_CRC_LEN;

    if (local_ts_valid) {
        ts_out->seconds    = local_ts.seconds;
        ts_out->nanoseconds = local_ts.nanoseconds;
        *ts_valid_out = true;
    }

    uint8_t reg_nsr = 0;
    ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
    emac->packets_remain = (reg_nsr & NSR_RXRDY);
    return ESP_OK;
err:
    *length = 0;
    return ret;
}

static void esp32_DM9058_task(void *arg)
{
    esp32_DM9058_t *emac = (esp32_DM9058_t *)arg;
    uint8_t status = 0;
    while (1) {
        // check if the task receives any notification
        if (emac->int_gpio_num >= 0) {                                   // if in interrupt mode
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 &&    // if no notification ...
                    gpio_get_level(emac->int_gpio_num) == 0) {           // ...and no interrupt asserted
                continue;                                                // -> just continue to check again
            }
        } else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        /* clear interrupt status */
        DM9058_register_read(emac, DM9058_ISR, &status);
        DM9058_register_write(emac, DM9058_ISR, status);
        /* packet received */
        if (status & ISR_PR) {
            do {
                uint32_t buf_len = 0;
                eth_mac_time_t rx_ts = {0};
                bool rx_ts_valid = false;
                if (DM9058_task_receive(emac, &buf_len, &rx_ts, &rx_ts_valid) == ESP_OK) {
                    /* if there is waiting frame */
                    if (buf_len > 0) {
                        uint8_t *buffer = malloc(buf_len);
                        if (buffer == NULL) {
                            ESP_LOGE(TAG, "no mem for receive buffer");
                        } else {
                            void *rx_info = NULL;
                            dm9058_ptp_packet_info_t info = {0};
                            esp_err_t pkt_info_ret = dm9058_parse_packet_info(emac->rx_buffer, buf_len, &info);
                            memcpy(buffer, emac->rx_buffer, buf_len);
                            ESP_LOGD(TAG, "receive len=%" PRIu32, buf_len);
                            /* Pass timestamp metadata only when available; otherwise keep info NULL. */
                            if (rx_ts_valid &&
                                pkt_info_ret == ESP_OK &&
                                info.is_ptp &&
                                (info.message_type == ESP_ETH_PTP_DM9058_MSG_SYNC ||
                                 info.message_type == ESP_ETH_PTP_DM9058_MSG_DELAY_REQ ||
                                 info.message_type == ESP_ETH_PTP_DM9058_MSG_PDELAY_REQ ||
                                 info.message_type == ESP_ETH_PTP_DM9058_MSG_PDELAY_RESP)) {
                                rx_info = &rx_ts;
                                ESP_LOGD(TAG, "forward rx ts to stack: %lu.%09lu", rx_ts.seconds, rx_ts.nanoseconds);
                            } else if (rx_ts_valid &&
                                       pkt_info_ret == ESP_OK &&
                                       info.is_ptp &&
                                       info.message_type != ESP_ETH_PTP_DM9058_MSG_SYNC &&
                                       info.message_type != ESP_ETH_PTP_DM9058_MSG_DELAY_REQ &&
                                       info.message_type != ESP_ETH_PTP_DM9058_MSG_PDELAY_REQ &&
                                       info.message_type != ESP_ETH_PTP_DM9058_MSG_PDELAY_RESP) {
                                ESP_LOGD(TAG, "timestamp filtered out (non-event msg), msg_type=0x%02x", info.message_type);
                            }
                            /* pass the buffer and optional rx info to stack */
                            emac->eth->stack_input_info(emac->eth, buffer, buf_len, rx_info);
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

static esp_err_t esp32_DM9058_del(esp_eth_mac_t *mac)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
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

esp_eth_mac_t *esp_eth_mac_new_dm9058(const eth_dm9058_config_t *DM9058_config, const eth_mac_config_t *mac_config)
{
    esp_eth_mac_t *ret = NULL;
    esp32_DM9058_t *emac = NULL;
    ESP_GOTO_ON_FALSE(DM9058_config, NULL, err, TAG, "can't set DM9058 specific config to null");
    ESP_GOTO_ON_FALSE(mac_config, NULL, err, TAG, "can't set mac config to null");
    ESP_GOTO_ON_FALSE((DM9058_config->int_gpio_num >= 0) != (DM9058_config->poll_period_ms > 0), NULL, err, TAG, "invalid configuration argument combination");
    emac = calloc(1, sizeof(esp32_DM9058_t));
    ESP_GOTO_ON_FALSE(emac, NULL, err, TAG, "calloc emac failed");
    /* bind methods and attributes */
    emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->int_gpio_num = DM9058_config->int_gpio_num;
    emac->poll_period_ms = DM9058_config->poll_period_ms;
    emac->parent.set_mediator = esp32_DM9058_set_mediator;
    emac->parent.init = esp32_DM9058_init;
    emac->parent.deinit = esp32_DM9058_deinit;
    emac->parent.start = esp32_DM9058_start;
    emac->parent.stop = esp32_DM9058_stop;
    emac->parent.del = esp32_DM9058_del;
    emac->parent.write_phy_reg = esp32_DM9058_write_phy_reg;
    emac->parent.read_phy_reg = esp32_DM9058_read_phy_reg;
    emac->parent.set_addr = esp32_DM9058_set_addr;
    emac->parent.get_addr = esp32_DM9058_get_addr;
    emac->parent.set_speed = esp32_DM9058_set_speed;
    emac->parent.set_duplex = esp32_DM9058_set_duplex;
    emac->parent.set_link = esp32_DM9058_set_link;
    emac->parent.set_promiscuous = esp32_DM9058_set_promiscuous;
    emac->parent.set_all_multicast = esp32_DM9058_set_all_multicast;
    emac->parent.set_peer_pause_ability = esp32_DM9058_set_peer_pause_ability;
    emac->parent.enable_flow_ctrl = esp32_DM9058_enable_flow_ctrl;
    emac->parent.transmit = esp32_DM9058_transmit;
    emac->parent.receive = esp32_DM9058_receive;
    emac->parent.add_mac_filter = esp32_DM9058_add_mac_filter;
    emac->parent.rm_mac_filter = esp32_DM9058_rm_mac_filter;
    emac->parent.custom_ioctl = esp32_DM9058_custom_ioctl;
    emac->parent.transmit_ctrl_vargs = esp32_DM9058_transmit_ctrl_vargs;

    if (DM9058_config->custom_spi_driver.init != NULL && DM9058_config->custom_spi_driver.deinit != NULL
            && DM9058_config->custom_spi_driver.read != NULL && DM9058_config->custom_spi_driver.write != NULL) {
        ESP_LOGD(TAG, "Using user's custom SPI Driver");
        emac->spi.init = DM9058_config->custom_spi_driver.init;
        emac->spi.deinit = DM9058_config->custom_spi_driver.deinit;
        emac->spi.read = DM9058_config->custom_spi_driver.read;
        emac->spi.write = DM9058_config->custom_spi_driver.write;
        /* Custom SPI driver device init */
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(DM9058_config->custom_spi_driver.config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    } else {
        ESP_LOGD(TAG, "Using default SPI Driver");
        emac->spi.init = DM9058_spi_init;
        emac->spi.deinit = DM9058_spi_deinit;
        emac->spi.read = DM9058_spi_read;
        emac->spi.write = DM9058_spi_write;
        /* SPI device init */
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(DM9058_config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    }

    /* create mutex for accessing multiple registers in atomic manner */
    emac->multi_reg_axs_mutex = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(emac->multi_reg_axs_mutex, NULL, err, TAG, "create multi registers access mutex failed");

    const esp_eth_ptp_dm9058_ops_t ptp_ops = {
        .reg_read = DM9058_ptp_reg_read,
        .reg_write = DM9058_ptp_reg_write,
        .reg_burst_read = DM9058_ptp_reg_burst_read,
        .reg_burst_write = DM9058_ptp_reg_burst_write,
        .delay_us = DM9058_ptp_delay_us,
        .delay_ms = DM9058_ptp_delay_ms,
        .lock = DM9058_ptp_lock,
        .unlock = DM9058_ptp_unlock,
    };
    ESP_GOTO_ON_FALSE(esp_eth_ptp_dm9058_init(&emac->ptp, emac, &ptp_ops) == ESP_OK, NULL, err, TAG, "init dm9058 ptp context failed");
    emac->ptp_auto_process = true;
#ifdef CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE
    emac->ptp_two_step_mode = true;
#else
    emac->ptp_two_step_mode = false;
#endif

    ESP_LOGD(TAG, "ptp_two_step_mode: %s", emac->ptp_two_step_mode ? "enabled" : "disabled");
    /* create DM9058 task */
    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = esp_cpu_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(esp32_DM9058_task, "DM9058_tsk", mac_config->rx_task_stack_size, emac,
                                                   mac_config->rx_task_prio, &emac->rx_task_hdl, core_num);
    ESP_GOTO_ON_FALSE(xReturned == pdPASS, NULL, err, TAG, "create DM9058 task failed");

    emac->rx_buffer = heap_caps_malloc(ETH_MAX_PACKET_SIZE, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(emac->rx_buffer, NULL, err, TAG, "RX buffer allocation failed");

    if (emac->int_gpio_num < 0) {
        const esp_timer_create_args_t poll_timer_args = {
            .callback = DM9058_poll_timer,
            .name = "esp32_spi_poll_timer",
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
