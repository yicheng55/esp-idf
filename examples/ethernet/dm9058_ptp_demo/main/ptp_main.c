/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "ethernet_init.h"
#include "esp_vfs_l2tap.h"
#include "freertos/event_groups.h"
#include "ptpd.h"

#include "esp_eth_time.h"
#if CONFIG_EXAMPLE_PTP_TRANSPORT_IPV4
#include "esp_netif.h"
#endif

static const char *TAG = "ptp_example";

#define ETH_CONNECTED_BIT BIT0

static EventGroupHandle_t s_eth_event_group;

static esp_eth_handle_t *s_eth_handles;
static uint8_t s_eth_port_cnt;

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == ETH_EVENT) {
        switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            if (s_eth_event_group) {
                xEventGroupSetBits(s_eth_event_group, ETH_CONNECTED_BIT);
            }
            ESP_LOGI(TAG, "Ethernet Link Up");
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            if (s_eth_event_group) {
                xEventGroupClearBits(s_eth_event_group, ETH_CONNECTED_BIT);
            }
            ESP_LOGW(TAG, "Ethernet Link Down");
            break;
        default:
            break;
        }
    }
}

void init_ethernet_and_netif(void)
{
    uint8_t eth_port_cnt;
    esp_eth_handle_t *eth_handles;
    esp_err_t ret;

    ESP_LOGI(TAG, "[1] esp_event_loop_create_default");
    ret = esp_event_loop_create_default();
    ESP_LOGI(TAG, "[1] ret=%d (%s)", ret, esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);

    s_eth_event_group = xEventGroupCreate();

    ESP_LOGI(TAG, "[7] esp_event_handler_register");
    ret = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    ESP_LOGI(TAG, "[7] ret=%d (%s)", ret, esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "[2] example_eth_init");
    ret = example_eth_init(&eth_handles, &eth_port_cnt);
    ESP_LOGI(TAG, "[2] ret=%d (%s)", ret, esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);
    s_eth_handles = eth_handles;
    s_eth_port_cnt = eth_port_cnt;

    ESP_LOGI(TAG, "[3] esp_netif_init");
    ret = esp_netif_init();
    ESP_LOGI(TAG, "[3] ret=%d (%s)", ret, esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "[4] esp_vfs_l2tap_intf_register");
    ret = esp_vfs_l2tap_intf_register(NULL);
    ESP_LOGI(TAG, "[4] ret=%d (%s)", ret, esp_err_to_name(ret));
    ESP_ERROR_CHECK(ret);

    esp_netif_inherent_config_t esp_netif_base_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t esp_netif_config = {
        .base = &esp_netif_base_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    char if_key_str[10];
    char if_desc_str[10];
    char num_str[3];
    for (int i = 0; i < eth_port_cnt; i++) {
        itoa(i, num_str, 10);
        strcat(strcpy(if_key_str, "ETH_"), num_str);
        strcat(strcpy(if_desc_str, "eth"), num_str);
        esp_netif_base_config.if_key = if_key_str;
        esp_netif_base_config.if_desc = if_desc_str;
        esp_netif_base_config.route_prio -= i*5;
        esp_netif_t *eth_netif = esp_netif_new(&esp_netif_config);

        // attach Ethernet driver to TCP/IP stack
        ESP_LOGI(TAG, "[5.%d] esp_netif_attach", i);
        ret = esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handles[i]));
        ESP_LOGI(TAG, "[5.%d] ret=%d (%s)", i, ret, esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    }

    for (int i = 0; i < eth_port_cnt; i++) {
        ESP_LOGI(TAG, "[6.%d] esp_eth_start", i);
        ret = esp_eth_start(eth_handles[i]);
        ESP_LOGI(TAG, "[6.%d] ret=%d (%s)", i, ret, esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    }

    EventBits_t bits = xEventGroupWaitBits(s_eth_event_group, ETH_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(5000));
    if ((bits & ETH_CONNECTED_BIT) == 0) {
        ESP_LOGW(TAG, "Ethernet Link Up timeout");
    }

#if CONFIG_EXAMPLE_PTP_TRANSPORT_IPV4
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("ETH_0");
    if (netif) {
        for (int attempt = 0; attempt < 120; attempt++) {
            esp_netif_ip_info_t ip;
            if (esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0) {
                ESP_LOGI(TAG, "IPv4 ready on ETH_0");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            if (attempt == 119) {
                ESP_LOGW(TAG, "IPv4 address wait timeout (DHCP/static required for UDP PTP)");
            }
        }
    }
#endif
}

void app_main(void)
{
    init_ethernet_and_netif();

    // Check if Ethernet is connected before proceeding
    if (s_eth_port_cnt == 0 || !(xEventGroupGetBits(s_eth_event_group) & ETH_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "Ethernet not initialized or not connected, aborting");
        return;
    }

    esp_eth_clock_cfg_t clock_cfg = {
        .eth_hndl  = s_eth_handles[0],
#if CONFIG_NETUTILS_PTPD_IEEE_802_1AS
        .transport = DM9058_PTP_TRANSPORT_IEEE_802_3,
#elif CONFIG_EXAMPLE_PTP_TRANSPORT_IPV4
        .transport = DM9058_PTP_TRANSPORT_UDP_IPV4,
#else
        .transport = DM9058_PTP_TRANSPORT_IEEE_802_3,
#endif
    };
    ESP_LOGI(TAG, "PTP mode: slave, transport=%s, sync=%s",
#if CONFIG_NETUTILS_PTPD_IEEE_802_1AS
             "802.1AS",
#elif CONFIG_EXAMPLE_PTP_TRANSPORT_IPV4
             "UDP/IPv4",
#else
             "L2",
#endif
#if CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC
             "two-step");
#else
             "one-step");
#endif
    esp_eth_clock_init(CLOCK_PTP_SYSTEM, &clock_cfg);

#if CONFIG_EXAMPLE_PTP_TRANSPORT_IPV4
    ptpd_register_eth_handle(s_eth_handles[0]);
#endif
    ptpd_start("ETH_0");

    // Wait until PTP clock receives its first time-set from the master
    struct timespec cur_time = {0, 0};
    ESP_LOGI(TAG, "Waiting for PTP clock sync...");
    while (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &cur_time) == -1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "PTP clock available: %llu.%09lu",
             (unsigned long long)cur_time.tv_sec, (unsigned long)cur_time.tv_nsec);

    // DM9058 hardware PPS output is managed by the ptpd task
    // (ETH_MAC_DM9058_CMD_PPS_INIT / PPS_UPDATE called in ptp_daemon)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &cur_time);
        ESP_LOGI(TAG, "PTP time: %llu.%09lu",
                 (unsigned long long)cur_time.tv_sec, (unsigned long)cur_time.tv_nsec);
    }
}
