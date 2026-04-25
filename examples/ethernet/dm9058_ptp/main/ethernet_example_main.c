/* Ethernet Basic Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "ethernet_init.h"
#include "sdkconfig.h"

#if CONFIG_EXAMPLE_USE_PTP
#include "esp_eth_mac_spi.h"
#endif

static const char *TAG = "eth_example";

#if CONFIG_EXAMPLE_USE_PTP
static esp_eth_handle_t s_ptp_eth_handle = NULL;
static bool s_ptp_enabled = false;

static void ptp_task(void *arg)
{
    eth_dm9058_ptp_time_t ptp_time;

    while (1) {
        if (s_ptp_enabled && s_ptp_eth_handle) {
            if (esp_eth_ioctl(s_ptp_eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TIME, &ptp_time) == ESP_OK) {
                ESP_LOGI(TAG, "PTP Time: %"PRIu32".%09"PRIu32" s", ptp_time.seconds, ptp_time.nanoseconds);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_EXAMPLE_PTP_SYNC_INTERVAL_MS));
    }
}

static esp_err_t ptp_init(esp_eth_handle_t eth_handle)
{
    esp_err_t ret = ESP_OK;

    s_ptp_eth_handle = eth_handle;

    /* Set PTP transport type */
    eth_dm9058_ptp_transport_t transport;
#if CONFIG_EXAMPLE_PTP_TRANSPORT_L2
    transport = DM9058_PTP_TRANSPORT_IEEE_802_3;
    ESP_LOGI(TAG, "PTP Transport: Ethernet L2 (EtherType 0x88F7)");
#elif CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4
    transport = DM9058_PTP_TRANSPORT_UDP_IPV4;
    ESP_LOGI(TAG, "PTP Transport: UDP over IPv4");
#elif CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV6
    transport = DM9058_PTP_TRANSPORT_UDP_IPV6;
    ESP_LOGI(TAG, "PTP Transport: UDP over IPv6");
#else
    transport = DM9058_PTP_TRANSPORT_IEEE_802_3;
#endif

    ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_S_PTP_TRANSPORT, &transport);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PTP transport");
        return ret;
    }

    /* Enable PTP */
    bool enable = true;
    ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &enable);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PTP");
        return ret;
    }

    /* Set initial PTP time (use current timestamp or 0) */
    eth_dm9058_ptp_time_t init_time = {
        .seconds = 0,
        .nanoseconds = 0
    };
    ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_S_PTP_TIME, &init_time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial PTP time");
        return ret;
    }

    s_ptp_enabled = true;
    ESP_LOGI(TAG, "PTP initialized successfully");

    /* Create PTP monitoring task */
    xTaskCreate(ptp_task, "ptp_task", 4096, NULL, 5, NULL);

    return ESP_OK;
}
#endif /* CONFIG_EXAMPLE_USE_PTP */

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
#if CONFIG_EXAMPLE_USE_PTP
        if (!s_ptp_enabled) {
            ptp_init(eth_handle);
        }
#endif
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

void app_main(void)
{
    // Initialize Ethernet driver
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    ESP_ERROR_CHECK(example_eth_init(&eth_handles, &eth_port_cnt));

    // Initialize TCP/IP network interface aka the esp-netif (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *eth_netifs[eth_port_cnt];
    esp_eth_netif_glue_handle_t eth_netif_glues[eth_port_cnt];

    // Create instance(s) of esp-netif for Ethernet(s)
    if (eth_port_cnt == 1) {
        // Use ESP_NETIF_DEFAULT_ETH when just one Ethernet interface is used and you don't need to modify
        // default esp-netif configuration parameters.
        esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
        eth_netifs[0] = esp_netif_new(&cfg);
        eth_netif_glues[0] = esp_eth_new_netif_glue(eth_handles[0]);
        // Attach Ethernet driver to TCP/IP stack
        ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[0], eth_netif_glues[0]));
    } else {
        // Use ESP_NETIF_INHERENT_DEFAULT_ETH when multiple Ethernet interfaces are used and so you need to modify
        // esp-netif configuration parameters for each interface (name, priority, etc.).
        esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
        esp_netif_config_t cfg_spi = {
            .base = &esp_netif_config,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
        };
        char if_key_str[10];
        char if_desc_str[10];
        char num_str[3];
        for (int i = 0; i < eth_port_cnt; i++) {
            itoa(i, num_str, 10);
            strcat(strcpy(if_key_str, "ETH_"), num_str);
            strcat(strcpy(if_desc_str, "eth"), num_str);
            esp_netif_config.if_key = if_key_str;
            esp_netif_config.if_desc = if_desc_str;
            esp_netif_config.route_prio -= i*5;
            eth_netifs[i] = esp_netif_new(&cfg_spi);
            eth_netif_glues[i] = esp_eth_new_netif_glue(eth_handles[i]);
            // Attach Ethernet driver to TCP/IP stack
            ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[i], eth_netif_glues[i]));
        }
    }

    // Register user defined event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Start Ethernet driver state machine
    for (int i = 0; i < eth_port_cnt; i++) {
        ESP_ERROR_CHECK(esp_eth_start(eth_handles[i]));
    }

#if CONFIG_EXAMPLE_ETH_DEINIT_AFTER_S >= 0
    // For demonstration purposes, wait and then deinit Ethernet network
    vTaskDelay(pdMS_TO_TICKS(CONFIG_EXAMPLE_ETH_DEINIT_AFTER_S * 1000));
    ESP_LOGI(TAG, "stop and deinitialize Ethernet network...");
    // Stop Ethernet driver state machine and destroy netif
    for (int i = 0; i < eth_port_cnt; i++) {
        ESP_ERROR_CHECK(esp_eth_stop(eth_handles[i]));
        ESP_ERROR_CHECK(esp_eth_del_netif_glue(eth_netif_glues[i]));
        esp_netif_destroy(eth_netifs[i]);
    }
    esp_netif_deinit();
    ESP_ERROR_CHECK(example_eth_deinit(eth_handles, eth_port_cnt));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, got_ip_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler));
    ESP_ERROR_CHECK(esp_event_loop_delete_default());
#endif // EXAMPLE_ETH_DEINIT_AFTER_S > 0
}
