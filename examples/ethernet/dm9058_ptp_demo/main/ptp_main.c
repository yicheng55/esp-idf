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
#include "driver/gpio.h"
#include "freertos/event_groups.h"
#include "ptpd.h"

#include "esp_eth_time.h"
#if CONFIG_EXAMPLE_PTP_TRANSPORT_IPV4
#include "esp_netif.h"
#endif

static const char *TAG = "ptp_example";

#define ETH_CONNECTED_BIT BIT0

static EventGroupHandle_t s_eth_event_group;

static struct timespec s_next_time;
static bool s_gpio_level;
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

static bool ts_callback(esp_eth_mediator_t *eth, void *user_args)
{
    (void)eth;
    (void)user_args;
    gpio_set_level(CONFIG_EXAMPLE_PTP_PULSE_GPIO, s_gpio_level ^= 1);

    struct timespec interval = {
        .tv_sec = 0,
        .tv_nsec = CONFIG_EXAMPLE_PTP_PULSE_WIDTH_NS
    };
    timespecadd(&s_next_time, &interval, &s_next_time);

    struct timespec curr_time;
    esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &curr_time);
    if (timespeccmp(&s_next_time, &curr_time, >)) {
        esp_eth_clock_set_target_time(CLOCK_PTP_SYSTEM, &s_next_time);
    }

    ESP_LOGD(TAG, "PTP pulse: curr %llu.%09lu next %llu.%09lu",
             (unsigned long long)curr_time.tv_sec, (unsigned long)curr_time.tv_nsec,
             (unsigned long long)s_next_time.tv_sec, (unsigned long)s_next_time.tv_nsec);
    return false;
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
    int pid = ptpd_start("ETH_0");

    struct timespec cur_time = {0, 0};
    // wait for the clock to be available
    ESP_LOGI(TAG, "init.s curr time: %llu.%09lu", cur_time.tv_sec, cur_time.tv_nsec);

    while (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &cur_time) == -1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "init.e curr time: %llu.%09lu", cur_time.tv_sec, cur_time.tv_nsec);

    if (esp_eth_clock_register_target_cb(CLOCK_PTP_SYSTEM, ts_callback) != 0) {
        ESP_LOGE(TAG, "esp_eth_clock_register_target_cb failed");
        return;
    }

    // initialize output pin
    gpio_config_t gpio_out_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_EXAMPLE_PTP_PULSE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpio_out_cfg);
    gpio_set_level(CONFIG_EXAMPLE_PTP_PULSE_GPIO, 0);

    bool first_pass = true;
    bool clock_source_valid = false;
    bool clock_source_valid_last = false;
    int32_t clock_source_valid_cnt = 0;
    while (1) {
        struct ptpd_status_s ptp_status;
        // if valid PTP status
        if (ptpd_status(pid, &ptp_status) == 0) {
            if (ptp_status.clock_source_valid) {
                clock_source_valid_cnt++;
            } else {
                clock_source_valid_cnt = 0;
            }
        } else {
            clock_source_valid_cnt = 0;
        }
        // consider the source valid only after n consequent intervals to be sure clock was synced
        if (clock_source_valid_cnt > 2) {
            clock_source_valid = true;
        } else {
            clock_source_valid = false;
        }
        // source validity changed => resync the pulse for ptp slave OR when the first pass to PTP master
        // starts generating its pulses
        if ((clock_source_valid == true && clock_source_valid_last == false) || first_pass) {
            first_pass = false;
            // get the most recent (now synced) time
            esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &cur_time);
            // compute the next target time
            s_next_time.tv_sec = 1;
            timespecadd(&s_next_time, &cur_time, &s_next_time);
            s_next_time.tv_nsec = CONFIG_EXAMPLE_PTP_PULSE_WIDTH_NS;
            ESP_LOGI(TAG, "PTP slave locked: source_valid=%d cnt=%" PRId32, clock_source_valid, clock_source_valid_cnt);
            ESP_LOGI(TAG, "Starting Pulse train");
            ESP_LOGI(TAG, "curr time: %llu.%09lu", cur_time.tv_sec, cur_time.tv_nsec);
            ESP_LOGI(TAG, "next time: %llu.%09lu", s_next_time.tv_sec, s_next_time.tv_nsec);
            s_gpio_level = 0;
            gpio_set_level(CONFIG_EXAMPLE_PTP_PULSE_GPIO, s_gpio_level);
            esp_eth_clock_set_target_time(CLOCK_PTP_SYSTEM, &s_next_time);
        }
        clock_source_valid_last = clock_source_valid;
    }
}
