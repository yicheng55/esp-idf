idf_component_register(SRCS "ethernet_example_main.c"
                       PRIV_REQUIRES esp_netif esp_eth esp_eth_time ethernet_init driver ptpd
                       INCLUDE_DIRS ".")
