set(ptpd_srcs "ptpd.c")
set(ptpd_reqs esp_eth esp_netif esp_eth_time)

if(NOT CONFIG_EXAMPLE_PTP_TRANSPORT_L2)
    list(APPEND ptpd_srcs "ptpd_dm9058_port.c")
endif()

idf_component_register(SRCS ${ptpd_srcs}
                       PRIV_REQUIRES ${ptpd_reqs}
                       INCLUDE_DIRS "." "./include")

if(NOT CONFIG_EXAMPLE_PTP_TRANSPORT_L2)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP_PTP_DM9058_UDP_SLAVE=1)
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
