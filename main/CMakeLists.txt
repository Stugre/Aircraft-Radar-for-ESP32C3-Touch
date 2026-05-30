idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_timer
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_http_client
        esp_http_server
        esp-tls
        mbedtls
)
