idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_event
        esp_http_server
        esp_netif
        esp_timer
        lwip
        json
        mdns
        esp_driver_i2c
        esp_driver_gpio
        u8g2
        u8g2-hal-esp-idf
)
