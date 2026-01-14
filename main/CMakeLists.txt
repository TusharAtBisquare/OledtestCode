idf_component_register(SRCS "i2c_oled_example_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_i2c esp_driver_gpio u8g2 u8g2-hal-esp-idf)