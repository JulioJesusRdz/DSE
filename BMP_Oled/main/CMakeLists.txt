idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES Oled BMP280 esp_driver_i2c esp_driver_spi esp_driver_gpio)