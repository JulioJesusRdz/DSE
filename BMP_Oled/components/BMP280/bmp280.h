#pragma once
#include "esp_err.h"

esp_err_t i2c_init(void);
esp_err_t bmp280_init_sensor(void);
esp_err_t bmp280_read_sensor(float *temperature, float *pressure);