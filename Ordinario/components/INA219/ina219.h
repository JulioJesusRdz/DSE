#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float v_bus_V;      /* tension del lado de la carga (VIN-), en V   */
    float i_mA;         /* corriente, en mA                            */
    float p_mW;         /* potencia = V x I calculada en firmware, mW  */
    float p_reg_mW;     /* potencia del registro del INA219 (control)  */
} ina219_data_t;

/* Crea el bus I2C (SDA 21, SCL 22), reinicia y calibra el INA219. */
esp_err_t ina219_init(void);

/* Lee y valida una medicion. ESP_ERR_INVALID_RESPONSE si no es plausible. */
esp_err_t ina219_read(ina219_data_t *out);

/* Imprime direccion, shunt, calibracion y LSB usados (para el reporte). */
void ina219_log_config(void);
