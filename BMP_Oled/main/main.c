#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* Incluimos los headers de los componentes */
#include "bmp280.h" 
#include "oled.h"

static const char *TAG = "MAIN";

void app_main(void)
{
 
    ESP_LOGI(TAG, "Iniciando BMP280 con I2C nativo");
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(bmp280_init_sensor());
 
    /* Inicializar la pantalla */
    oled_init();
    /* Borrar el contenido */
    oled_clear();
 
    /* Escribir mensajes en diferentes renglones */
    oled_write_text(0, 0, "ESP32 SPI");
    oled_write_text(2, 0, "HOLA DOC");
    oled_write_text(4, 0, "BMP280 LISTO");
 
    vTaskDelay(pdMS_TO_TICKS(3000));
 
while (1)
{
    float temperature = 0.0f;
    float pressure = 0.0f;
 
    esp_err_t error = bmp280_read_sensor(&temperature, &pressure);
 
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Error de lectura: %s",
                 esp_err_to_name(error));
    } else {
        // La presión se convierte de Pa a hPa.
        update_oled(temperature, pressure / 100.0f);
    }
 
    vTaskDelay(pdMS_TO_TICKS(2000));
}
 
}
