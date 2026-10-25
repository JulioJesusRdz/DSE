#include "bmp280.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
 
/* Configuracion I2C */
#define I2C_SDA_GPIO       21
#define I2C_SCL_GPIO       22
#define I2C_FREQUENCY      100000
 
#define BMP280_ADDRESS     0x77
#define BMP280_REG_ID      0xD0
#define BMP280_REG_RESET   0xE0
#define BMP280_REG_CTRL    0xF4
#define BMP280_REG_CONFIG  0xF5
#define BMP280_REG_PRESS   0xF7
#define BMP280_REG_CALIB   0x88
 
static const char *TAG = "BMP280";
 
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t bmp_handle;
 
/* Coeficientes de calibracion */
typedef struct {
    uint16_t T1;
    int16_t  T2, T3;
    uint16_t P1;
    int16_t  P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_calib_t;
 
static bmp280_calib_t calib;
static int32_t t_fine;
 
/* Leer uno o varios registros */
static esp_err_t read_registers(uint8_t reg, uint8_t *data,size_t length);
 
/* Escribir un registro */
static esp_err_t write_register(uint8_t reg, uint8_t value);
 
/* Convertir dos bytes little-endian a entero sin signo */
static uint16_t read_u16(const uint8_t *data);
 
/* Inicializar el bus I2C y registrar el sensor */
//esp_err_t i2c_init(void);
 
/* Leer los coeficientes almacenados en el sensor */
static esp_err_t read_calibration(void);
 
/* Inicializar y configurar el BMP280 */
esp_err_t bmp280_init_sensor(void);
 
/* Calcular temperatura a partir del ADC */
static float calculate_temperature(int32_t adc_T);
 
/* Calcular presion en Pa usando t_fine */
static float calculate_pressure(int32_t adc_P);
 
/* Leer temperatura y presion */
//static esp_err_t bmp280_read_sensor(void);
 
/* Enviar las mediciones por el puerto serie */
static void send_serial_data(float temperature, float pressure);
 
 
/*void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando BMP280 con I2C nativo");
 
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(bmp280_init_sensor());
 
     while (1) {
        esp_err_t error = bmp280_read_sensor();
 
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "Error de lectura: %s",
                     esp_err_to_name(error));
        }
 
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
 
 
}
*/
 
/* Leer uno o varios registros */
static esp_err_t read_registers(uint8_t reg,
                                uint8_t *data,
                                size_t length)
{
    return i2c_master_transmit_receive(
        bmp_handle, &reg, 1, data, length, 1000
    );
}
 
/* Escribir un registro */
static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
 
    return i2c_master_transmit(
        bmp_handle, data, sizeof(data), 1000
    );
}
 
/* Convertir dos bytes little-endian a entero sin signo */
static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] |
           ((uint16_t)data[1] << 8);
}
 
 
/* Inicializar el bus I2C y registrar el sensor */
esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
 
    ESP_ERROR_CHECK(
        i2c_new_master_bus(&bus_config, &bus_handle)
    );
 
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_ADDRESS,
        .scl_speed_hz = I2C_FREQUENCY,
    };
 
    return i2c_master_bus_add_device(
        bus_handle, &device_config, &bmp_handle
    );
}
 
 
/* Leer los coeficientes almacenados en el sensor */
static esp_err_t read_calibration(void)
{
    uint8_t data[24];
 
    esp_err_t error = read_registers(
        BMP280_REG_CALIB, data, sizeof(data)
    );
 
    if (error != ESP_OK) {
        return error;
    }
 
    calib.T1 = read_u16(&data[0]);
    calib.T2 = (int16_t)read_u16(&data[2]);
    calib.T3 = (int16_t)read_u16(&data[4]);
 
    calib.P1 = read_u16(&data[6]);
    calib.P2 = (int16_t)read_u16(&data[8]);
    calib.P3 = (int16_t)read_u16(&data[10]);
    calib.P4 = (int16_t)read_u16(&data[12]);
    calib.P5 = (int16_t)read_u16(&data[14]);
    calib.P6 = (int16_t)read_u16(&data[16]);
    calib.P7 = (int16_t)read_u16(&data[18]);
    calib.P8 = (int16_t)read_u16(&data[20]);
    calib.P9 = (int16_t)read_u16(&data[22]);
 
    if (calib.P1 == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
 
    return ESP_OK;
}
 
/* Inicializar y configurar el BMP280 */
esp_err_t bmp280_init_sensor(void)
{
    uint8_t id;
 
    ESP_RETURN_ON_ERROR(
        read_registers(BMP280_REG_ID, &id, 1),
        TAG, "No se pudo leer el ID"
    );
 
    if (id != 0x58) {
        ESP_LOGE(TAG, "ID incorrecto: 0x%02X (esperado 0x58)", id);
        return ESP_ERR_INVALID_VERSION;
    }
 
    ESP_LOGI(TAG, "BMP280 detectado: ID=0x%02X", id);
 
    ESP_RETURN_ON_ERROR(
        write_register(BMP280_REG_RESET, 0xB6),
        TAG, "Error al reiniciar el sensor"
    );
 
    vTaskDelay(pdMS_TO_TICKS(10));
 
    ESP_RETURN_ON_ERROR(
        read_calibration(),
        TAG, "Error al leer la calibracion"
    );
 
    /* Filtro desactivado y espera de 1 segundo */
    ESP_RETURN_ON_ERROR(
        write_register(BMP280_REG_CONFIG, 0xA0),
        TAG, "Error al configurar el sensor"
    );
 
    /* Temperatura x1, presion x1, modo normal */
    ESP_RETURN_ON_ERROR(
        write_register(BMP280_REG_CTRL, 0x25),
        TAG, "Error al iniciar las mediciones"
    );
 
    return ESP_OK;
}
 
/* Calcular temperatura a partir del ADC */
static float calculate_temperature(int32_t adc_T)
{
    int32_t var1, var2;
 
    var1 = ((((adc_T >> 3) -
             ((int32_t)calib.T1 << 1))) *
             (int32_t)calib.T2) >> 11;
 
    var2 = (((((adc_T >> 4) -
             (int32_t)calib.T1) *
             ((adc_T >> 4) -
             (int32_t)calib.T1)) >> 12) *
             (int32_t)calib.T3) >> 14;
 
    t_fine = var1 + var2;
 
    return (float)((t_fine * 5 + 128) >> 8) / 100.0f;
}
 
/* Calcular presion en Pa usando t_fine */
static float calculate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;
 
    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * calib.P6;
    var2 += (var1 * calib.P5) << 17;
    var2 += ((int64_t)calib.P4) << 35;
 
    var1 = ((var1 * var1 * calib.P3) >> 8) +
           ((var1 * calib.P2) << 12);
 
    var1 = ((((int64_t)1 << 47) + var1) *
            calib.P1) >> 33;
 
    if (var1 == 0) {
        return -1.0f;
    }
 
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
 
    var1 = ((int64_t)calib.P9 * (p >> 13) *
            (p >> 13)) >> 25;
 
    var2 = ((int64_t)calib.P8 * p) >> 19;
 
    p = ((p + var1 + var2) >> 8) +
        ((int64_t)calib.P7 << 4);
 
    /* El resultado esta en Pa con 8 bits fraccionarios */
    return (float)p / 256.0f;
}
 
/* Leer temperatura y presion */
esp_err_t bmp280_read_sensor(float *temperature, float *pressure)
{
    uint8_t data[6];

    ESP_RETURN_ON_ERROR(
        read_registers(BMP280_REG_PRESS, data, sizeof(data)),
        TAG, "Error al leer las mediciones"
    );

    int32_t adc_P = ((int32_t)data[0] << 12) |
                    ((int32_t)data[1] << 4) |
                    (data[2] >> 4);

    int32_t adc_T = ((int32_t)data[3] << 12) |
                    ((int32_t)data[4] << 4) |
                    (data[5] >> 4);

    float t = calculate_temperature(adc_T);   /* primero la temperatura, calcula t_fine */
    float p = calculate_pressure(adc_P);

    if (p < 0.0f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *temperature = t;
    *pressure = p;   /* en Pa; main.c ya divide entre 100 */

    return ESP_OK;
}
 
/* Enviar las mediciones por el puerto serie */
static void send_serial_data(float temperature, float pressure)
{
    printf("DATA,%.2f,%.2f\n",
           (double)temperature,
           (double)pressure);
}