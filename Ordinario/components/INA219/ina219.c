#include "ina219.h"
#include <stdint.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------------- Pines y direccion ---------------- */
#define I2C_SDA_GPIO      21
#define I2C_SCL_GPIO      22
#define I2C_FREQ_HZ       100000      /* 100 kHz: lecturas de 1 Hz, mayor margen de ruido */
#define I2C_TIMEOUT_MS    100
#define INA219_ADDR       0x40        /* A0 = A1 = GND */

/* ---------------- Registros ---------------- */
#define REG_CONFIG        0x00
#define REG_SHUNT         0x01
#define REG_BUS           0x02
#define REG_POWER         0x03
#define REG_CURRENT       0x04
#define REG_CALIB         0x05

/*
 * Configuracion (registro 0x00):
 *   BRNG = 0      -> rango de bus de 16 V (la alimentacion es de 5 V)
 *   PG   = 01     -> PGA /2: +-80 mV en el shunt = +-800 mA con 0.1 ohm
 *   BADC = 1111   -> 128 muestras promediadas (68.1 ms) para el voltaje de bus
 *   SADC = 1111   -> 128 muestras promediadas (68.1 ms) para el voltaje del shunt
 *   MODE = 111    -> shunt y bus en modo continuo
 */
#define CONFIG_VALUE      0x0FFF
#define CONFIG_RESET      0x8000

/*
 * Calibracion:
 *   R_shunt = 0.1 ohm (resistencia "R100" del modulo)
 *   Current_LSB = 0.1 mA  (coincide con el LSB del shunt: 10 uV / 0.1 ohm)
 *   Cal = trunc(0.04096 / (Current_LSB x R_shunt)) = 0.04096 / (1e-4 x 0.1) = 4096
 *   Power_LSB = 20 x Current_LSB = 2 mW
 */
#define SHUNT_OHMS        0.1f
#define CALIB_VALUE       4096
#define CURRENT_LSB_MA    0.1f
#define POWER_LSB_MW      2.0f
#define BUS_LSB_V         0.004f      /* 4 mV por LSB */

/* Limites de plausibilidad de la alimentacion de 5 V */
#define V_MIN_V           2.5f
#define V_MAX_V           6.0f
#define I_MIN_MA          0.0f
#define I_MAX_MA          800.0f      /* fondo de escala del PGA /2 */

static const char *TAG = "INA219";

static i2c_master_bus_handle_t bus = NULL;
static i2c_master_dev_handle_t dev = NULL;

static esp_err_t escribir16(uint8_t reg, uint16_t valor)
{
    uint8_t buf[3] = { reg, (uint8_t)(valor >> 8), (uint8_t)(valor & 0xFF) };
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t leer16(uint8_t reg, uint16_t *valor)
{
    uint8_t buf[2] = {0, 0};
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        *valor = (uint16_t)((buf[0] << 8) | buf[1]);
    }
    return err;
}

esp_err_t ina219_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* ademas de los pull-ups del modulo */
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG, "Error al crear el bus I2C");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA219_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &dev), TAG,
                        "Error al registrar el INA219 (direccion 0x40)");

    ESP_RETURN_ON_ERROR(escribir16(REG_CONFIG, CONFIG_RESET), TAG, "El INA219 no responde");
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_RETURN_ON_ERROR(escribir16(REG_CONFIG, CONFIG_VALUE), TAG, "Error al configurar");
    ESP_RETURN_ON_ERROR(escribir16(REG_CALIB, CALIB_VALUE), TAG, "Error al calibrar");

    /* Verificacion por lectura de vuelta */
    uint16_t cfg = 0, cal = 0;
    ESP_RETURN_ON_ERROR(leer16(REG_CONFIG, &cfg), TAG, "No se pudo leer CONFIG");
    ESP_RETURN_ON_ERROR(leer16(REG_CALIB, &cal), TAG, "No se pudo leer CALIB");
    if (cfg != CONFIG_VALUE || cal != CALIB_VALUE) {
        ESP_LOGE(TAG, "Verificacion fallida: CONFIG=0x%04X (esperado 0x%04X) CALIB=%u (esperado %u)",
                 cfg, CONFIG_VALUE, cal, CALIB_VALUE);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t ina219_read(ina219_data_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "Puntero nulo");

    uint16_t bus_reg = 0, cur_reg = 0, pow_reg = 0;
    ESP_RETURN_ON_ERROR(leer16(REG_BUS, &bus_reg), TAG, "Error leyendo voltaje de bus");
    ESP_RETURN_ON_ERROR(leer16(REG_CURRENT, &cur_reg), TAG, "Error leyendo corriente");
    ESP_RETURN_ON_ERROR(leer16(REG_POWER, &pow_reg), TAG, "Error leyendo potencia");

    /* Bit 0 (OVF): desbordamiento matematico, el dato no es confiable */
    if (bus_reg & 0x0001) {
        ESP_LOGW(TAG, "OVF activo (BUS=0x%04X, CUR=0x%04X)", bus_reg, cur_reg);
        return ESP_ERR_INVALID_RESPONSE;
    }

    float v = (float)(bus_reg >> 3) * BUS_LSB_V;
    float i = (float)(int16_t)cur_reg * CURRENT_LSB_MA;

    if (v < V_MIN_V || v > V_MAX_V || i < I_MIN_MA || i > I_MAX_MA) {
        ESP_LOGW(TAG, "Lectura rechazada: V=%.3f V  I=%.2f mA (aceptado: %.1f-%.1f V, %.0f-%.0f mA)",
                 (double)v, (double)i, (double)V_MIN_V, (double)V_MAX_V,
                 (double)I_MIN_MA, (double)I_MAX_MA);
        return ESP_ERR_INVALID_RESPONSE;
    }

    out->v_bus_V = v;
    out->i_mA = i;
    out->p_mW = v * i;                              /* V x mA = mW */
    out->p_reg_mW = (float)pow_reg * POWER_LSB_MW;
    return ESP_OK;
}

void ina219_log_config(void)
{
    ESP_LOGI(TAG, "Direccion I2C: 0x%02X | SDA=GPIO%d SCL=GPIO%d | %u kHz",
             INA219_ADDR, I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ / 1000);
    ESP_LOGI(TAG, "Shunt = %.2f ohm | CONFIG=0x%04X (16 V, PGA/2 +-80 mV, 128 muestras)",
             (double)SHUNT_OHMS, CONFIG_VALUE);
    ESP_LOGI(TAG, "CALIB=%u | Current_LSB=%.1f mA | Power_LSB=%.1f mW | Bus_LSB=%.0f mV",
             CALIB_VALUE, (double)CURRENT_LSB_MA, (double)POWER_LSB_MW, (double)(BUS_LSB_V * 1000.0f));
}
