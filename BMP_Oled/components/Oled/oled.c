#include "oled.h"
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
 
/* Pines de la pantalla OLED */
#define OLED_SCLK   GPIO_NUM_18
#define OLED_MOSI   GPIO_NUM_23
#define OLED_RES    GPIO_NUM_17
#define OLED_DC     GPIO_NUM_16
#define OLED_CS     GPIO_NUM_19
 
/* Configuracion de la pantalla */
#define OLED_WIDTH   128
#define OLED_PAGES   8
 
#define SPI_FREQUENCY  8000000
 
static spi_device_handle_t oled_spi;
 
/* Prototipos de funciones */
void oled_init(void);
static void oled_reset(void);
static void oled_command(uint8_t command);
static void oled_data(uint8_t data);
static void oled_set_position(uint8_t page, uint8_t column);
void oled_clear(void);
void oled_write_text(uint8_t page, uint8_t column, const char *text);
static const uint8_t *get_glyph(char character);
 
/* Fuente 5x7: letras A-Z y numeros 0-9 */
static const uint8_t font[][5] = {
    /* A-Z */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
 
    /* 0-9 */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}  /* 9 */
};
 
//void app_main(void)
//{
    /* Inicializar la pantalla */
//    oled_init();
 
    /* Borrar el contenido */
//    oled_clear();
 
    /* Escribir mensajes en diferentes renglones */
//   oled_write_text(0, 0, "ESP32 SPI");
//    oled_write_text(2, 0, "HOLA DOC");
//    oled_write_text(4, 0, "BMP280 LISTO");
 
//    while (1) {
 
//        vTaskDelay(pdMS_TO_TICKS(1000));
//    }
 
//}

    void update_oled(float temperature, float pressure)
{
    char line[24];
 
    // Primera línea: ESP32 SPI
    oled_write_text(0, 0, "ESP32 SPI");
 
    // Segunda línea: DSE
    oled_write_text(2, 0, "DSE");
 
    // Tercera línea: temperatura
    snprintf(line, sizeof(line), "TEMPERATURA=%.1fC",
             (double)temperature);
    oled_write_text(4, 0, line);
 
    // Cuarta línea: presión en hPa
    snprintf(line, sizeof(line), "PRESION=%.1f HPA",
         (double)pressure);
 
    oled_write_text(6, 0, line);
}
 
/* Configurar el bus SPI y las señales de control */
void oled_init(void)
{
    gpio_config_t gpio_configuracion = {
        .pin_bit_mask =
            (1ULL << OLED_RES) |
            (1ULL << OLED_DC),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
 
    ESP_ERROR_CHECK(gpio_config(&gpio_configuracion));
 
    spi_bus_config_t bus_config = {
        .mosi_io_num = OLED_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = OLED_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = OLED_WIDTH
    };
 
    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI2_HOST,
                           &bus_config,
                           SPI_DMA_DISABLED)
    );
 
    spi_device_interface_config_t dispositivo_config = {
        .clock_speed_hz = SPI_FREQUENCY,
        .mode = 0,
        .spics_io_num = OLED_CS,
        .queue_size = 1
    };
 
    ESP_ERROR_CHECK(
        spi_bus_add_device(SPI2_HOST,
                           &dispositivo_config,
                           &oled_spi)
    );
 
    oled_reset();
 
    /* Secuencia de inicializacion SSD1306 */
    oled_command(0xAE); /* Pantalla apagada */
    oled_command(0xD5);
    oled_command(0x80); /* Reloj */
    oled_command(0xA8);
    oled_command(0x3F); /* Multiplex 64 filas */
    oled_command(0xD3);
    oled_command(0x00); /* Desplazamiento */
    oled_command(0x40); /* Inicio de pantalla */
    oled_command(0x8D);
    oled_command(0x14); /* Bomba de carga */
    oled_command(0x20);
    oled_command(0x02); /* Direccionamiento por paginas */
    oled_command(0xA1); /* Orientacion horizontal */
    oled_command(0xC8); /* Orientacion vertical */
    oled_command(0xDA);
    oled_command(0x12); /* Configuracion de pines */
    oled_command(0x81);
    oled_command(0x7F); /* Contraste */
    oled_command(0xD9);
    oled_command(0xF1); /* Pre-carga */
    oled_command(0xDB);
    oled_command(0x40); /* Nivel VCOMH */
    oled_command(0xA4); /* Mostrar RAM */
    oled_command(0xA6); /* Imagen normal */
    oled_command(0xAF); /* Pantalla encendida */
}
 
/* Reiniciar fisicamente el controlador */
static void oled_reset(void)
{
    gpio_set_level(OLED_RES, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
 
    gpio_set_level(OLED_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}
 
/* Enviar un comando: DC = 0 */
static void oled_command(uint8_t command)
{
    spi_transaction_t transaccion = {
        .length = 8,
        .tx_buffer = &command
    };
 
    gpio_set_level(OLED_DC, 0);
 
    ESP_ERROR_CHECK(
        spi_device_transmit(oled_spi, &transaccion)
    );
}
 
/* Enviar un byte de imagen: DC = 1 */
static void oled_data(uint8_t data)
{
    spi_transaction_t transaccion = {
        .length = 8,
        .tx_buffer = &data
    };
 
    gpio_set_level(OLED_DC, 1);
 
    ESP_ERROR_CHECK(
        spi_device_transmit(oled_spi, &transaccion)
    );
}
 
/* Seleccionar pagina y columna inicial */
static void oled_set_position(uint8_t page, uint8_t column)
{
    oled_command(0xB0 | page);
    oled_command(0x00 | (column & 0x0F));
    oled_command(0x10 | (column >> 4));
}
 
/* Borrar todos los pixeles */
void oled_clear(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_set_position(page, 0);
 
        for (uint8_t column = 0;
             column < OLED_WIDTH;
             column++) {
            oled_data(0x00);
        }
    }
}
 
/* Obtener el patron de una letra o numero */
/* Obtener el patron de una letra o numero */
static const uint8_t *get_glyph(char character)
{
    static const uint8_t espacio[5] = {0, 0, 0, 0, 0};

    /* Caracteres especiales */
    static const uint8_t punto[5]   = {0x00, 0x60, 0x60, 0x00, 0x00}; /* . */
    static const uint8_t igual[5]   = {0x14, 0x14, 0x14, 0x14, 0x14}; /* = */
    static const uint8_t porcent[5] = {0x23, 0x13, 0x08, 0x64, 0x62}; /* % */
    static const uint8_t guion[5]   = {0x08, 0x08, 0x08, 0x08, 0x08}; /* - */
    static const uint8_t dospuntos[5] = {0x00, 0x36, 0x36, 0x00, 0x00}; /* : */

    /* Revisar los especiales antes de convertir a mayusculas */
    switch (character) {
        case '.': return punto;
        case '=': return igual;
        case '%': return porcent;
        case '-': return guion;
        case ':': return dospuntos;
        default:  break;
    }

    character = (char)toupper((unsigned char)character);

    if (character >= 'A' && character <= 'Z') {
        return font[character - 'A'];
    }

    if (character >= '0' && character <= '9') {
        return font[26 + character - '0'];
    }

    return espacio;
}
 
/* Escribir una palabra o frase en una pagina */
void oled_write_text(uint8_t page,
                            uint8_t column,
                            const char *text)
{
    if (page >= OLED_PAGES || column >= OLED_WIDTH) {
        return;
    }
 
    oled_set_position(page, column);
 
    while (*text != '\0' && column < OLED_WIDTH) {
        const uint8_t *glyph = get_glyph(*text);
 
        /* Cinco columnas forman cada caracter */
        for (int i = 0; i < 5 && column < OLED_WIDTH; i++) {
            oled_data(glyph[i]);
            column++;
        }
 
        /* Separacion entre caracteres */
        if (column < OLED_WIDTH) {
            oled_data(0x00);
            column++;
        }
 
        text++;
    }
}