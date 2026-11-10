/*
 * ADC One-shot a 10 kHz.
 *
 * La tasa se controla con un GPTimer (alarma cada 100 us). Su ISR solo envia una
 * notificacion a la tarea de adquisicion; la lectura adc_oneshot_read() se hace
 * en la tarea (no es segura en ISR). Si llegan 2 o mas notificaciones juntas, la
 * tarea fue demasiado lenta y se cuentan como muestras perdidas.
 *
 * Nunca se imprime desde esta tarea: la UART la maneja Experiment, asi el
 * texto no altera la temporizacion del muestreo.
 */
#include "acq_oneshot.h"
#include "acq_common.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define POT_CANAL        ADC_CHANNEL_6     /* GPIO34 */
#define TAREA_PRIORIDAD  20
#define TAREA_NUCLEO     1
#define TAREA_STACK      4096

static const char *TAG = "ACQ_OS";

static adc_oneshot_unit_handle_t adc = NULL;
static gptimer_handle_t timer = NULL;
static TaskHandle_t tarea = NULL;
static SemaphoreHandle_t fin = NULL;
static volatile bool activa = false;

static bool IRAM_ATTR on_alarma(gptimer_handle_t t, const gptimer_alarm_event_data_t *e, void *ctx)
{
    BaseType_t despertar = pdFALSE;
    vTaskNotifyGiveFromISR(tarea, &despertar);
    return despertar == pdTRUE;
}

static void tarea_adquisicion(void *arg)
{
    uint16_t blk[ADC_BLOCK_SAMPLES];
    size_t n = 0;
    uint32_t perdidas = 0;
    uint32_t errores = 0;
    uint32_t busy = 0;

    while (activa) {
        uint32_t ticks = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (ticks == 0) {
            continue;                       /* sin tick (por ejemplo, al detener) */
        }

        int64_t t0 = esp_timer_get_time();
        if (ticks > 1) {
            perdidas += ticks - 1;          /* ticks atrasados = muestras perdidas */
        }

        int raw = 0;
        if (adc_oneshot_read(adc, POT_CANAL, &raw) == ESP_OK) {
            blk[n++] = (uint16_t)raw;
        } else {
            errores++;
        }
        busy += (uint32_t)(esp_timer_get_time() - t0);

        if (n == ADC_BLOCK_SAMPLES) {
            acq_common_block_done(blk, n, perdidas, errores, 0, busy);
            n = 0;
            perdidas = 0;
            errores = 0;
            busy = 0;
        }
    }

    xSemaphoreGive(fin);
    vTaskDelete(NULL);
}

void acq_oneshot_stop(void)
{
    if (timer != NULL) {
        gptimer_stop(timer);
        gptimer_disable(timer);
        gptimer_del_timer(timer);
        timer = NULL;
    }
    if (tarea != NULL) {
        activa = false;
        xSemaphoreTake(fin, pdMS_TO_TICKS(500));
        tarea = NULL;
    }
    if (fin != NULL) {
        vSemaphoreDelete(fin);
        fin = NULL;
    }
    if (adc != NULL) {
        adc_oneshot_del_unit(adc);
        adc = NULL;
    }
}

esp_err_t acq_oneshot_start(void)
{
    esp_err_t err;
    acq_common_reset();

    /* ADC1, 12 bits, atenuacion 12 dB (rango aprox. 0 - 3.1 V) */
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    err = adc_oneshot_new_unit(&unit_cfg, &adc);
    if (err != ESP_OK) goto fallo;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    err = adc_oneshot_config_channel(adc, POT_CANAL, &chan_cfg);
    if (err != ESP_OK) goto fallo;

    fin = xSemaphoreCreateBinary();
    if (fin == NULL) { err = ESP_ERR_NO_MEM; goto fallo; }

    activa = true;
    if (xTaskCreatePinnedToCore(tarea_adquisicion, "acq_oneshot", TAREA_STACK, NULL,
                                TAREA_PRIORIDAD, &tarea, TAREA_NUCLEO) != pdPASS) {
        activa = false;
        tarea = NULL;
        err = ESP_ERR_NO_MEM;
        goto fallo;
    }

    /* GPTimer a 1 MHz; alarma cada 1e6 / 10000 = 100 ticks = 100 us */
    gptimer_config_t t_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    err = gptimer_new_timer(&t_cfg, &timer);
    if (err != ESP_OK) goto fallo;

    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarma };
    err = gptimer_register_event_callbacks(timer, &cbs, NULL);
    if (err != ESP_OK) goto fallo;

    gptimer_alarm_config_t alarma = {
        .alarm_count = 1000000 / ADC_FS_EFECTIVA_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    err = gptimer_set_alarm_action(timer, &alarma);
    if (err != ESP_OK) goto fallo;

    err = gptimer_enable(timer);
    if (err != ESP_OK) goto fallo;
    err = gptimer_start(timer);
    if (err != ESP_OK) goto fallo;

    ESP_LOGI(TAG, "One-shot iniciado: %d Hz, bloques de %d muestras", ADC_FS_EFECTIVA_HZ, ADC_BLOCK_SAMPLES);
    return ESP_OK;

fallo:
    ESP_LOGE(TAG, "Error al iniciar One-shot: %s", esp_err_to_name(err));
    acq_oneshot_stop();
    return err;
}
