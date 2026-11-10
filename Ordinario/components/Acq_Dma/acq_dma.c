/*
 * ADC Continuous (DMA).
 *
 * El ESP32 solo admite de 20 kHz a 2 MHz en modo continuo, por lo que el
 * objetivo de 10 kHz del documento no puede configurarse directamente. Se
 * muestrea a 20 kHz por hardware y se promedian pares de muestras consecutivas
 * en firmware (decimacion 2:1) para obtener 10 kHz EFECTIVOS, igual que el
 * metodo One-shot. Esa decimacion es una diferencia inevitable y se documenta.
 *
 * El DMA llena el pool con tramas de 512 bytes (256 muestras de hardware). El
 * callback on_conv_done solo notifica a la tarea; el procesamiento se hace en
 * la tarea, no en la ISR. on_pool_ovf cuenta los desbordamientos del pool.
 */
#include "acq_dma.h"
#include "acq_common.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define POT_CANAL         ADC_CHANNEL_6        /* GPIO34 */
#define DMA_FS_HW_HZ      20000                /* minimo del ESP32 en modo continuo */
#define DMA_DECIMACION    (DMA_FS_HW_HZ / ADC_FS_EFECTIVA_HZ)
#define TRAMA_BYTES       512                  /* 256 muestras de hardware (12.8 ms) */
#define POOL_BYTES        4096                 /* 8 tramas = ~102 ms de reserva      */
#define TAREA_PRIORIDAD   20
#define TAREA_NUCLEO      1
#define TAREA_STACK       4096

_Static_assert((DMA_FS_HW_HZ % ADC_FS_EFECTIVA_HZ) == 0, "La frecuencia de hardware debe ser multiplo de la efectiva");

static const char *TAG = "ACQ_DMA";

static adc_continuous_handle_t adc = NULL;
static TaskHandle_t tarea = NULL;
static SemaphoreHandle_t fin = NULL;
static volatile bool activa = false;
static volatile uint32_t ovf_isr = 0;

static uint8_t raw[TRAMA_BYTES];
static adc_continuous_data_t parseadas[TRAMA_BYTES / SOC_ADC_DIGI_RESULT_BYTES];

static bool IRAM_ATTR cb_conv_done(adc_continuous_handle_t h, const adc_continuous_evt_data_t *e, void *u)
{
    BaseType_t despertar = pdFALSE;
    vTaskNotifyGiveFromISR(tarea, &despertar);
    return despertar == pdTRUE;
}

static bool IRAM_ATTR cb_pool_ovf(adc_continuous_handle_t h, const adc_continuous_evt_data_t *e, void *u)
{
    ovf_isr++;
    return false;
}

static void tarea_adquisicion(void *arg)
{
    uint16_t blk[ADC_BLOCK_SAMPLES];
    size_t n = 0;
    uint32_t err = 0;
    uint32_t busy = 0;
    uint32_t ovf_prev = 0;
    uint32_t acc = 0;
    uint32_t k = 0;

    while (activa) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));      /* despierta con cada trama */

        while (activa) {
            uint32_t nbytes = 0;
            esp_err_t r = adc_continuous_read(adc, raw, sizeof(raw), &nbytes, 0);
            if (r == ESP_ERR_TIMEOUT) {
                break;                                      /* ya no hay datos */
            }

            int64_t t0 = esp_timer_get_time();
            if (r != ESP_OK) {
                err++;                      /* ESP_ERR_INVALID_STATE: el pool se desbordo */
            }
            nbytes -= nbytes % SOC_ADC_DIGI_RESULT_BYTES;
            if (nbytes == 0) {
                break;
            }

            uint32_t np = 0;
            if (adc_continuous_parse_data(adc, raw, nbytes, parseadas, &np) != ESP_OK) {
                err++;
                continue;
            }

            for (uint32_t i = 0; i < np; i++) {
                if (!parseadas[i].valid) {
                    err++;
                    continue;
                }
                acc += parseadas[i].raw_data;
                if (++k < DMA_DECIMACION) {
                    continue;
                }
                blk[n++] = (uint16_t)(acc / DMA_DECIMACION);   /* promedio de pares: 20 -> 10 kHz */
                acc = 0;
                k = 0;

                if (n == ADC_BLOCK_SAMPLES) {
                    int64_t tn = esp_timer_get_time();
                    busy += (uint32_t)(tn - t0);
                    t0 = tn;

                    uint32_t ovf_ahora = ovf_isr;
                    acq_common_block_done(blk, n, 0, err, ovf_ahora - ovf_prev, busy);
                    ovf_prev = ovf_ahora;
                    n = 0;
                    err = 0;
                    busy = 0;
                }
            }
            busy += (uint32_t)(esp_timer_get_time() - t0);
        }
    }

    xSemaphoreGive(fin);
    vTaskDelete(NULL);
}

void acq_dma_stop(void)
{
    if (adc != NULL) {
        adc_continuous_stop(adc);
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
        adc_continuous_deinit(adc);
        adc = NULL;
    }
}

esp_err_t acq_dma_start(void)
{
    esp_err_t err;
    acq_common_reset();
    ovf_isr = 0;

    adc_continuous_handle_cfg_t h_cfg = {
        .max_store_buf_size = POOL_BYTES,
        .conv_frame_size = TRAMA_BYTES,
    };
    err = adc_continuous_new_handle(&h_cfg, &adc);
    if (err != ESP_OK) goto fallo;

    adc_digi_pattern_config_t patron = {
        .atten = ADC_ATTEN_DB_12,
        .channel = POT_CANAL,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    };
    adc_continuous_config_t cfg = {
        .pattern_num = 1,
        .adc_pattern = &patron,
        .sample_freq_hz = DMA_FS_HW_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    err = adc_continuous_config(adc, &cfg);
    if (err != ESP_OK) goto fallo;

    fin = xSemaphoreCreateBinary();
    if (fin == NULL) { err = ESP_ERR_NO_MEM; goto fallo; }

    activa = true;
    if (xTaskCreatePinnedToCore(tarea_adquisicion, "acq_dma", TAREA_STACK, NULL,
                                TAREA_PRIORIDAD, &tarea, TAREA_NUCLEO) != pdPASS) {
        activa = false;
        tarea = NULL;
        err = ESP_ERR_NO_MEM;
        goto fallo;
    }

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = cb_conv_done,
        .on_pool_ovf = cb_pool_ovf,
    };
    err = adc_continuous_register_event_callbacks(adc, &cbs, NULL);
    if (err != ESP_OK) goto fallo;

    err = adc_continuous_start(adc);
    if (err != ESP_OK) goto fallo;

    ESP_LOGI(TAG, "DMA iniciado: %d Hz de hardware, decimacion %d:1 -> %d Hz efectivos, bloques de %d",
             DMA_FS_HW_HZ, DMA_DECIMACION, ADC_FS_EFECTIVA_HZ, ADC_BLOCK_SAMPLES);
    return ESP_OK;

fallo:
    ESP_LOGE(TAG, "Error al iniciar el ADC continuo: %s", esp_err_to_name(err));
    acq_dma_stop();
    return err;
}
