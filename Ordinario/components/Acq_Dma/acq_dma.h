#pragma once
#include "esp_err.h"

/* Adquisicion CON DMA: ADC1 canal 6 (GPIO34), ADC Continuous. */
esp_err_t acq_dma_start(void);
void      acq_dma_stop(void);
