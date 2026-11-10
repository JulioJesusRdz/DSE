#pragma once
#include "esp_err.h"

/* Adquisicion SIN DMA: ADC1 canal 6 (GPIO34), lecturas por software a 10 kHz. */
esp_err_t acq_oneshot_start(void);
void      acq_oneshot_stop(void);
