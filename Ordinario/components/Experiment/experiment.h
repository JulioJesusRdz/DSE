#pragma once
#include "esp_err.h"

/* Inicializa el INA219 y muestra los parametros del experimento. */
esp_err_t experiment_init(void);

/*
 * Ejecuta el protocolo completo: por cada repeticion, estabilizacion + prueba
 * sin DMA, pausa controlada y estabilizacion + prueba con DMA (el orden se
 * alterna entre repeticiones). Al final imprime la comparacion por UART.
 */
void experiment_run(void);
