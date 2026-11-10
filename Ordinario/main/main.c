/*
 * Evaluacion del consumo energetico del ESP32: ADC One-shot (sin DMA)
 * frente a ADC Continuous (con DMA), medido con un INA219.
 */
#include <stdio.h>
#include "experiment.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 

void app_main(void)
{
    // 1. Imprimir mensaje de inicio usando printf
    printf("Evaluacion energetica ADC con y sin DMA (ESP32)\n");

    // 2. Inicializar el hardware necesario (INA219, etc.)
    experiment_init();

    // 3. Bucle principal
    while (1)
    {
        // Ejecutar el protocolo completo del experimento
        experiment_run();
        // Al terminar las 3 repeticiones, imprimir aviso
        printf("Experimento finalizado. Reiniciando en 2 segundos...\n");
        // 4. Retardo usando vTaskDelay
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}