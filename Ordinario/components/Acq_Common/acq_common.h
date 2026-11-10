#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---------- Parametros comunes a AMBOS metodos (comparacion justa) ---------- */
#define ADC_FS_EFECTIVA_HZ   10000   /* frecuencia efectiva de muestreo (10 kHz)  */
#define ADC_BLOCK_SAMPLES    256     /* muestras por bloque = 25.6 ms a 10 kHz    */
#define ADC_POT_GPIO         34      /* ADC1_CH6                                  */

typedef struct {
    uint64_t muestras;          /* muestras efectivas procesadas                    */
    uint32_t bloques;
    uint32_t perdidas;          /* ticks de muestreo perdidos (solo One-shot)       */
    uint32_t err_adc;           /* lecturas fallidas / muestras invalidas / estado de
                                   desbordamiento reportado por la lectura DMA      */
    uint32_t overflows;         /* eventos de desbordamiento del pool DMA           */
    uint64_t t_proc_total_us;   /* tiempo total en acq_common_block_done()          */
    uint32_t t_proc_max_us;
    uint64_t busy_us;           /* tiempo de CPU de la tarea de adquisicion         */
    uint64_t suma;              /* estadisticos de la senal (cuentas crudas)        */
    uint64_t suma_cuad;
    uint16_t min;
    uint16_t max;
    int64_t  t_inicio_us;       /* inicio de la ventana contada                     */
    int64_t  t_ultimo_us;       /* instante del ultimo bloque procesado             */
} acq_stats_t;

/* Pone todo en cero (con la adquisicion detenida). */
void acq_common_reset(void);

/* Reinicia los contadores en el siguiente limite de bloque, con la adquisicion
 * en marcha. Se usa al terminar la estabilizacion. */
void acq_common_request_restart(void);
bool acq_common_restart_done(void);

/*
 * Procesamiento por bloque, IDENTICO para ambos metodos: suma, suma de
 * cuadrados, minimo y maximo. Tambien acumula contadores de error y mide su
 * propio tiempo.
 */
void acq_common_block_done(const uint16_t *blk, size_t n,
                           uint32_t perdidas, uint32_t err_adc,
                           uint32_t overflows, uint32_t busy_us);

void acq_common_get(acq_stats_t *out);
