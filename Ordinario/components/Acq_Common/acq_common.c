#include "acq_common.h"
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static acq_stats_t st;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool restart_req = false;
static volatile bool restart_ok = false;

static void poner_a_cero(void)
{
    memset(&st, 0, sizeof(st));
    st.min = 0xFFFF;
}

void acq_common_reset(void)
{
    portENTER_CRITICAL(&lock);
    poner_a_cero();
    portEXIT_CRITICAL(&lock);
    restart_req = false;
    restart_ok = false;
}

void acq_common_request_restart(void)
{
    restart_ok = false;
    restart_req = true;
}

bool acq_common_restart_done(void)
{
    return restart_ok;
}

void acq_common_block_done(const uint16_t *blk, size_t n,
                           uint32_t perdidas, uint32_t err_adc,
                           uint32_t overflows, uint32_t busy_us)
{
    int64_t t0 = esp_timer_get_time();

    /* --- Procesamiento del bloque (igual para One-shot y DMA) --- */
    uint32_t suma = 0;
    uint64_t suma_cuad = 0;
    uint16_t mn = 0xFFFF;
    uint16_t mx = 0;
    for (size_t i = 0; i < n; i++) {
        uint32_t v = blk[i];
        suma += v;
        suma_cuad += (uint64_t)v * v;
        if (v < mn) mn = (uint16_t)v;
        if (v > mx) mx = (uint16_t)v;
    }

    int64_t t1 = esp_timer_get_time();
    uint32_t t_proc = (uint32_t)(t1 - t0);

    portENTER_CRITICAL(&lock);
    if (restart_req) {
        /* El bloque que cruza el inicio de la ventana se descarta: la ventana
         * contada empieza exactamente en este limite de bloque. */
        poner_a_cero();
        st.t_inicio_us = t1;
        restart_req = false;
        restart_ok = true;
    } else {
        st.muestras += n;
        st.bloques++;
        st.perdidas += perdidas;
        st.err_adc += err_adc;
        st.overflows += overflows;
        st.suma += suma;
        st.suma_cuad += suma_cuad;
        if (mn < st.min) st.min = mn;
        if (mx > st.max) st.max = mx;
        st.t_proc_total_us += t_proc;
        if (t_proc > st.t_proc_max_us) st.t_proc_max_us = t_proc;
        st.busy_us += (uint64_t)busy_us + t_proc;
        st.t_ultimo_us = t1;
    }
    portEXIT_CRITICAL(&lock);
}

void acq_common_get(acq_stats_t *out)
{
    portENTER_CRITICAL(&lock);
    *out = st;
    portEXIT_CRITICAL(&lock);
}
