/*
 * Protocolo del experimento (secciones 8 y 10 del documento).
 *
 * Todo el texto sale por UART desde ESTA tarea (nucleo 0). Las tareas de
 * adquisicion (nucleo 1) nunca imprimen, y el formato y el volumen de mensajes
 * son identicos para ambos metodos.
 */
#include "experiment.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ina219.h"
#include "acq_common.h"
#include "acq_oneshot.h"
#include "acq_dma.h"

/* ================= Parametros del experimento ================= */
#define DURACION_PRUEBA_S        3   /* ventana contada por metodo                  */
#define ESTABILIZACION_S         3    /* previa a cada prueba (no se contabiliza)    */
#define INTERVALO_INA_MS         1000  /* lectura del INA219                          */
#define INTERVALO_RESUMEN_S      1     /* linea de resumen por UART                   */
#define PAUSA_ENTRE_PRUEBAS_S    2     /* pausa y reinicializacion (no se contabiliza) */
#define REPETICIONES             1     /* pares de pruebas, orden alternado           */

/* Plausibilidad durante la estabilizacion */
#define I_MIN_PLAUSIBLE_MA       10.0f /* debajo de esto: posible alimentacion paralela por USB */
#define V_MIN_PLAUSIBLE_V        3.5f
#define V_TOLERANCIA_ENTRE_PRUEBAS_V  0.15f

typedef enum { MODO_SIN_DMA = 0, MODO_CON_DMA = 1 } modo_t;

static const char *NOMBRE_MODO[2] = { "SIN_DMA", "CON_DMA" };

typedef struct {
    esp_err_t (*start)(void);
    void (*stop)(void);
} metodo_t;

static const metodo_t METODOS[2] = {
    { acq_oneshot_start, acq_oneshot_stop },
    { acq_dma_start,     acq_dma_stop     },
};

typedef struct {
    double   duracion_s;
    double   v_prom_V;
    double   i_prom_mA;
    double   p_prom_mW;
    double   energia_J;
    uint32_t lecturas_ina;
    uint32_t err_ina;
    acq_stats_t adc;
    double   fs_efectiva_hz;
    double   t_proc_prom_ms;
    double   t_proc_max_ms;
    double   carga_cpu_pct;
} resultado_t;

static const char *TAG = "EXP";

/* Resultados en RAM estatica (no en la pila) */
static resultado_t resultados[REPETICIONES][2];
static double v_referencia = 0.0;

/* ======================== Utilidades ======================== */
static void esperar_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static modo_t modo_en_posicion(int rep, int posicion)
{
    /* Repeticiones pares: SIN_DMA primero; impares: CON_DMA primero */
    int primero = (rep % 2 == 0) ? MODO_SIN_DMA : MODO_CON_DMA;
    return (modo_t)(posicion == 0 ? primero : 1 - primero);
}

/* ===================== Estabilizacion ===================== */
static void estabilizar(modo_t modo)
{
    printf("[%s] Estabilizando %d s (no se contabiliza)...\n", NOMBRE_MODO[modo], ESTABILIZACION_S);

    double suma_v = 0.0;
    double suma_i = 0.0;
    int validas = 0;

    for (int s = 0; s < ESTABILIZACION_S; s++) {
        esperar_ms(INTERVALO_INA_MS);
        ina219_data_t d;
        if (ina219_read(&d) == ESP_OK) {
            suma_v += d.v_bus_V;
            suma_i += d.i_mA;
            validas++;
        }
    }

    if (validas == 0) {
        ESP_LOGW(TAG, "Estabilizacion: ninguna lectura valida del INA219. Revise el cableado.");
        return;
    }

    double v = suma_v / validas;
    double i = suma_i / validas;
    printf("[%s] Estabilizacion: V=%.3f V  I=%.2f mA (%d lecturas validas)\n",
           NOMBRE_MODO[modo], v, i, validas);

    if (i < (double)I_MIN_PLAUSIBLE_MA || v < (double)V_MIN_PLAUSIBLE_V) {
        ESP_LOGW(TAG, "Lectura poco plausible: posible alimentacion paralela por USB que omite el INA219.");
    }
    if (v_referencia == 0.0) {
        v_referencia = v;       /* tension de referencia de la sesion */
    }
}

/* ======================== Una prueba ======================== */
static esp_err_t ejecutar_prueba(modo_t modo, int rep, resultado_t *res)
{
    memset(res, 0, sizeof(*res));

    ESP_RETURN_ON_ERROR(METODOS[modo].start(), TAG, "No se pudo iniciar la adquisicion %s", NOMBRE_MODO[modo]);

    estabilizar(modo);

    /* La ventana contada comienza en un limite de bloque, con la adquisicion en marcha */
    acq_common_request_restart();
    int64_t espera_inicio = esp_timer_get_time();
    while (!acq_common_restart_done()) {
        if ((esp_timer_get_time() - espera_inicio) > 2000000) {
            METODOS[modo].stop();
            ESP_LOGE(TAG, "La adquisicion no entrego bloques: no se pudo iniciar la ventana");
            return ESP_ERR_TIMEOUT;
        }
        esperar_ms(1);
    }
    acq_stats_t s;
    acq_common_get(&s);
    const int64_t t_inicio = s.t_inicio_us;
    const int64_t t_fin = t_inicio + (int64_t)DURACION_PRUEBA_S * 1000000LL;

    printf("[%s] Inicio de la ventana de %d s (repeticion %d/%d)\n",
           NOMBRE_MODO[modo], DURACION_PRUEBA_S, rep + 1, REPETICIONES);

    /* ---- Ventana contada: lectura del INA219 e integracion de energia ---- */
    double energia = 0.0;
    double suma_v = 0.0, suma_i = 0.0, suma_p = 0.0;
    uint32_t lecturas = 0, err_ina = 0;
    ina219_data_t d = {0};
    bool hay_dato = false;
    int64_t t_prev = t_inicio;       /* instante de la ultima lectura valida */
    int64_t t_ina = t_inicio;
    int64_t t_resumen = t_inicio;
    int64_t ahora = t_inicio;

    for (;;) {
        ahora = esp_timer_get_time();
        const bool ultimo = (ahora >= t_fin);

        if (ultimo || (ahora - t_ina) >= (int64_t)INTERVALO_INA_MS * 1000LL) {
            t_ina = ahora;
            ina219_data_t m;
            if (ina219_read(&m) == ESP_OK) {
                /* E = sum(Pk x dt_k), con la marca de tiempo real entre lecturas validas */
                double dt = (double)(ahora - t_prev) / 1e6;
                energia += ((double)m.p_mW / 1000.0) * dt;
                t_prev = ahora;
                suma_v += m.v_bus_V;
                suma_i += m.i_mA;
                suma_p += m.p_mW;
                lecturas++;
                d = m;
                hay_dato = true;
            } else {
                err_ina++;       /* el intervalo se cubre con la siguiente lectura valida */
            }
        }

        if (ultimo) {
            break;
        }

        if ((ahora - t_resumen) >= (int64_t)INTERVALO_RESUMEN_S * 1000000LL) {
            t_resumen += (int64_t)INTERVALO_RESUMEN_S * 1000000LL;
            acq_stats_t p;
            acq_common_get(&p);
            printf("[%-7s r%d] t=%6.1f s | V=%.3f V | I=%7.2f mA | P=%7.1f mW | E=%.3f J | muestras=%llu | perdidas=%lu | err_adc=%lu | ovf=%lu | err_ina=%lu\n",
                   NOMBRE_MODO[modo], rep + 1,
                   (double)(ahora - t_inicio) / 1e6,
                   hay_dato ? (double)d.v_bus_V : 0.0,
                   hay_dato ? (double)d.i_mA : 0.0,
                   hay_dato ? (double)d.p_mW : 0.0,
                   energia,
                   (unsigned long long)p.muestras,
                   (unsigned long)p.perdidas, (unsigned long)p.err_adc,
                   (unsigned long)p.overflows, (unsigned long)err_ina);
        }
        esperar_ms(10);
    }

    /* Si la lectura final fallo, el ultimo tramo se cubre con la ultima potencia valida */
    if (hay_dato && t_prev < ahora) {
        energia += ((double)d.p_mW / 1000.0) * ((double)(ahora - t_prev) / 1e6);
    }

    /* Foto de los contadores ANTES de detener, y luego cierre ordenado */
    acq_common_get(&s);
    METODOS[modo].stop();

    /* ---- Resultados ---- */
    res->duracion_s = (double)(ahora - t_inicio) / 1e6;
    res->lecturas_ina = lecturas;
    res->err_ina = err_ina;
    res->energia_J = energia;
    if (lecturas > 0) {
        res->v_prom_V = suma_v / lecturas;
        res->i_prom_mA = suma_i / lecturas;
        res->p_prom_mW = suma_p / lecturas;
    }
    res->adc = s;

    double t_adq_s = (double)(s.t_ultimo_us - s.t_inicio_us) / 1e6;
    if (t_adq_s > 0.0 && s.bloques > 0) {
        res->fs_efectiva_hz = (double)s.muestras / t_adq_s;
        res->t_proc_prom_ms = (double)s.t_proc_total_us / s.bloques / 1000.0;
        res->t_proc_max_ms = (double)s.t_proc_max_us / 1000.0;
        res->carga_cpu_pct = 100.0 * (double)s.busy_us / (t_adq_s * 1e6);
    }

    printf("[%s] Prueba terminada: %.1f s | E=%.3f J | muestras=%llu | fs efectiva=%.1f Hz\n",
           NOMBRE_MODO[modo], res->duracion_s, res->energia_J,
           (unsigned long long)s.muestras, res->fs_efectiva_hz);
    return ESP_OK;
}

/* ========== Pausa y reinicializacion controlada entre pruebas ========== */
static void pausa_y_verificacion(void)
{
    printf("--- Pausa de %d s y reinicializacion (no se contabiliza) ---\n", PAUSA_ENTRE_PRUEBAS_S);
    /* La adquisicion ya esta detenida y los contadores se reinician en acq_*_start() */
    esperar_ms((uint32_t)PAUSA_ENTRE_PRUEBAS_S * 1000);

    double suma = 0.0;
    int validas = 0;
    for (int k = 0; k < 3; k++) {
        ina219_data_t d;
        if (ina219_read(&d) == ESP_OK) {
            suma += d.v_bus_V;
            validas++;
        }
        esperar_ms(200);
    }
    if (validas == 0) {
        ESP_LOGW(TAG, "No se pudo verificar la alimentacion tras la pausa");
        return;
    }
    double v = suma / validas;
    if (v_referencia > 0.0 && fabs(v - v_referencia) > (double)V_TOLERANCIA_ENTRE_PRUEBAS_V) {
        ESP_LOGW(TAG, "Alimentacion distinta a la de referencia: %.3f V vs %.3f V. Condiciones no equivalentes.",
                 v, v_referencia);
    } else {
        printf("Verificacion de alimentacion: V=%.3f V (referencia %.3f V) -> equivalente\n", v, v_referencia);
    }
}

/* ============================ Reporte ============================ */
#define NUM_METRICAS 11

static const char *NOMBRE_METRICA[NUM_METRICAS] = {
    "Duracion efectiva (s)",
    "Voltaje promedio (V)",
    "Corriente promedio (mA)",
    "Potencia promedio (mW)",
    "Energia acumulada (J)",
    "Muestras validas",
    "Muestras perdidas / errores",
    "Tiempo de proc. por bloque (ms)",
    "Desbordamientos DMA",
    "Carga CPU de adquisicion (%)",
    "Frecuencia efectiva (Hz)",
};

static double valor_metrica(const resultado_t *r, int m)
{
    switch (m) {
        case 0:  return r->duracion_s;
        case 1:  return r->v_prom_V;
        case 2:  return r->i_prom_mA;
        case 3:  return r->p_prom_mW;
        case 4:  return r->energia_J;
        case 5:  return (double)r->adc.muestras;
        case 6:  return (double)r->adc.perdidas + (double)r->adc.err_adc;
        case 7:  return r->t_proc_prom_ms;
        case 8:  return (double)r->adc.overflows;
        case 9:  return r->carga_cpu_pct;
        default: return r->fs_efectiva_hz;
    }
}

static void media_y_desv(const double *x, int n, double *media, double *desv)
{
    double s = 0.0;
    for (int i = 0; i < n; i++) s += x[i];
    double m = s / n;
    double q = 0.0;
    for (int i = 0; i < n; i++) q += (x[i] - m) * (x[i] - m);
    *media = m;
    *desv = (n > 1) ? sqrt(q / (n - 1)) : 0.0;     /* desviacion muestral */
}

static void reporte_final(void)
{
    printf("\n================ REPORTE FINAL: %d repeticion(es) ================\n", REPETICIONES);
    printf("Referencia: prueba SIN DMA.\n");
    printf("Diferencia %% = (CON_DMA - SIN_DMA) / SIN_DMA x 100\n");
    printf("Reduccion de energia %% = (E_sinDMA - E_DMA) / E_sinDMA x 100 (negativo: DMA consumio mas)\n\n");

    for (int rep = 0; rep < REPETICIONES; rep++) {
        double es = resultados[rep][MODO_SIN_DMA].energia_J;
        double ed = resultados[rep][MODO_CON_DMA].energia_J;
        double red = (es > 0.0) ? (es - ed) / es * 100.0 : 0.0;
        printf("Rep %d (primero %-7s): E_sin=%.3f J  E_dma=%.3f J  reduccion=%+.2f %%\n",
               rep + 1, NOMBRE_MODO[modo_en_posicion(rep, 0)], es, ed, red);
    }

    printf("\n%-34s | %-24s | %-24s | %s\n", "Metrica", "SIN DMA (media +/- sd)", "CON DMA (media +/- sd)", "Dif. %");
    for (int m = 0; m < NUM_METRICAS; m++) {
        double xs[REPETICIONES], xd[REPETICIONES];
        for (int rep = 0; rep < REPETICIONES; rep++) {
            xs[rep] = valor_metrica(&resultados[rep][MODO_SIN_DMA], m);
            xd[rep] = valor_metrica(&resultados[rep][MODO_CON_DMA], m);
        }
        double ms, ss, md, sd;
        media_y_desv(xs, REPETICIONES, &ms, &ss);
        media_y_desv(xd, REPETICIONES, &md, &sd);

        if (fabs(ms) > 1e-12) {
            printf("%-34s | %11.3f +/- %-9.3f | %11.3f +/- %-9.3f | %+8.2f\n",
                   NOMBRE_METRICA[m], ms, ss, md, sd, (md - ms) / ms * 100.0);
        } else {
            printf("%-34s | %11.3f +/- %-9.3f | %11.3f +/- %-9.3f | %8s\n",
                   NOMBRE_METRICA[m], ms, ss, md, sd, "n/a");
        }
    }

    double es_m, es_s, ed_m, ed_s;
    double xs[REPETICIONES], xd[REPETICIONES];
    for (int rep = 0; rep < REPETICIONES; rep++) {
        xs[rep] = resultados[rep][MODO_SIN_DMA].energia_J;
        xd[rep] = resultados[rep][MODO_CON_DMA].energia_J;
    }
    media_y_desv(xs, REPETICIONES, &es_m, &es_s);
    media_y_desv(xd, REPETICIONES, &ed_m, &ed_s);
    if (es_m > 0.0) {
        printf("\nReduccion de energia con medias: %+.2f %% (referencia: E_sinDMA = %.3f J)\n",
               (es_m - ed_m) / es_m * 100.0, es_m);
    }
    printf("Compare la diferencia con la dispersion entre repeticiones (sd) antes de concluir.\n");
    printf("==================================================================\n");
}

/* ============================ Publico ============================ */
esp_err_t experiment_init(void)
{
    ESP_RETURN_ON_ERROR(ina219_init(), TAG, "No se pudo iniciar el INA219. Revise SDA=21, SCL=22 y la alimentacion.");
    ina219_log_config();

    printf("Parametros: fs efectiva=%d Hz | bloque=%d muestras | ventana=%d s | estabilizacion=%d s | INA219 cada %d ms | resumen cada %d s | repeticiones=%d\n",
           ADC_FS_EFECTIVA_HZ, ADC_BLOCK_SAMPLES, DURACION_PRUEBA_S, ESTABILIZACION_S,
           INTERVALO_INA_MS, INTERVALO_RESUMEN_S, REPETICIONES);
    return ESP_OK;
}

void experiment_run(void)
{
    for (int rep = 0; rep < REPETICIONES; rep++) {
        printf("\n######## Repeticion %d de %d ########\n", rep + 1, REPETICIONES);

        for (int pos = 0; pos < 2; pos++) {
            modo_t modo = modo_en_posicion(rep, pos);
            printf("\n>>> Prueba %s\n", NOMBRE_MODO[modo]);

            if (ejecutar_prueba(modo, rep, &resultados[rep][modo]) != ESP_OK) {
                ESP_LOGE(TAG, "Prueba %s fallida; se cancela el experimento", NOMBRE_MODO[modo]);
                return;
            }
            if (pos == 0 || rep < REPETICIONES - 1) {
                pausa_y_verificacion();
            }
        }
    }
    reporte_final();
}
