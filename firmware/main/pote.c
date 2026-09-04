#include "pote.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "pote";

// Lectura directa: cursor del potenciómetro conectado directo al ADC,
// sin amplificador ni divisor de referencia (versión simple, un solo canal).
#define CANAL_SENAL   ADC_CHANNEL_7   /* GPIO35 */
#define ATENUACION    ADC_ATTEN_DB_12

// Mismo rango que espera sensores_microros_main.c (posicion = grados/300*100),
// así el /posicion final da igual que con la versión amplificada.
#define RECORRIDO_DEG 300.0f

static adc_oneshot_unit_handle_t s_adc  = NULL;
static adc_cali_handle_t         s_cali = NULL;

static esp_err_t crear_calibracion(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ATENUACION,
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_cali_create_scheme_curve_fitting(&cfg, &s_cali);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ATENUACION,
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_cali_create_scheme_line_fitting(&cfg, &s_cali);
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t pote_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ATENUACION,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, CANAL_SENAL, &chan_cfg));

    if (crear_calibracion() != ESP_OK) {
        ESP_LOGW(TAG, "Sin calibracion de fabrica, solo cuentas crudas");
        s_cali = NULL;
    }
    return ESP_OK;
}

esp_err_t pote_leer(pote_muestra_t *m)
{
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc, CANAL_SENAL, &m->raw_senal));

    m->mv_senal = m->raw_senal;
    if (s_cali) {
        adc_cali_raw_to_voltage(s_cali, m->raw_senal, &m->mv_senal);
    }

    // No hay canal de referencia en esta versión.
    m->raw_ref = 0;
    m->mv_ref  = 0;

    // Mapeo lineal directo del ADC (0-4095) al recorrido mecánico total.
    m->grados = ((float)m->raw_senal / 4095.0f) * RECORRIDO_DEG;

    return ESP_OK;
}
