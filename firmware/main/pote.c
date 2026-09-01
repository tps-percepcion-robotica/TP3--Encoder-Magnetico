#include "pote.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "pote";

#define CANAL_SENAL   ADC_CHANNEL_7   /* GPIO35 */
#define CANAL_REF     ADC_CHANNEL_6   /* GPIO34 */
#define ATENUACION    ADC_ATTEN_DB_12

#define GANANCIA_AMP  2.0f
#define R_POTE        10000.0f
#define R_PIE         1000.0f
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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, CANAL_REF,   &chan_cfg));

    if (crear_calibracion() != ESP_OK) {
        ESP_LOGW(TAG, "Sin calibracion de fabrica, solo cuentas crudas");
        s_cali = NULL;
    }
    return ESP_OK;
}

esp_err_t pote_leer(pote_muestra_t *m)
{
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc, CANAL_SENAL, &m->raw_senal));
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc, CANAL_REF,   &m->raw_ref));

    m->mv_senal = m->raw_senal;
    m->mv_ref   = m->raw_ref;
    if (s_cali) {
        adc_cali_raw_to_voltage(s_cali, m->raw_senal, &m->mv_senal);
        adc_cali_raw_to_voltage(s_cali, m->raw_ref,   &m->mv_ref);
    }

    float v_cursor = m->mv_senal / GANANCIA_AMP;
    float v_arriba = m->mv_ref;
    float v_abajo  = v_arriba * R_PIE / (R_POTE + R_PIE);

    float span = v_arriba - v_abajo;
    m->grados = (span > 1.0f)
              ? ((v_cursor - v_abajo) / span) * RECORRIDO_DEG
              : 0.0f;

    return ESP_OK;
}
