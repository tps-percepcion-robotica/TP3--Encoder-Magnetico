#pragma once

#include "esp_err.h"

typedef struct {
    int   raw_senal;
    int   raw_ref;
    int   mv_senal;
    int   mv_ref;
    float grados;
} pote_muestra_t;

esp_err_t pote_init(void);
esp_err_t pote_leer(pote_muestra_t *m);
