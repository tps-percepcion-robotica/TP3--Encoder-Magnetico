#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint16_t raw_angle;
    uint16_t angle;
    float    grados_raw;
    float    grados;
    uint8_t  status;
    uint8_t  agc;
    uint16_t magnitud;
    bool     md;
    bool     ml;
    bool     mh;
} as5600_muestra_t;

esp_err_t as5600_init(void);
esp_err_t as5600_leer(as5600_muestra_t *m);
void      as5600_escanear_bus(void);
