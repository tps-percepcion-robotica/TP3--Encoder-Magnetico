#include "as5600.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "as5600";

#define I2C_PUERTO      I2C_NUM_0
#define PIN_SDA         21
#define PIN_SCL         22
#define AS5600_ADDR     0x36
#define I2C_FREQ_HZ     100000          // 100 kHz, no 400 kHz
#define I2C_TIMEOUT_MS  100

#define REG_STATUS      0x0B
#define REG_RAW_ANGLE   0x0C
#define REG_ANGLE       0x0E
#define REG_AGC         0x1A
#define REG_MAGNITUDE   0x1B

#define CUENTAS_POR_VUELTA  4096.0f

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

static esp_err_t leer_registros(uint8_t reg, uint8_t *dst, size_t n)
{
    esp_err_t err = ESP_FAIL;
    for (int intento = 0; intento < 3; intento++) {
        err = i2c_master_transmit_receive(s_dev, &reg, 1, dst, n, I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return err;
}

static esp_err_t leer_u12(uint8_t reg, uint16_t *valor)
{
    uint8_t buf[2];
    esp_err_t err = leer_registros(reg, buf, 2);
    if (err != ESP_OK) {
        return err;
    }
    *valor = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    return ESP_OK;
}

esp_err_t as5600_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = I2C_PUERTO,
        .sda_io_num                   = PIN_SDA,
        .scl_io_num                   = PIN_SCL,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo crear el bus I2C: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AS5600_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo agregar el dispositivo: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Bus I2C listo: SDA=%d SCL=%d a %d Hz",
             PIN_SDA, PIN_SCL, I2C_FREQ_HZ);
    return ESP_OK;
}

void as5600_escanear_bus(void)
{
    ESP_LOGI(TAG, "Escaneando bus I2C...");
    int encontrados = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "  Dispositivo en 0x%02X", addr);
            encontrados++;
        }
    }
    if (encontrados == 0) {
        ESP_LOGE(TAG, "  Ningun dispositivo. Revisar cableado y pull-ups.");
    }
}

esp_err_t as5600_leer(as5600_muestra_t *m)
{
    esp_err_t err;

    err = leer_registros(REG_STATUS, &m->status, 1);
    if (err != ESP_OK) return err;

    m->mh = (m->status >> 3) & 0x01;
    m->ml = (m->status >> 4) & 0x01;
    m->md = (m->status >> 5) & 0x01;

    err = leer_u12(REG_RAW_ANGLE, &m->raw_angle);
    if (err != ESP_OK) return err;

    err = leer_u12(REG_ANGLE, &m->angle);
    if (err != ESP_OK) return err;

    err = leer_registros(REG_AGC, &m->agc, 1);
    if (err != ESP_OK) return err;

    err = leer_u12(REG_MAGNITUDE, &m->magnitud);
    if (err != ESP_OK) return err;

    m->grados_raw = m->raw_angle * (360.0f / CUENTAS_POR_VUELTA);
    m->grados     = m->angle     * (360.0f / CUENTAS_POR_VUELTA);

    return ESP_OK;
}
