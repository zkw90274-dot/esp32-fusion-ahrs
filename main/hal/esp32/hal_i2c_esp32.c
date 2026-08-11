/**
 * @file    hal_i2c_esp32.c
 * @brief   ESP32 硬件 I2C HAL 实现
 */

#include "../hal_i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "hal_i2c";

/* ==================== 内部结构 ==================== */

struct hal_i2c {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t port;
    uint8_t addr;
};

/* ==================== API 实现 ==================== */

hal_i2c_t *hal_i2c_create(const hal_i2c_config_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    hal_i2c_t *i2c = (hal_i2c_t *)calloc(1, sizeof(hal_i2c_t));
    if (i2c == NULL) {
        return NULL;
    }

    i2c->port = config->port;
    i2c->addr = config->addr;

    /* 创建 I2C 总线 */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = config->port,
        .scl_io_num = config->scl_pin,
        .sda_io_num = config->sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &i2c->bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        free(i2c);
        return NULL;
    }

    /* 添加设备 */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->addr,
        .scl_speed_hz = config->freq_hz,
    };

    ret = i2c_master_bus_add_device(i2c->bus, &dev_cfg, &i2c->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(ret));
        i2c_del_master_bus(i2c->bus);
        free(i2c);
        return NULL;
    }

    ESP_LOGI(TAG, "I2C init OK (SCL=GPIO%u, SDA=GPIO%u, freq=%luHz, addr=0x%02X)",
             (unsigned)config->scl_pin, (unsigned)config->sda_pin,
             (unsigned long)config->freq_hz, (unsigned)config->addr);

    return i2c;
}

void hal_i2c_destroy(hal_i2c_t *i2c)
{
    if (i2c == NULL) {
        return;
    }

    if (i2c->dev != NULL) {
        i2c_master_bus_rm_device(i2c->dev);
    }
    if (i2c->bus != NULL) {
        i2c_del_master_bus(i2c->bus);
    }

    free(i2c);
}

int hal_i2c_write_reg(hal_i2c_t *i2c, uint8_t reg, uint8_t value)
{
    if (i2c == NULL || i2c->dev == NULL) {
        return -1;
    }

    uint8_t buf[2] = {reg, value};
    esp_err_t ret = i2c_master_transmit(i2c->dev, buf, 2, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: reg=0x%02X, err=%s", reg, esp_err_to_name(ret));
        return -1;
    }

    return 0;
}

int hal_i2c_read_reg(hal_i2c_t *i2c, uint8_t reg, uint8_t *value)
{
    if (i2c == NULL || i2c->dev == NULL || value == NULL) {
        return -1;
    }

    esp_err_t ret = i2c_master_transmit_receive(i2c->dev, &reg, 1, value, 1, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: reg=0x%02X, err=%s", reg, esp_err_to_name(ret));
        return -1;
    }

    return 0;
}

int hal_i2c_read_regs(hal_i2c_t *i2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (i2c == NULL || i2c->dev == NULL || buf == NULL || len == 0) {
        return -1;
    }

    esp_err_t ret = i2c_master_transmit_receive(i2c->dev, &reg, 1, buf, len, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C burst read failed: reg=0x%02X, len=%d, err=%s",
                 reg, len, esp_err_to_name(ret));
        return -1;
    }

    return 0;
}

int hal_i2c_probe(hal_i2c_t *i2c)
{
    if (i2c == NULL || i2c->bus == NULL) {
        return -1;
    }

    esp_err_t ret = i2c_master_probe(i2c->bus, i2c->addr, 100);
    if (ret != ESP_OK) {
        return -1;
    }

    return 0;
}
