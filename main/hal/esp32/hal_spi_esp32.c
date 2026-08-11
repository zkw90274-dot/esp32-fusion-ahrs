/**
 * @file    hal_spi_esp32.c
 * @brief   ESP32 硬件 SPI HAL 实现
 */

#include "../hal_spi.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "hal_spi";

#define MAX_TRANSFER_SIZE   256

/* ==================== 内部结构 ==================== */

struct hal_spi {
    spi_host_device_t host;
    spi_device_handle_t dev;
    uint8_t cs_pin;
};

/* ==================== API 实现 ==================== */

hal_spi_t *hal_spi_create(const hal_spi_config_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    hal_spi_t *spi = (hal_spi_t *)calloc(1, sizeof(hal_spi_t));
    if (spi == NULL) {
        return NULL;
    }

    spi->cs_pin = config->cs_pin;

    /* 配置 CS 引脚为输出，默认高电平 */
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << config->cs_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_cfg);
    gpio_set_level(config->cs_pin, 1);

    /* 确定 SPI 主机 */
    spi_host_device_t host;
    if (config->port == 2) {
        host = SPI2_HOST;
    } else if (config->port == 3) {
        host = SPI3_HOST;
    } else {
        host = SPI2_HOST;
    }
    spi->host = host;

    /* 初始化 SPI 总线 */
    spi_bus_config_t bus_cfg = {
        .sclk_io_num = config->sclk_pin,
        .mosi_io_num = config->mosi_pin,
        .miso_io_num = config->miso_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_TRANSFER_SIZE,
    };

    esp_err_t ret = spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        free(spi);
        return NULL;
    }

    /* 添加 SPI 设备 */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = config->freq_hz,
        .mode = config->mode,
        .spics_io_num = -1,     /* 软件控制 CS */
        .queue_size = 1,
    };

    ret = spi_bus_add_device(host, &dev_cfg, &spi->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(host);
        free(spi);
        return NULL;
    }

    ESP_LOGI(TAG, "SPI init OK (SCLK=GPIO%u, MOSI=GPIO%u, MISO=GPIO%u, CS=GPIO%u, freq=%luHz)",
             (unsigned)config->sclk_pin, (unsigned)config->mosi_pin,
             (unsigned)config->miso_pin, (unsigned)config->cs_pin,
             (unsigned long)config->freq_hz);

    return spi;
}

void hal_spi_destroy(hal_spi_t *spi)
{
    if (spi == NULL) {
        return;
    }

    if (spi->dev != NULL) {
        spi_bus_remove_device(spi->dev);
    }
    spi_bus_free(spi->host);
    gpio_reset_pin(spi->cs_pin);

    free(spi);
}

int hal_spi_write_reg(hal_spi_t *spi, uint8_t reg, uint8_t value)
{
    if (spi == NULL || spi->dev == NULL) {
        return -1;
    }

    spi_transaction_ext_t t = {
        .base = {
            .flags = SPI_TRANS_VARIABLE_CMD,
            .cmd = reg & 0x7F,
            .length = 8,
            .tx_buffer = &value,
        },
        .command_bits = 8,
    };

    gpio_set_level(spi->cs_pin, 0);
    esp_err_t ret = spi_device_polling_transmit(spi->dev, (spi_transaction_t *)&t);
    gpio_set_level(spi->cs_pin, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed: reg=0x%02X, err=%s", reg, esp_err_to_name(ret));
        return -1;
    }

    return 0;
}

int hal_spi_read_reg(hal_spi_t *spi, uint8_t reg, uint8_t *value)
{
    if (spi == NULL || spi->dev == NULL || value == NULL) {
        return -1;
    }

    return hal_spi_read_regs(spi, reg, value, 1);
}

int hal_spi_read_regs(hal_spi_t *spi, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (spi == NULL || spi->dev == NULL || buf == NULL || len == 0) {
        return -1;
    }

    if (len > MAX_TRANSFER_SIZE) {
        ESP_LOGE(TAG, "SPI read_regs: len %d exceeds max %d", len, MAX_TRANSFER_SIZE);
        return -1;
    }

    /* 使用静态 TX 缓冲区发送 0x00 填充字节（避免栈溢出） */
    static uint8_t s_tx_zero[MAX_TRANSFER_SIZE] = {0};

    spi_transaction_ext_t t = {
        .base = {
            .flags = SPI_TRANS_VARIABLE_CMD,
            .cmd = reg | 0x80,
            .length = 8U * len,
            .rxlength = 8U * len,
            .tx_buffer = s_tx_zero,
            .rx_buffer = buf,
        },
        .command_bits = 8,
    };

    gpio_set_level(spi->cs_pin, 0);
    esp_err_t ret = spi_device_polling_transmit(spi->dev, (spi_transaction_t *)&t);
    gpio_set_level(spi->cs_pin, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read failed: reg=0x%02X, len=%d, err=%s",
                 reg, len, esp_err_to_name(ret));
        return -1;
    }

    return 0;
}
