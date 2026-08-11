/**
 * @file    hal_spi.h
 * @brief   SPI 硬件抽象层接口
 * @details 平台无关的 SPI 主机接口，用于传感器驱动
 */

#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== SPI 句柄 ==================== */

typedef struct hal_spi hal_spi_t;

/* ==================== SPI 配置 ==================== */

typedef struct {
    uint8_t sclk_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t cs_pin;
    uint32_t freq_hz;
    uint8_t mode;           /* SPI mode 0-3 */
    uint8_t port;           /* SPI port number */
} hal_spi_config_t;

/* ==================== 默认配置 ==================== */

#define HAL_SPI_DEFAULT_CONFIG  { \
    .sclk_pin = 12, \
    .mosi_pin = 11, \
    .miso_pin = 10, \
    .cs_pin = 9, \
    .freq_hz = 1000000, \
    .mode = 0, \
    .port = 2 \
}

/* ==================== API ==================== */

/**
 * @brief   创建 SPI 实例
 * @param   config  配置
 * @return  SPI 句柄，NULL 失败
 */
hal_spi_t *hal_spi_create(const hal_spi_config_t *config);

/**
 * @brief   销毁 SPI 实例
 * @param   spi     SPI 句柄
 */
void hal_spi_destroy(hal_spi_t *spi);

/**
 * @brief   写入单个寄存器
 * @param   spi     SPI 句柄
 * @param   reg     寄存器地址
 * @param   value   写入值
 * @return  0=成功, -1=失败
 */
int hal_spi_write_reg(hal_spi_t *spi, uint8_t reg, uint8_t value);

/**
 * @brief   读取单个寄存器
 * @param   spi     SPI 句柄
 * @param   reg     寄存器地址
 * @param   value   读取值指针
 * @return  0=成功, -1=失败
 */
int hal_spi_read_reg(hal_spi_t *spi, uint8_t reg, uint8_t *value);

/**
 * @brief   读取多个连续寄存器
 * @param   spi     SPI 句柄
 * @param   reg     起始寄存器地址
 * @param   buf     数据缓冲区
 * @param   len     读取长度
 * @return  0=成功, -1=失败
 */
int hal_spi_read_regs(hal_spi_t *spi, uint8_t reg, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SPI_H */
