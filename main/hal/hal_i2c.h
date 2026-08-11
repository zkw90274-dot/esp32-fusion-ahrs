/**
 * @file    hal_i2c.h
 * @brief   I2C 硬件抽象层接口
 * @details 平台无关的 I2C 主机接口，用于传感器驱动
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== I2C 句柄 ==================== */

typedef struct hal_i2c hal_i2c_t;

/* ==================== I2C 配置 ==================== */

typedef struct {
    uint8_t scl_pin;
    uint8_t sda_pin;
    uint32_t freq_hz;
    uint8_t addr;           /* 7-bit slave address */
    uint8_t port;           /* I2C port number */
} hal_i2c_config_t;

/* ==================== 默认配置 ==================== */

#define HAL_I2C_DEFAULT_CONFIG  { \
    .scl_pin = 12, \
    .sda_pin = 10, \
    .freq_hz = 400000, \
    .addr = 0x68, \
    .port = 0 \
}

/* ==================== API ==================== */

/**
 * @brief   创建 I2C 实例
 * @param   config  配置
 * @return  I2C 句柄，NULL 失败
 */
hal_i2c_t *hal_i2c_create(const hal_i2c_config_t *config);

/**
 * @brief   销毁 I2C 实例
 * @param   i2c     I2C 句柄
 */
void hal_i2c_destroy(hal_i2c_t *i2c);

/**
 * @brief   写入单个寄存器
 * @param   i2c     I2C 句柄
 * @param   reg     寄存器地址
 * @param   value   写入值
 * @return  0=成功, -1=失败
 */
int hal_i2c_write_reg(hal_i2c_t *i2c, uint8_t reg, uint8_t value);

/**
 * @brief   读取单个寄存器
 * @param   i2c     I2C 句柄
 * @param   reg     寄存器地址
 * @param   value   读取值指针
 * @return  0=成功, -1=失败
 */
int hal_i2c_read_reg(hal_i2c_t *i2c, uint8_t reg, uint8_t *value);

/**
 * @brief   读取多个连续寄存器
 * @param   i2c     I2C 句柄
 * @param   reg     起始寄存器地址
 * @param   buf     数据缓冲区
 * @param   len     读取长度
 * @return  0=成功, -1=失败
 */
int hal_i2c_read_regs(hal_i2c_t *i2c, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief   探测设备是否存在
 * @param   i2c     I2C 句柄
 * @return  0=存在, -1=不存在
 */
int hal_i2c_probe(hal_i2c_t *i2c);

#ifdef __cplusplus
}
#endif

#endif /* HAL_I2C_H */
