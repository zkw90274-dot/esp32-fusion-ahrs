/**
 * @file    hw_i2c_soft.h
 * @brief   软件 I2C 驱动抽象接口
 * @details 平台无关的软件 I2C 接口定义
 *          适用于 ICM-42688-P 等传感器
 */

#ifndef __HW_I2C_SOFT_H__
#define __HW_I2C_SOFT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== I2C 地址定义 ==================== */

/* ICM-42688-P I2C 地址 (7位) */
#define ICM42688_I2C_ADDR_AD0_LOW    0x68   /* AD0 = GND */
#define ICM42688_I2C_ADDR_AD0_HIGH   0x69   /* AD0 = VDDIO */

/* 默认地址 (AD0 接 GND) */
#define ICM42688_I2C_ADDR            ICM42688_I2C_ADDR_AD0_LOW

/* ==================== 数据结构体 ==================== */

/**
 * @brief 软件 I2C 配置结构体（平台无关）
 */
typedef struct {
    uint8_t scl_pin;            // SCL 引脚号
    uint8_t sda_pin;            // SDA 引脚号
    uint32_t delay_us;          // 时钟延时（微秒），决定 I2C 速度
    uint8_t addr;               // 从机地址（7位）
} hw_i2c_soft_config_t;

/**
 * @brief 软件 I2C 句柄结构体（平台无关）
 */
typedef struct {
    void *platform_handle;      // 平台特定句柄
    uint8_t scl_pin;            // SCL 引脚号
    uint8_t sda_pin;            // SDA 引脚号
    uint32_t delay_us;          // 时钟延时
    uint8_t addr;               // 从机地址（7位）
    uint8_t inited;             // 初始化标志
} hw_i2c_soft_t;

/* ==================== 默认配置 ==================== */

/*
 * 默认配置：
 * - SCL = GPIO12 (原 SPI SCLK)
 * - SDA = GPIO10 (原 SPI MISO)
 * - 速度 = 400kHz (延时约 1.25us)
 * - 地址 = 0x68 (AD0 接 GND)
 */
#define HW_I2C_SOFT_DEFAULT_CONFIG  { \
    .scl_pin = 12,              /* GPIO12 */ \
    .sda_pin = 10,              /* GPIO10 */ \
    .delay_us = 2,              /* 约 400kHz */ \
    .addr = ICM42688_I2C_ADDR   /* 0x68 */ \
}

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化软件 I2C
 * @param   i2c     I2C 句柄指针
 * @param   config  配置结构体指针
 * @return  0=成功, -1=失败
 */
int hw_i2c_soft_init(hw_i2c_soft_t *i2c, const hw_i2c_soft_config_t *config);

/**
 * @brief   反初始化软件 I2C，释放资源
 * @param   i2c     I2C 句柄指针
 * @return  0=成功, -1=失败
 */
int hw_i2c_soft_deinit(hw_i2c_soft_t *i2c);

/**
 * @brief   写入单个寄存器
 * @param   i2c     I2C 句柄指针
 * @param   reg     寄存器地址
 * @param   value   写入的值
 * @return  0=成功, -1=失败
 */
int hw_i2c_soft_write_reg(hw_i2c_soft_t *i2c, uint8_t reg, uint8_t value);

/**
 * @brief   读取单个寄存器
 * @param   i2c     I2C 句柄指针
 * @param   reg     寄存器地址
 * @param   value   读取的值指针
 * @return  0=成功, -1=失败
 */
int hw_i2c_soft_read_reg(hw_i2c_soft_t *i2c, uint8_t reg, uint8_t *value);

/**
 * @brief   读取多个连续寄存器
 * @param   i2c     I2C 句柄指针
 * @param   reg     起始寄存器地址
 * @param   buf     数据缓冲区指针
 * @param   len     读取长度
 * @return  0=成功, -1=失败
 */
int hw_i2c_soft_read_regs(hw_i2c_soft_t *i2c, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief   写入多个连续寄存器
 * @param   i2c     I2C 句柄指针
 * @param   reg     起始寄存器地址
 * @param   buf     数据缓冲区指针
 * @param   len     写入长度
 * @return  0=成功, -1=失败
 */
int hw_i2c_soft_write_regs(hw_i2c_soft_t *i2c, uint8_t reg, const uint8_t *buf, uint16_t len);

/**
 * @brief   检测设备是否存在
 * @param   i2c     I2C 句柄指针
 * @return  0=存在, -1=不存在
 */
int hw_i2c_soft_probe(hw_i2c_soft_t *i2c);

#ifdef __cplusplus
}
#endif

#endif /* __HW_I2C_SOFT_H__ */
