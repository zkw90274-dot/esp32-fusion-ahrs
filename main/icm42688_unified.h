/**
 * @file    icm42688_unified.h
 * @brief   ICM-42688-P 统一驱动接口
 * @details 支持 I2C 和 SPI 两种接口，应用层无需关心底层实现
 */

#ifndef ICM42688_UNIFIED_H
#define ICM42688_UNIFIED_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 设备 ID ==================== */

#define ICM42688_WHO_AM_I_VAL  0x47

/* ==================== 接口类型 ==================== */

typedef enum {
    ICM42688_IFACE_SPI,
    ICM42688_IFACE_I2C,
} icm42688_iface_t;

/* ==================== 数据结构 ==================== */

typedef struct {
    float ax, ay, az;       /* 加速度计 (g) */
    float gx, gy, gz;       /* 陀螺仪 (dps) */
    float temperature;      /* 温度 (°C) */
} icm42688_data_t;

typedef struct {
    uint8_t gyro_fs;
    uint8_t accel_fs;
    uint8_t gyro_odr;
    uint8_t accel_odr;
} icm42688_config_t;

/* ==================== 量程定义 ==================== */

/* 陀螺仪量程 */
#define ICM42688_GYRO_FS_2000DPS   0x00
#define ICM42688_GYRO_FS_1000DPS   0x01
#define ICM42688_GYRO_FS_500DPS    0x02
#define ICM42688_GYRO_FS_250DPS    0x03
#define ICM42688_GYRO_FS_125DPS    0x04

/* 加速度计量程 */
#define ICM42688_ACCEL_FS_16G      0x00
#define ICM42688_ACCEL_FS_8G       0x01
#define ICM42688_ACCEL_FS_4G       0x02
#define ICM42688_ACCEL_FS_2G       0x03

/* ODR 定义 */
#define ICM42688_ODR_1000HZ        0x06
#define ICM42688_ODR_200HZ         0x07
#define ICM42688_ODR_100HZ         0x08
#define ICM42688_ODR_50HZ          0x09

/* ==================== 默认配置 ==================== */

#define ICM42688_DEFAULT_CONFIG  { \
    .gyro_fs = ICM42688_GYRO_FS_2000DPS, \
    .accel_fs = ICM42688_ACCEL_FS_2G, \
    .gyro_odr = ICM42688_ODR_200HZ, \
    .accel_odr = ICM42688_ODR_200HZ \
}

/* ==================== 句柄 ==================== */

typedef struct icm42688 icm42688_t;

/* ==================== SPI 配置 ==================== */

typedef struct {
    uint8_t sclk_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t cs_pin;
    uint32_t freq_hz;
} icm42688_spi_config_t;

/* ==================== I2C 配置 ==================== */

typedef struct {
    uint8_t scl_pin;
    uint8_t sda_pin;
    uint32_t freq_hz;
    uint8_t addr;
} icm42688_i2c_config_t;

/* ==================== API ==================== */

/**
 * @brief   通过 SPI 创建 ICM42688 实例
 * @param   spi_config  SPI 配置（NULL 使用默认）
 * @param   imu_config  IMU 配置（NULL 使用默认）
 * @return  IMU 句柄，NULL 失败
 */
icm42688_t *icm42688_create_spi(const icm42688_spi_config_t *spi_config,
                                const icm42688_config_t *imu_config);

/**
 * @brief   通过 I2C 创建 ICM42688 实例
 * @param   i2c_config  I2C 配置（NULL 使用默认）
 * @param   imu_config  IMU 配置（NULL 使用默认）
 * @return  IMU 句柄，NULL 失败
 */
icm42688_t *icm42688_create_i2c(const icm42688_i2c_config_t *i2c_config,
                                const icm42688_config_t *imu_config);

/**
 * @brief   销毁 ICM42688 实例
 * @param   imu     IMU 句柄
 */
void icm42688_destroy(icm42688_t *imu);

/**
 * @brief   读取 WHO_AM_I
 * @param   imu     IMU 句柄
 * @return  设备 ID，0xFF 表示错误
 */
uint8_t icm42688_read_id(icm42688_t *imu);

/**
 * @brief   读取所有传感器数据
 * @param   imu     IMU 句柄
 * @param   data    数据输出
 * @return  0=成功, -1=失败
 */
int icm42688_read_all(icm42688_t *imu, icm42688_data_t *data);

/**
 * @brief   获取接口类型
 * @param   imu     IMU 句柄
 * @return  接口类型
 */
icm42688_iface_t icm42688_get_iface(icm42688_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_UNIFIED_H */
