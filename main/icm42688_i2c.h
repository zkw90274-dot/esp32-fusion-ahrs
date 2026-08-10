/**
 * @file    icm42688_i2c.h
 * @brief   ICM-42688-P 六轴 IMU 驱动（软件 I2C 接口）
 * @details 基于平台抽象层，支持移植到不同 MCU
 */

#ifndef __ICM42688_I2C_H__
#define __ICM42688_I2C_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 设备 ID ==================== */

#define ICM42688_I2C_WHO_AM_I_VAL              0x47

/* ==================== I2C 地址 ==================== */

#define ICM42688_I2C_ADDR_AD0_LOW              0x68   /* AD0 = GND */
#define ICM42688_I2C_ADDR_AD0_HIGH             0x69   /* AD0 = VDDIO */

/* ==================== 寄存器地址（与 SPI 版本相同） ==================== */

/* 配置寄存器 */
#define ICM42688_I2C_REG_DEVICE_CONFIG         0x11
#define ICM42688_I2C_REG_INT_CONFIG            0x14

/* 数据寄存器 */
#define ICM42688_I2C_REG_TEMP_DATA1            0x1D
#define ICM42688_I2C_REG_ACCEL_DATA_X1         0x1F
#define ICM42688_I2C_REG_GYRO_DATA_X1          0x25

/* 状态寄存器 */
#define ICM42688_I2C_REG_INT_STATUS            0x2D

/* 中断配置寄存器 */
#define ICM42688_I2C_REG_INT_SOURCE0           0x65

/* 配置寄存器 */
#define ICM42688_I2C_REG_PWR_MGMT0             0x4E
#define ICM42688_I2C_REG_GYRO_CONFIG0          0x4F
#define ICM42688_I2C_REG_ACCEL_CONFIG0         0x50
#define ICM42688_I2C_REG_GYRO_CONFIG1          0x51
#define ICM42688_I2C_REG_GYRO_ACCEL_CONFIG0    0x52

/* ID 寄存器 */
#define ICM42688_I2C_REG_WHO_AM_I              0x75

/* ==================== 量程定义 ==================== */

/* 陀螺仪量程 */
#define ICM42688_I2C_GYRO_FS_2000DPS           0x00
#define ICM42688_I2C_GYRO_FS_1000DPS           0x01
#define ICM42688_I2C_GYRO_FS_500DPS            0x02
#define ICM42688_I2C_GYRO_FS_250DPS            0x03
#define ICM42688_I2C_GYRO_FS_125DPS            0x04

/* 加速度计量程 */
#define ICM42688_I2C_ACCEL_FS_16G              0x00
#define ICM42688_I2C_ACCEL_FS_8G               0x01
#define ICM42688_I2C_ACCEL_FS_4G               0x02
#define ICM42688_I2C_ACCEL_FS_2G               0x03

/* ODR 定义 */
#define ICM42688_I2C_ODR_1000HZ                0x06
#define ICM42688_I2C_ODR_200HZ                 0x07
#define ICM42688_I2C_ODR_100HZ                 0x08
#define ICM42688_I2C_ODR_50HZ                  0x09

/* ==================== 数据结构体 ==================== */

/**
 * @brief IMU 传感器数据结构体（转换后的物理值）
 */
typedef struct {
    float ax, ay, az;       // 加速度计 (g)
    float gx, gy, gz;       // 陀螺仪 (dps)
    float temperature;      // 温度 (deg C)
} icm42688_i2c_data_t;

/**
 * @brief IMU 配置结构体
 */
typedef struct {
    uint8_t gyro_fs;        // 陀螺仪量程
    uint8_t accel_fs;       // 加速度计量程
    uint8_t gyro_odr;       // 陀螺仪 ODR
    uint8_t accel_odr;      // 加速度计 ODR
} icm42688_i2c_config_t;

/* ==================== 默认配置 ==================== */

#define ICM42688_I2C_DEFAULT_CONFIG  { \
    .gyro_fs = ICM42688_I2C_GYRO_FS_2000DPS, \
    .accel_fs = ICM42688_I2C_ACCEL_FS_2G, \
    .gyro_odr = ICM42688_I2C_ODR_200HZ, \
    .accel_odr = ICM42688_I2C_ODR_200HZ \
}

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化 ICM42688 (I2C 模式)
 * @param   config  配置参数（NULL 使用默认配置）
 * @return  0=成功, -1=失败
 */
int8_t icm42688_i2c_init(const icm42688_i2c_config_t *config);

/**
 * @brief   读取 WHO_AM_I 寄存器
 * @return  设备 ID (0x47)
 */
uint8_t icm42688_i2c_read_id(void);

/**
 * @brief   一次性读取所有传感器数据
 * @param   data    数据结构体指针
 * @return  0=成功, -1=失败
 */
int8_t icm42688_i2c_get_all_data(icm42688_i2c_data_t *data);

/**
 * @brief   获取温度
 * @param   temperature 温度值指针 (deg C)
 * @return  0=成功, -1=失败
 */
int8_t icm42688_i2c_get_temperature(float *temperature);

/**
 * @brief   获取加速度计数据 (g)
 * @param   ax, ay, az  加速度值指针
 * @return  0=成功, -1=失败
 */
int8_t icm42688_i2c_get_accelerometer(float *ax, float *ay, float *az);

/**
 * @brief   获取陀螺仪数据 (dps)
 * @param   gx, gy, gz  角速度值指针
 * @return  0=成功, -1=失败
 */
int8_t icm42688_i2c_get_gyroscope(float *gx, float *gy, float *gz);

/**
 * @brief   软复位
 * @return  0=成功, -1=失败
 */
int8_t icm42688_i2c_reset(void);

/**
 * @brief   自检
 * @return  0=通过, -1=失败
 */
int8_t icm42688_i2c_self_test(void);

/**
 * @brief   获取当前配置
 * @param   config  配置结构体指针
 */
void icm42688_i2c_get_config(icm42688_i2c_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_I2C_H__ */
