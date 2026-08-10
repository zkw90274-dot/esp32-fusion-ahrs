/**
 * @file    icm42688_i2c.c
 * @brief   ICM-42688-P 六轴 IMU 驱动实现（软件 I2C）
 * @details 基于平台抽象层，支持移植到不同 MCU
 */

#include "icm42688_i2c.h"
#include "hw_i2c_soft.h"
#include "platform.h"
#include <stddef.h>

static const char *TAG = "icm42688_i2c";

/* ==================== 私有变量 ==================== */

static hw_i2c_soft_t s_i2c;                     // 软件 I2C 句柄
static icm42688_i2c_config_t s_config;         // 当前配置
static float s_acc_sensitivity = 1.0f;         // 加速度计灵敏度 (g/LSB)
static float s_gyro_sensitivity = 1.0f;        // 陀螺仪灵敏度 (dps/LSB)

/* ==================== 灵敏度查找表 ==================== */

/* 陀螺仪灵敏度表 (LSB/dps) */
static const float gyro_sensitivity_table[5] = {
    16.4f,      // ICM42688_I2C_GYRO_FS_2000DPS
    32.8f,      // ICM42688_I2C_GYRO_FS_1000DPS
    65.5f,      // ICM42688_I2C_GYRO_FS_500DPS
    131.0f,     // ICM42688_I2C_GYRO_FS_250DPS
    262.0f      // ICM42688_I2C_GYRO_FS_125DPS
};

/* 加速度计灵敏度表 (LSB/g) */
static const float accel_sensitivity_table[4] = {
    2048.0f,    // ICM42688_I2C_ACCEL_FS_16G
    4096.0f,    // ICM42688_I2C_ACCEL_FS_8G
    8192.0f,    // ICM42688_I2C_ACCEL_FS_4G
    16384.0f    // ICM42688_I2C_ACCEL_FS_2G
};

/* ==================== 内部函数 ==================== */

/**
 * @brief   写入单个寄存器
 */
static void icm_write_reg(uint8_t reg, uint8_t value)
{
    hw_i2c_soft_write_reg(&s_i2c, reg, value);
}

/**
 * @brief   读取单个寄存器
 */
static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t value;
    hw_i2c_soft_read_reg(&s_i2c, reg, &value);
    return value;
}

/**
 * @brief   读取多个连续寄存器
 */
static void icm_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    hw_i2c_soft_read_regs(&s_i2c, reg, buf, len);
}

/**
 * @brief   更新灵敏度
 */
static void icm_update_sensitivity(void)
{
    if (s_config.gyro_fs < 5) {
        s_gyro_sensitivity = 1.0f / gyro_sensitivity_table[s_config.gyro_fs];
    }
    if (s_config.accel_fs < 4) {
        s_acc_sensitivity = 1.0f / accel_sensitivity_table[s_config.accel_fs];
    }
}

/**
 * @brief   等待复位完成
 */
static int8_t icm_wait_reset_done(void)
{
    uint32_t timeout = 100;  // 100ms 超时
    uint8_t int_status;

    while (timeout > 0) {
        int_status = icm_read_reg(ICM42688_I2C_REG_INT_STATUS);
        if (int_status & 0x10) {  // RESET_DONE 位
            return 0;
        }
        platform_delay_ms(1);
        timeout--;
    }

    PLATFORM_LOGE(TAG, "Reset timeout!");
    return -1;
}

/* ==================== 公共 API 实现 ==================== */

/**
 * @brief   读取 WHO_AM_I 寄存器
 */
uint8_t icm42688_i2c_read_id(void)
{
    return icm_read_reg(ICM42688_I2C_REG_WHO_AM_I);
}

/**
 * @brief   软复位
 */
int8_t icm42688_i2c_reset(void)
{
    PLATFORM_LOGI(TAG, "Performing soft reset...");

    // 触发软复位
    icm_write_reg(ICM42688_I2C_REG_DEVICE_CONFIG, 0x01);

    // 等待复位完成
    platform_delay_ms(10);
    return icm_wait_reset_done();
}

/**
 * @brief   自检
 */
int8_t icm42688_i2c_self_test(void)
{
    uint8_t whoami = icm42688_i2c_read_id();

    if (whoami != ICM42688_I2C_WHO_AM_I_VAL) {
        PLATFORM_LOGE(TAG, "Self test failed: WHO_AM_I = 0x%02X (expected 0x%02X)",
                 whoami, ICM42688_I2C_WHO_AM_I_VAL);
        return -1;
    }

    PLATFORM_LOGI(TAG, "Self test passed: WHO_AM_I = 0x%02X", whoami);
    return 0;
}

/**
 * @brief   获取当前配置
 */
void icm42688_i2c_get_config(icm42688_i2c_config_t *config)
{
    if (config != NULL) {
        *config = s_config;
    }
}

/**
 * @brief   初始化 ICM42688 (I2C 模式)
 * @details 按照官方 datasheet 推荐流程：
 *          1. 等待上电稳定 (100ms)
 *          2. 读取 WHO_AM_I 验证通信 (期望 0x47)
 *          3. 软复位 + 等待复位完成
 *          4. 配置陀螺仪量程和 ODR
 *          5. 配置加速度计量程和 ODR
 *          6. 配置 UI 滤波器带宽
 *          7. 配置中断 (数据就绪 → INT1)
 *          8. 使能 6轴低噪声模式
 *          9. 等待陀螺仪启动 (50ms)
 */
int8_t icm42688_i2c_init(const icm42688_i2c_config_t *config)
{
    uint8_t device_id;

    PLATFORM_LOGI(TAG, "=== 初始化 ICM-42688-P (I2C) ===");

    // 使用默认配置或用户配置
    if (config != NULL) {
        s_config = *config;
    } else {
        s_config.gyro_fs = ICM42688_I2C_GYRO_FS_2000DPS;
        s_config.accel_fs = ICM42688_I2C_ACCEL_FS_2G;
        s_config.gyro_odr = ICM42688_I2C_ODR_200HZ;
        s_config.accel_odr = ICM42688_I2C_ODR_200HZ;
    }

    // 初始化软件 I2C
    hw_i2c_soft_config_t i2c_cfg = HW_I2C_SOFT_DEFAULT_CONFIG;
    if (hw_i2c_soft_init(&s_i2c, &i2c_cfg) != 0) {
        PLATFORM_LOGE(TAG, "I2C init failed");
        return -1;
    }

    // ========== 步骤 1: 等待上电稳定 (100ms) ==========
    PLATFORM_LOGI(TAG, "[1/9] 等待上电稳定...");
    platform_delay_ms(100);

    // ========== 步骤 2: 读取 WHO_AM_I 验证通信 ==========
    PLATFORM_LOGI(TAG, "[2/9] 读取 WHO_AM_I 验证通信...");
    device_id = icm_read_reg(ICM42688_I2C_REG_WHO_AM_I);
    if (device_id != ICM42688_I2C_WHO_AM_I_VAL) {
        PLATFORM_LOGE(TAG, "WHO_AM_I 校验失败! 读取=0x%02X, 期望=0x%02X",
                 device_id, ICM42688_I2C_WHO_AM_I_VAL);
        hw_i2c_soft_deinit(&s_i2c);  // 清理 I2C 资源
        return -1;
    }
    PLATFORM_LOGI(TAG, "  WHO_AM_I = 0x%02X [OK]", device_id);

    // ========== 步骤 3: 软复位 + 等待复位完成 ==========
    PLATFORM_LOGI(TAG, "[3/9] 软复位...");
    if (icm42688_i2c_reset() != 0) {
        PLATFORM_LOGE(TAG, "软复位失败!");
        hw_i2c_soft_deinit(&s_i2c);  // 清理 I2C 资源
        return -1;
    }
    PLATFORM_LOGI(TAG, "  复位完成 [OK]");

    // ========== 步骤 4: 配置陀螺仪量程和 ODR ==========
    PLATFORM_LOGI(TAG, "[4/9] 配置陀螺仪...");
    uint8_t gyro_config0 = (s_config.gyro_fs << 5) | s_config.gyro_odr;
    icm_write_reg(ICM42688_I2C_REG_GYRO_CONFIG0, gyro_config0);
    PLATFORM_LOGI(TAG, "  FS=%d (+/- %d dps), ODR=0x%02X",
             s_config.gyro_fs,
             s_config.gyro_fs == 0 ? 2000 :
             s_config.gyro_fs == 1 ? 1000 :
             s_config.gyro_fs == 2 ? 500 :
             s_config.gyro_fs == 3 ? 250 : 125,
             s_config.gyro_odr);

    // ========== 步骤 5: 配置加速度计量程和 ODR ==========
    PLATFORM_LOGI(TAG, "[5/9] 配置加速度计...");
    uint8_t accel_config0 = (s_config.accel_fs << 5) | s_config.accel_odr;
    icm_write_reg(ICM42688_I2C_REG_ACCEL_CONFIG0, accel_config0);
    PLATFORM_LOGI(TAG, "  FS=%d (+/- %dg), ODR=0x%02X",
             s_config.accel_fs,
             s_config.accel_fs == 0 ? 16 :
             s_config.accel_fs == 1 ? 8 :
             s_config.accel_fs == 2 ? 4 : 2,
             s_config.accel_odr);

    // ========== 步骤 6: 配置 UI 滤波器带宽 ==========
    PLATFORM_LOGI(TAG, "[6/9] 配置滤波器...");
    icm_write_reg(ICM42688_I2C_REG_GYRO_CONFIG1, 0x16);  // 2阶滤波器
    icm_write_reg(ICM42688_I2C_REG_GYRO_ACCEL_CONFIG0, 0x11);  // BW = ODR/4
    PLATFORM_LOGI(TAG, "  Gyro: 2阶滤波器, BW=ODR/4");
    PLATFORM_LOGI(TAG, "  Accel: 2阶滤波器, BW=ODR/4");

    // ========== 步骤 7: 配置中断 (数据就绪 → INT1) ==========
    PLATFORM_LOGI(TAG, "[7/9] 配置中断...");
    icm_write_reg(ICM42688_I2C_REG_INT_CONFIG, 0x00);  // 推挽输出，低电平有效
    icm_write_reg(ICM42688_I2C_REG_INT_SOURCE0, 0x08);  // 数据就绪中断
    PLATFORM_LOGI(TAG, "  数据就绪中断 → INT1");

    // ========== 步骤 8: 使能 6轴低噪声模式 ==========
    PLATFORM_LOGI(TAG, "[8/9] 使能 6轴低噪声模式...");
    uint8_t pwr_mgmt0 = (3 << 2) | 3;  // Gyro LN + Accel LN
    icm_write_reg(ICM42688_I2C_REG_PWR_MGMT0, pwr_mgmt0);
    PLATFORM_LOGI(TAG, "  Gyro: 低噪声模式");
    PLATFORM_LOGI(TAG, "  Accel: 低噪声模式");

    // ========== 步骤 9: 等待陀螺仪启动 (50ms) ==========
    PLATFORM_LOGI(TAG, "[9/9] 等待陀螺仪启动...");
    platform_delay_ms(50);

    // 更新灵敏度
    icm_update_sensitivity();

    PLATFORM_LOGI(TAG, "=== ICM-42688-P 初始化完成 (I2C) ===");
    PLATFORM_LOGI(TAG, "  Gyro sensitivity: %.4f dps/LSB", (double)(1.0f / s_gyro_sensitivity));
    PLATFORM_LOGI(TAG, "  Accel sensitivity: %.4f g/LSB", (double)(1.0f / s_acc_sensitivity));

    return 0;
}

/**
 * @brief   获取温度
 */
int8_t icm42688_i2c_get_temperature(float *temperature)
{
    uint8_t buffer[2];
    int16_t raw_temp;

    icm_read_regs(ICM42688_I2C_REG_TEMP_DATA1, buffer, 2);
    raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);

    // 温度转换公式：TEMP(deg C) = TEMP_DATA / 132.48 + 25
    *temperature = (float)raw_temp / 132.48f + 25.0f;

    return 0;
}

/**
 * @brief   获取加速度计数据 (g)
 */
int8_t icm42688_i2c_get_accelerometer(float *ax, float *ay, float *az)
{
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;

    icm_read_regs(ICM42688_I2C_REG_ACCEL_DATA_X1, buffer, 6);

    raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    if (ax != NULL) *ax = (float)raw_x * s_acc_sensitivity;
    if (ay != NULL) *ay = (float)raw_y * s_acc_sensitivity;
    if (az != NULL) *az = (float)raw_z * s_acc_sensitivity;

    return 0;
}

/**
 * @brief   获取陀螺仪数据 (dps)
 */
int8_t icm42688_i2c_get_gyroscope(float *gx, float *gy, float *gz)
{
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;

    icm_read_regs(ICM42688_I2C_REG_GYRO_DATA_X1, buffer, 6);

    raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    if (gx != NULL) *gx = (float)raw_x * s_gyro_sensitivity;
    if (gy != NULL) *gy = (float)raw_y * s_gyro_sensitivity;
    if (gz != NULL) *gz = (float)raw_z * s_gyro_sensitivity;

    return 0;
}

/**
 * @brief   一次性读取所有传感器数据
 */
int8_t icm42688_i2c_get_all_data(icm42688_i2c_data_t *data)
{
    uint8_t buffer[14];
    int16_t raw_temp, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

    if (data == NULL) {
        return -1;
    }

    // 从 TEMP_DATA1 开始连续读取 14 字节
    // 温度(2) + 加速度计(6) + 陀螺仪(6)
    icm_read_regs(ICM42688_I2C_REG_TEMP_DATA1, buffer, 14);

    raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_ax   = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_ay   = (int16_t)((buffer[4] << 8) | buffer[5]);
    raw_az   = (int16_t)((buffer[6] << 8) | buffer[7]);
    raw_gx   = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw_gy   = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw_gz   = (int16_t)((buffer[12] << 8) | buffer[13]);

    // 转换为物理单位
    data->temperature = (float)raw_temp / 132.48f + 25.0f;
    data->ax = (float)raw_ax * s_acc_sensitivity;
    data->ay = (float)raw_ay * s_acc_sensitivity;
    data->az = (float)raw_az * s_acc_sensitivity;
    data->gx = (float)raw_gx * s_gyro_sensitivity;
    data->gy = (float)raw_gy * s_gyro_sensitivity;
    data->gz = (float)raw_gz * s_gyro_sensitivity;

    return 0;
}
