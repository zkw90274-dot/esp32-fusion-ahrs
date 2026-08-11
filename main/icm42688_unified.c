/**
 * @file    icm42688_unified.c
 * @brief   ICM-42688-P 统一驱动实现
 * @details 通过 HAL 抽象层访问传感器，支持 SPI 和 I2C
 */

#include "icm42688_unified.h"
#include "hal/hal_spi.h"
#include "hal/hal_i2c.h"
#include "platform.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "icm42688";

/* ==================== 寄存器定义 ==================== */

#define REG_DEVICE_CONFIG       0x11
#define REG_INT_CONFIG          0x14
#define REG_INT_CONFIG          0x14
#define REG_TEMP_DATA1          0x1D
#define REG_ACCEL_DATA_X1       0x1F
#define REG_GYRO_DATA_X1        0x25
#define REG_INT_STATUS          0x2D
#define REG_FIFO_CONFIG         0x16
#define REG_FIFO_CONFIG1        0x5F
#define REG_SIGNAL_PATH_RESET   0x4B
#define REG_INTF_CONFIG0        0x4C
#define REG_PWR_MGMT0           0x4E
#define REG_GYRO_CONFIG0        0x4F
#define REG_ACCEL_CONFIG0       0x50
#define REG_GYRO_CONFIG1        0x51
#define REG_GYRO_ACCEL_CONFIG0  0x52
#define REG_ACCEL_CONFIG1       0x53
#define REG_TMST_CONFIG         0x54
#define REG_INT_SOURCE0         0x65
#define REG_WHO_AM_I            0x75
#define REG_REG_BANK_SEL        0x76

/* ==================== 寄存器值定义 ==================== */

/* DEVICE_CONFIG */
#define DEV_CONFIG_RESET        0x01

/* GYRO_CONFIG1: 2阶滤波器, BW=ODR/4 */
#define GYRO_CONFIG1_DEFAULT    0x16

/* GYRO_ACCEL_CONFIG0: 加速度计和陀螺仪滤波器带宽 */
#define GYRO_ACCEL_CFG0_DEFAULT 0x11

/* INTF_CONFIG0: 小端格式 */
#define INTF_CONFIG0_LITTLE_ENDIAN  0x30

/* TMST_CONFIG: 使能时间戳 */
#define TMST_CONFIG_ENABLE      0x11

/* INT_CONFIG: 推挽输出，低电平有效 */
#define INT_CONFIG_DEFAULT      0x00

/* INT_SOURCE0: 数据就绪中断 */
#define INT_SRC0_DATA_RDY       0x08

/* PWR_MGMT0: Gyro LN + Accel LN */
#define PWR_MGMT0_ALL_LN        0x0F

/* INT_STATUS: RESET_DONE */
#define INT_STATUS_RESET_DONE   0x10

/* ==================== 内部结构 ==================== */

struct icm42688 {
    icm42688_iface_t iface;
    union {
        hal_spi_t *spi;
        hal_i2c_t *i2c;
    } bus;
    icm42688_config_t config;
    float gyro_sensitivity;
    float accel_sensitivity;
};

/* ==================== 灵敏度表 ==================== */

static const float gyro_sens_table[5] = {
    16.4f, 32.8f, 65.5f, 131.0f, 262.0f
};

static const float accel_sens_table[4] = {
    2048.0f, 4096.0f, 8192.0f, 16384.0f
};

/* ==================== 内部函数 ==================== */

static int imu_write_reg(icm42688_t *imu, uint8_t reg, uint8_t value)
{
    if (imu->iface == ICM42688_IFACE_SPI) {
        return hal_spi_write_reg(imu->bus.spi, reg, value);
    } else {
        return hal_i2c_write_reg(imu->bus.i2c, reg, value);
    }
}

static int imu_read_reg(icm42688_t *imu, uint8_t reg, uint8_t *value)
{
    if (imu->iface == ICM42688_IFACE_SPI) {
        return hal_spi_read_reg(imu->bus.spi, reg, value);
    } else {
        return hal_i2c_read_reg(imu->bus.i2c, reg, value);
    }
}

static int imu_read_regs(icm42688_t *imu, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (imu->iface == ICM42688_IFACE_SPI) {
        return hal_spi_read_regs(imu->bus.spi, reg, buf, len);
    } else {
        return hal_i2c_read_regs(imu->bus.i2c, reg, buf, len);
    }
}

static void imu_update_sensitivity(icm42688_t *imu)
{
    if (imu->config.gyro_fs < 5) {
        imu->gyro_sensitivity = 1.0f / gyro_sens_table[imu->config.gyro_fs];
    }
    if (imu->config.accel_fs < 4) {
        imu->accel_sensitivity = 1.0f / accel_sens_table[imu->config.accel_fs];
    }
}

static int imu_select_bank(icm42688_t *imu, uint8_t bank)
{
    if (bank > 4) return -1;
    return imu_write_reg(imu, REG_REG_BANK_SEL, bank);
}

static int imu_wait_reset(icm42688_t *imu)
{
    uint32_t timeout = 100;
    uint8_t status;

    while (timeout > 0) {
        if (imu_read_reg(imu, REG_INT_STATUS, &status) == 0) {
            if (status & INT_STATUS_RESET_DONE) {
                return 0;
            }
        }
        platform_delay_ms(1);
        timeout--;
    }

    PLATFORM_LOGE(TAG, "Reset timeout!");
    return -1;
}

static int imu_init_hardware(icm42688_t *imu)
{
    uint8_t device_id;

    PLATFORM_LOGI(TAG, "=== 初始化 ICM-42688-P ===");

    /* 等待上电稳定 */
    PLATFORM_LOGI(TAG, "[1/8] 等待上电稳定...");
    platform_delay_ms(500);

    /* 读取 WHO_AM_I */
    PLATFORM_LOGI(TAG, "[2/8] 读取 WHO_AM_I...");
    if (imu_read_reg(imu, REG_WHO_AM_I, &device_id) != 0) {
        PLATFORM_LOGE(TAG, "WHO_AM_I 读取失败!");
        return -1;
    }
    PLATFORM_LOGI(TAG, "  WHO_AM_I = 0x%02X (期望 0x%02X)", device_id, ICM42688_WHO_AM_I_VAL);
    if (device_id != ICM42688_WHO_AM_I_VAL) {
        PLATFORM_LOGE(TAG, "WHO_AM_I 校验失败!");
        return -1;
    }

    /* 软复位 */
    PLATFORM_LOGI(TAG, "[3/8] 软复位...");
    imu_write_reg(imu, REG_DEVICE_CONFIG, DEV_CONFIG_RESET);
    platform_delay_ms(10);
    if (imu_wait_reset(imu) != 0) {
        return -1;
    }
    imu_select_bank(imu, 0);

    /* 配置陀螺仪 */
    PLATFORM_LOGI(TAG, "[4/8] 配置陀螺仪...");
    imu_write_reg(imu, REG_GYRO_CONFIG0,
                  (imu->config.gyro_fs << 5) | imu->config.gyro_odr);

    /* 配置加速度计 */
    PLATFORM_LOGI(TAG, "[5/8] 配置加速度计...");
    imu_write_reg(imu, REG_ACCEL_CONFIG0,
                  (imu->config.accel_fs << 5) | imu->config.accel_odr);

    /* 配置滤波器 */
    PLATFORM_LOGI(TAG, "[6/8] 配置滤波器...");
    imu_write_reg(imu, REG_GYRO_CONFIG1, GYRO_CONFIG1_DEFAULT);
    imu_write_reg(imu, REG_GYRO_ACCEL_CONFIG0, GYRO_ACCEL_CFG0_DEFAULT);

    /* 配置接口和中断 */
    PLATFORM_LOGI(TAG, "[7/8] 配置接口...");
    imu_write_reg(imu, REG_INTF_CONFIG0, INTF_CONFIG0_LITTLE_ENDIAN);
    imu_write_reg(imu, REG_TMST_CONFIG, TMST_CONFIG_ENABLE);
    imu_write_reg(imu, REG_INT_CONFIG, INT_CONFIG_DEFAULT);
    imu_write_reg(imu, REG_INT_SOURCE0, INT_SRC0_DATA_RDY);

    /* 禁用 FIFO */
    imu_write_reg(imu, REG_FIFO_CONFIG, 0x00);
    imu_write_reg(imu, REG_FIFO_CONFIG1, 0x00);

    /* 使能传感器 */
    PLATFORM_LOGI(TAG, "[8/8] 使能 6轴低噪声模式...");
    imu_write_reg(imu, REG_PWR_MGMT0, PWR_MGMT0_ALL_LN);
    platform_delay_ms(50);

    /* 更新灵敏度 */
    imu_update_sensitivity(imu);

    PLATFORM_LOGI(TAG, "=== 初始化完成 ===");
    PLATFORM_LOGI(TAG, "  Gyro: %.4f dps/LSB", (double)(1.0f / imu->gyro_sensitivity));
    PLATFORM_LOGI(TAG, "  Accel: %.4f g/LSB", (double)(1.0f / imu->accel_sensitivity));

    return 0;
}

/* ==================== 公共 API ==================== */

icm42688_t *icm42688_create_spi(const icm42688_spi_config_t *spi_config,
                                const icm42688_config_t *imu_config)
{
    icm42688_t *imu = (icm42688_t *)calloc(1, sizeof(icm42688_t));
    if (imu == NULL) {
        return NULL;
    }

    imu->iface = ICM42688_IFACE_SPI;

    /* 使用默认配置或用户配置 */
    if (imu_config != NULL) {
        imu->config = *imu_config;
    } else {
        imu->config.gyro_fs = ICM42688_GYRO_FS_2000DPS;
        imu->config.accel_fs = ICM42688_ACCEL_FS_2G;
        imu->config.gyro_odr = ICM42688_ODR_200HZ;
        imu->config.accel_odr = ICM42688_ODR_200HZ;
    }

    /* 创建 SPI 实例 */
    hal_spi_config_t hal_cfg;
    if (spi_config != NULL) {
        hal_cfg.sclk_pin = spi_config->sclk_pin;
        hal_cfg.mosi_pin = spi_config->mosi_pin;
        hal_cfg.miso_pin = spi_config->miso_pin;
        hal_cfg.cs_pin = spi_config->cs_pin;
        hal_cfg.freq_hz = spi_config->freq_hz;
        hal_cfg.mode = 0;
        hal_cfg.port = 2;
    } else {
        hal_cfg = (hal_spi_config_t)HAL_SPI_DEFAULT_CONFIG;
    }

    imu->bus.spi = hal_spi_create(&hal_cfg);
    if (imu->bus.spi == NULL) {
        PLATFORM_LOGE(TAG, "SPI init failed");
        free(imu);
        return NULL;
    }

    /* 初始化传感器 */
    if (imu_init_hardware(imu) != 0) {
        hal_spi_destroy(imu->bus.spi);
        free(imu);
        return NULL;
    }

    return imu;
}

icm42688_t *icm42688_create_i2c(const icm42688_i2c_config_t *i2c_config,
                                const icm42688_config_t *imu_config)
{
    icm42688_t *imu = (icm42688_t *)calloc(1, sizeof(icm42688_t));
    if (imu == NULL) {
        return NULL;
    }

    imu->iface = ICM42688_IFACE_I2C;

    /* 使用默认配置或用户配置 */
    if (imu_config != NULL) {
        imu->config = *imu_config;
    } else {
        imu->config.gyro_fs = ICM42688_GYRO_FS_2000DPS;
        imu->config.accel_fs = ICM42688_ACCEL_FS_2G;
        imu->config.gyro_odr = ICM42688_ODR_200HZ;
        imu->config.accel_odr = ICM42688_ODR_200HZ;
    }

    /* 创建 I2C 实例 */
    hal_i2c_config_t hal_cfg;
    if (i2c_config != NULL) {
        hal_cfg.scl_pin = i2c_config->scl_pin;
        hal_cfg.sda_pin = i2c_config->sda_pin;
        hal_cfg.freq_hz = i2c_config->freq_hz;
        hal_cfg.addr = i2c_config->addr;
        hal_cfg.port = 0;
    } else {
        hal_cfg = (hal_i2c_config_t)HAL_I2C_DEFAULT_CONFIG;
    }

    imu->bus.i2c = hal_i2c_create(&hal_cfg);
    if (imu->bus.i2c == NULL) {
        PLATFORM_LOGE(TAG, "I2C init failed");
        free(imu);
        return NULL;
    }

    /* 初始化传感器 */
    if (imu_init_hardware(imu) != 0) {
        hal_i2c_destroy(imu->bus.i2c);
        free(imu);
        return NULL;
    }

    return imu;
}

void icm42688_destroy(icm42688_t *imu)
{
    if (imu == NULL) {
        return;
    }

    if (imu->iface == ICM42688_IFACE_SPI) {
        hal_spi_destroy(imu->bus.spi);
    } else {
        hal_i2c_destroy(imu->bus.i2c);
    }

    free(imu);
}

uint8_t icm42688_read_id(icm42688_t *imu)
{
    if (imu == NULL) {
        return 0xFF;
    }

    uint8_t id = 0xFF;
    imu_read_reg(imu, REG_WHO_AM_I, &id);
    return id;
}

int icm42688_read_all(icm42688_t *imu, icm42688_data_t *data)
{
    if (imu == NULL || data == NULL) {
        return -1;
    }

    uint8_t buf[14];
    if (imu_read_regs(imu, REG_TEMP_DATA1, buf, 14) != 0) {
        return -1;
    }

    int16_t raw_temp = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ax   = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_ay   = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t raw_az   = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t raw_gx   = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy   = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz   = (int16_t)((buf[12] << 8) | buf[13]);

    data->temperature = (float)raw_temp / 132.48f + 25.0f;
    data->ax = (float)raw_ax * imu->accel_sensitivity;
    data->ay = (float)raw_ay * imu->accel_sensitivity;
    data->az = (float)raw_az * imu->accel_sensitivity;
    data->gx = (float)raw_gx * imu->gyro_sensitivity;
    data->gy = (float)raw_gy * imu->gyro_sensitivity;
    data->gz = (float)raw_gz * imu->gyro_sensitivity;

    return 0;
}

icm42688_iface_t icm42688_get_iface(icm42688_t *imu)
{
    if (imu == NULL) {
        return ICM42688_IFACE_SPI;
    }
    return imu->iface;
}
