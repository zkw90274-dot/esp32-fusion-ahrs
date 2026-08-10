/**
 * @file    hello_world_main.c
 * @brief   ICM42688 IMU 姿态解算（基于 Fusion 库，I2C 模式）
 *
 * 使用 xioTechnologies/Fusion 库进行 6 轴姿态解算
 * 支持运行时零漂补偿（FusionBias）+ AHRS 融合算法
 *
 * 数据流: ICM42688(I2C) → 零漂补偿 → Fusion AHRS → 四元数 → 欧拉角
 *
 * 硬件接线:
 *   SCL = GPIO12 (原 SPI SCLK)
 *   SDA = GPIO10 (原 SPI MISO)
 *   CS  = GPIO9  (必须接高电平，选择 I2C 模式)
 *   AD0 = GPIO11 (地址选择，接 GND = 0x68)
 */

#include <stdio.h>
#include "platform.h"
#include "icm42688_i2c.h"
#include "Fusion.h"

static const char *TAG = "main";

/* ==================== 采样参数 ==================== */

#define SAMPLE_RATE     200
#define SAMPLE_DT_MS    (1000 / SAMPLE_RATE)
#define SAMPLE_DT_S     (1.0f / SAMPLE_RATE)

/* ==================== Fusion 配置 ==================== */

#define GYRO_RANGE      2000.0f     // 陀螺仪量程 +/-2000 dps（必须与实际量程匹配）
#define AHRS_GAIN       0.5f        // AHRS 融合增益（0~1）
#define ACC_REJECTION   10.0f       // 加速度拒绝阈值（度）
#define REJECTION_TIMEOUT 5.0f      // 恢复超时（秒）

/* ==================== 零漂标定 ==================== */

#define CALIBRATE_COUNT 500

static float s_gyro_offset_x = 0, s_gyro_offset_y = 0, s_gyro_offset_z = 0;

/**
 * @brief   陀螺仪零漂标定（500 样本平均）
 */
static void calibrate_gyro(void)
{
    icm42688_i2c_data_t data;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    PLATFORM_LOGI(TAG, "开始零漂标定，保持传感器静止...");

    for (int i = 0; i < CALIBRATE_COUNT; i++) {
        icm42688_i2c_get_all_data(&data);
        sum_x += data.gx;
        sum_y += data.gy;
        sum_z += data.gz;

        /* 每 100 个样本输出进度 */
        if ((i + 1) % 100 == 0) {
            PLATFORM_LOGI(TAG, "标定进度: %d/%d", i + 1, CALIBRATE_COUNT);
        }

        platform_delay_ms(5);
    }

    s_gyro_offset_x = sum_x / CALIBRATE_COUNT;
    s_gyro_offset_y = sum_y / CALIBRATE_COUNT;
    s_gyro_offset_z = sum_z / CALIBRATE_COUNT;

    PLATFORM_LOGI(TAG, "零漂标定完成: X=%.4f, Y=%.4f, Z=%.4f deg/s",
             (double)s_gyro_offset_x, (double)s_gyro_offset_y, (double)s_gyro_offset_z);
}

/**
 * @brief   IMU 姿态解算任务
 */
static void imu_task(void *arg)
{
    icm42688_i2c_data_t data;

    /* 等待系统稳定 */
    platform_delay_ms(500);

    /* 初始化 ICM42688 (I2C 模式) */
    PLATFORM_LOGI(TAG, "Initializing ICM42688 (I2C mode)...");

    /* 创建 IMU 配置，确保量程与 Fusion 设置匹配 */
    icm42688_i2c_config_t imu_config = {
        .gyro_fs = ICM42688_I2C_GYRO_FS_2000DPS,   // 匹配 GYRO_RANGE = 2000.0f
        .accel_fs = ICM42688_I2C_ACCEL_FS_2G,
        .gyro_odr = ICM42688_I2C_ODR_200HZ,         // 匹配 SAMPLE_RATE = 200
        .accel_odr = ICM42688_I2C_ODR_200HZ
    };

    if (icm42688_i2c_init(&imu_config) != 0) {
        PLATFORM_LOGE(TAG, "ICM42688 I2C init failed!");
        platform_task_delete();
        return;
    }
    PLATFORM_LOGI(TAG, "ICM42688 I2C init OK, WHO_AM_I = 0x%02X", icm42688_i2c_read_id());
    platform_delay_ms(50);

    /* 零漂标定 */
    calibrate_gyro();

    /* 初始化 Fusion AHRS */
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    FusionAhrsSettings settings = fusionAhrsDefaultSettings;
    settings.sampleRate = (float)SAMPLE_RATE;
    settings.convention = FusionConventionNwu;
    settings.gain = AHRS_GAIN;
    settings.gyroscopeRange = GYRO_RANGE;
    settings.accelerationRejection = ACC_REJECTION;
    settings.rejectionTimeout = REJECTION_TIMEOUT;
    FusionAhrsSetSettings(&ahrs, &settings);

    /* 初始化 Fusion Bias（运行时零漂补偿） */
    FusionBias bias;
    FusionBiasInitialise(&bias);

    FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
    biasSettings.sampleRate = (float)SAMPLE_RATE;
    biasSettings.stationaryThreshold = 3.0f;    // dps
    biasSettings.stationaryPeriod = 3.0f;        // 秒
    FusionBiasSetSettings(&bias, &biasSettings);

    PLATFORM_LOGI(TAG, "=== Fusion AHRS 启动 ===");
    PLATFORM_LOGI(TAG, "采样率: %d Hz, 增益: %.2f", SAMPLE_RATE, (double)AHRS_GAIN);
    PLATFORM_LOGI(TAG, "加速度拒绝: %.1f deg, 量程: +/- %.0f dps", (double)ACC_REJECTION, (double)GYRO_RANGE);

    /* 主循环 */
    uint32_t last_tick = platform_get_tick_count();
    const uint32_t period_ms = SAMPLE_DT_MS;

    while (1) {
        /* 1. 精确周期延时 */
        platform_delay_ms(period_ms);

        /* 2. 计算实际采样周期 */
        uint32_t current_tick = platform_get_tick_count();
        float dt = (current_tick - last_tick) * platform_get_tick_period_ms() / 1000.0f;
        last_tick = current_tick;

        /* 更新 Fusion 采样周期 */
        FusionAhrsSetSamplePeriod(&ahrs, dt);

        /* 3. 读取原始数据 */
        icm42688_i2c_get_all_data(&data);

        /* 4. 零漂补偿 */
        float gx = data.gx - s_gyro_offset_x;
        float gy = data.gy - s_gyro_offset_y;
        float gz = data.gz - s_gyro_offset_z;

        /* 5. 运行时零漂补偿 */
        FusionVector gyroscope = {.array = {gx, gy, gz}};
        gyroscope = FusionBiasUpdate(&bias, gyroscope);

        /* 6. 加速度计数据 */
        FusionVector accelerometer = {.array = {data.ax, data.ay, data.az}};

        /* 7. AHRS 更新（6轴，无磁力计） */
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer);

        /* 8. 获取欧拉角 */
        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
        FusionEuler euler = FusionQuaternionToEuler(quat);

        /* 9. VOFA 输出 */
        printf("%.2f,%.2f,%.2f\n",
               (double)euler.angle.roll,
               (double)euler.angle.pitch,
               (double)euler.angle.yaw);
    }
}

void app_main(void)
{
    PLATFORM_LOGI(TAG, "ICM42688 IMU - Fusion AHRS (I2C mode)");

    PLATFORM_LOGI(TAG, "Chip: %s, cores: %d, revision: %d",
             platform_get_chip_model(),
             platform_get_cpu_cores(),
             platform_get_chip_revision());

    platform_task_create(imu_task, "imu_task", 8192, NULL,
                        PLATFORM_TASK_PRIORITY_NORMAL, NULL);
}
