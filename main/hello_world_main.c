/**
 * @file    hello_world_main.c
 * @brief   ICM42688 IMU 姿态解算（基于 Fusion 库）
 *
 * 使用 xioTechnologies/Fusion 库进行 6 轴姿态解算
 * 支持运行时零漂补偿（FusionBias）+ AHRS 融合算法
 *
 * 数据流: ICM42688 → 零漂补偿 → Fusion AHRS → 四元数 → 欧拉角
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "icm42688.h"
#include "Fusion.h"

static const char *TAG = "main";

/* ==================== 采样参数 ==================== */

#define SAMPLE_RATE     200
#define SAMPLE_DT_MS    (1000 / SAMPLE_RATE)
#define SAMPLE_DT_S     (1.0f / SAMPLE_RATE)

/* ==================== Fusion 配置 ==================== */

#define GYRO_RANGE      2000.0f     // 陀螺仪量程 ±2000 dps（必须与实际量程匹配）
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
    icm42688_sensor_data_t data;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    ESP_LOGI(TAG, "开始零漂标定，保持传感器静止...");

    for (int i = 0; i < CALIBRATE_COUNT; i++) {
        icm42688_get_all_data(&data);
        sum_x += data.gx;
        sum_y += data.gy;
        sum_z += data.gz;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    s_gyro_offset_x = sum_x / CALIBRATE_COUNT;
    s_gyro_offset_y = sum_y / CALIBRATE_COUNT;
    s_gyro_offset_z = sum_z / CALIBRATE_COUNT;

    ESP_LOGI(TAG, "零漂标定完成: X=%.4f, Y=%.4f, Z=%.4f °/s",
             (double)s_gyro_offset_x, (double)s_gyro_offset_y, (double)s_gyro_offset_z);
}

/**
 * @brief   IMU 姿态解算任务
 */
static void imu_task(void *arg)
{
    icm42688_sensor_data_t data;

    /* 等待系统稳定 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 初始化 ICM42688 */
    ESP_LOGI(TAG, "Initializing ICM42688...");
    if (icm42688_init(NULL) != 0) {
        ESP_LOGE(TAG, "ICM42688 init failed!");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "ICM42688 init OK, WHO_AM_I = 0x%02X", icm42688_read_id());
    vTaskDelay(pdMS_TO_TICKS(50));

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

    ESP_LOGI(TAG, "=== Fusion AHRS 启动 ===");
    ESP_LOGI(TAG, "采样率: %d Hz, 增益: %.2f", SAMPLE_RATE, (double)AHRS_GAIN);
    ESP_LOGI(TAG, "加速度拒绝: %.1f°, 量程: ±%.0f dps", (double)ACC_REJECTION, (double)GYRO_RANGE);

    /* 主循环 */
    TickType_t last_tick = xTaskGetTickCount();

    while (1) {
        /* 1. 计算实际采样周期 */
        TickType_t current_tick = xTaskGetTickCount();
        float dt = (current_tick - last_tick) * portTICK_PERIOD_MS / 1000.0f;
        last_tick = current_tick;

        /* 更新 Fusion 采样周期 */
        FusionAhrsSetSamplePeriod(&ahrs, dt);

        /* 2. 读取原始数据 */
        icm42688_get_all_data(&data);

        /* 3. 零漂补偿 */
        float gx = data.gx - s_gyro_offset_x;
        float gy = data.gy - s_gyro_offset_y;
        float gz = data.gz - s_gyro_offset_z;

        /* 4. 运行时零漂补偿 */
        FusionVector gyroscope = {.array = {gx, gy, gz}};
        gyroscope = FusionBiasUpdate(&bias, gyroscope);

        /* 5. 加速度计数据 */
        FusionVector accelerometer = {.array = {data.ax, data.ay, data.az}};

        /* 6. AHRS 更新（6轴，无磁力计） */
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer);

        /* 7. 获取欧拉角 */
        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
        FusionEuler euler = FusionQuaternionToEuler(quat);

        /* 8. VOFA 输出 */
        printf("%.2f,%.2f,%.2f\n",
               (double)euler.angle.roll,
               (double)euler.angle.pitch,
               (double)euler.angle.yaw);

        /* 精确采样率控制 */
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DT_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 ICM42688 IMU - Fusion AHRS 姿态解算");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s, cores: %d, revision: %d",
             CONFIG_IDF_TARGET, chip_info.cores, chip_info.revision);

    xTaskCreate(imu_task, "imu_task", 8192, NULL, 5, NULL);
}
