/**
 * @file    app_main.c
 * @brief   ICM42688 IMU 姿态解算应用
 * @details 使用统一驱动接口，支持 SPI/I2C 无缝切换
 *
 * 硬件接线 (SPI 模式):
 *   SCLK = GPIO12
 *   MOSI = GPIO11 (SDI)
 *   MISO = GPIO10 (SDO)
 *   CS   = GPIO9
 *   INT1 = GPIO13 (可选)
 */

#include <stdio.h>
#include <math.h>
#include "platform.h"
#include "icm42688_unified.h"
#include "Fusion.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "app";

/* ==================== 配置 ==================== */

/* 接口选择：0=SPI, 1=I2C */
#define USE_I2C         0

/* GPIO 引脚定义 */
#define PIN_SCLK        12
#define PIN_MOSI        11
#define PIN_MISO        10
#define PIN_CS          9
#define PIN_SCL         12
#define PIN_SDA         10
#define PIN_INT1        13

/* 采样参数 */
#define SAMPLE_RATE     100
#define SAMPLE_DT_MS    10

/* Fusion 参数 */
#define GYRO_RANGE      2000.0f
#define AHRS_GAIN       0.3f
#define ACC_REJECTION   10.0f
#define REJECTION_TIMEOUT 5.0f

/* 标定参数 */
#define CALIBRATE_COUNT 500

/* GPIO */
#define INT1_PIN        GPIO_NUM_13
#define CS_PIN          GPIO_NUM_9

/* ==================== 全局变量 ==================== */

static float s_gyro_offset_x = 0, s_gyro_offset_y = 0, s_gyro_offset_z = 0;
static TaskHandle_t s_task_handle = NULL;

/* ==================== 中断处理 ==================== */

static void IRAM_ATTR int1_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_task_handle, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void setup_interrupt(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INT1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT1_PIN, int1_isr, NULL);
    PLATFORM_LOGI(TAG, "INT1 interrupt on GPIO%d", INT1_PIN);
}

/* ==================== 标定 ==================== */

static void calibrate(icm42688_t *imu)
{
    icm42688_data_t data;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    PLATFORM_LOGI(TAG, "零漂标定中，请保持静止...");

    for (int i = 0; i < CALIBRATE_COUNT; i++) {
        icm42688_read_all(imu, &data);
        sum_x += data.gx;
        sum_y += data.gy;
        sum_z += data.gz;
        platform_delay_ms(5);
    }

    s_gyro_offset_x = sum_x / CALIBRATE_COUNT;
    s_gyro_offset_y = sum_y / CALIBRATE_COUNT;
    s_gyro_offset_z = sum_z / CALIBRATE_COUNT;

    PLATFORM_LOGI(TAG, "标定完成: X=%.4f Y=%.4f Z=%.4f",
             (double)s_gyro_offset_x, (double)s_gyro_offset_y, (double)s_gyro_offset_z);
}

/* ==================== 主任务 ==================== */

static void imu_task(void *arg)
{
    s_task_handle = xTaskGetCurrentTaskHandle();

    /* 创建 IMU 实例 */
    icm42688_t *imu;
#if USE_I2C
    PLATFORM_LOGI(TAG, "使用 I2C 模式");
    icm42688_i2c_config_t i2c_cfg = {
        .scl_pin = PIN_SCL,
        .sda_pin = PIN_SDA,
        .freq_hz = 400000,
        .addr = 0x68,
    };
    imu = icm42688_create_i2c(&i2c_cfg, NULL);
#else
    PLATFORM_LOGI(TAG, "使用 SPI 模式");
    icm42688_spi_config_t spi_cfg = {
        .sclk_pin = PIN_SCLK,
        .mosi_pin = PIN_MOSI,
        .miso_pin = PIN_MISO,
        .cs_pin = PIN_CS,
        .freq_hz = 1000000,
    };
    imu = icm42688_create_spi(&spi_cfg, NULL);
#endif

    if (imu == NULL) {
        PLATFORM_LOGE(TAG, "IMU 创建失败!");
        platform_task_delete();
        return;
    }

    PLATFORM_LOGI(TAG, "IMU 初始化成功, WHO_AM_I=0x%02X", icm42688_read_id(imu));

    /* 配置中断 */
    setup_interrupt();

    /* 标定 */
    calibrate(imu);

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

    FusionBias bias;
    FusionBiasInitialise(&bias);
    FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
    biasSettings.sampleRate = (float)SAMPLE_RATE;
    biasSettings.stationaryThreshold = 2.0f;
    biasSettings.stationaryPeriod = 5.0f;
    FusionBiasSetSettings(&bias, &biasSettings);

    PLATFORM_LOGI(TAG, "=== 开始姿态解算 ===");

    /* 主循环 */
    uint32_t last_tick = platform_get_tick_count();

    while (1) {
        /* 等待中断或超时 */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SAMPLE_DT_MS));

        /* 计算 dt */
        uint32_t now = platform_get_tick_count();
        float dt = (now - last_tick) * platform_get_tick_period_ms() / 1000.0f;
        last_tick = now;

        FusionAhrsSetSamplePeriod(&ahrs, dt);

        /* 读取传感器 */
        icm42688_data_t data;
        icm42688_read_all(imu, &data);

        /* 零漂补偿 */
        float gx = data.gx - s_gyro_offset_x;
        float gy = data.gy - s_gyro_offset_y;
        float gz = data.gz - s_gyro_offset_z;

        /* AHRS 更新 */
        FusionVector gyro = {.array = {gx, gy, gz}};
        FusionVector accel = {.array = {data.ax, data.ay, data.az}};
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accel);

        /* 输出欧拉角 */
        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
        FusionEuler euler = FusionQuaternionToEuler(quat);
        printf("%.2f,%.2f,%.2f\n",
               (double)euler.angle.roll,
               (double)euler.angle.pitch,
               (double)euler.angle.yaw);
    }
}

/* ==================== 入口 ==================== */

void app_main(void)
{
    PLATFORM_LOGI(TAG, "ICM42688 Fusion AHRS (Unified Driver)");
    PLATFORM_LOGI(TAG, "Chip: %s, cores: %d",
             platform_get_chip_model(), platform_get_cpu_cores());

    platform_task_create(imu_task, "imu_task", 8192, NULL,
                        PLATFORM_TASK_PRIORITY_NORMAL, NULL);
}
