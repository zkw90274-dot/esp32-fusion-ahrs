/**
 * @file    platform_esp32.c
 * @brief   ESP32 平台抽象层实现
 * @details 实现 platform.h 定义的接口，基于 ESP-IDF
 */

#include "platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "platform";

/* ==================== 私有变量 ==================== */

static platform_log_level_t s_log_level = PLATFORM_LOG_INFO;

/* ==================== 延时函数实现 ==================== */

void platform_delay_ms(uint32_t ms)
{
    /* 非零延时至少等待一个 tick，避免 vTaskDelay(0) 导致任务不阻塞 */
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks == 0 && ms > 0) {
        ticks = 1;
    }
    vTaskDelay(ticks);
}

void platform_delay_us(uint32_t us)
{
    ets_delay_us(us);
}

/* ==================== 时间函数实现 ==================== */

uint32_t platform_get_tick_count(void)
{
    return xTaskGetTickCount();
}

float platform_get_tick_period_ms(void)
{
    return (float)portTICK_PERIOD_MS;
}

uint32_t platform_get_uptime_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ==================== 任务管理实现 ==================== */

int platform_task_create(platform_task_func_t func,
                        const char *name,
                        uint32_t stack_size,
                        void *arg,
                        platform_task_priority_t priority,
                        void **handle)
{
    TaskHandle_t task_handle = NULL;
    BaseType_t ret;

    ret = xTaskCreate(
        (TaskFunction_t)func,
        name,
        stack_size,  // ESP-IDF 的 xTaskCreate 栈参数单位是字节
        arg,
        (UBaseType_t)priority,
        &task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task: %s", name);
        return -1;
    }

    if (handle != NULL) {
        *handle = (void*)task_handle;
    }

    return 0;
}

void platform_task_delete(void)
{
    vTaskDelete(NULL);
}

void platform_task_yield(void)
{
    taskYIELD();
}

/* ==================== 互斥锁实现 ==================== */

platform_mutex_t platform_mutex_create(void)
{
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    return (platform_mutex_t)mutex;
}

int platform_mutex_lock(platform_mutex_t mutex, uint32_t timeout_ms)
{
    if (mutex == NULL) {
        return -1;
    }

    TickType_t ticks;
    if (timeout_ms == 0xFFFFFFFF) {
        ticks = portMAX_DELAY;
    } else {
        ticks = pdMS_TO_TICKS(timeout_ms);
    }

    if (xSemaphoreTake((SemaphoreHandle_t)mutex, ticks) == pdTRUE) {
        return 0;
    }

    return -1;
}

int platform_mutex_unlock(platform_mutex_t mutex)
{
    if (mutex == NULL) {
        return -1;
    }

    if (xSemaphoreGive((SemaphoreHandle_t)mutex) == pdTRUE) {
        return 0;
    }

    return -1;
}

void platform_mutex_delete(platform_mutex_t mutex)
{
    if (mutex != NULL) {
        vSemaphoreDelete((SemaphoreHandle_t)mutex);
    }
}

/* ==================== 日志函数实现 ==================== */

void platform_log_set_level(platform_log_level_t level)
{
    s_log_level = level;

    /* 同时设置 ESP-IDF 日志级别 */
    switch (level) {
        case PLATFORM_LOG_ERROR:
            esp_log_level_set("*", ESP_LOG_ERROR);
            break;
        case PLATFORM_LOG_WARN:
            esp_log_level_set("*", ESP_LOG_WARN);
            break;
        case PLATFORM_LOG_INFO:
            esp_log_level_set("*", ESP_LOG_INFO);
            break;
        case PLATFORM_LOG_DEBUG:
            esp_log_level_set("*", ESP_LOG_DEBUG);
            break;
        case PLATFORM_LOG_VERBOSE:
            esp_log_level_set("*", ESP_LOG_VERBOSE);
            break;
    }
}

void platform_log(platform_log_level_t level, const char *tag, const char *fmt, ...)
{
    if (level > s_log_level) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    platform_logv(level, tag, fmt, args);
    va_end(args);
}

void platform_logv(platform_log_level_t level, const char *tag, const char *fmt, va_list args)
{
    if (level > s_log_level) {
        return;
    }

    /* 映射到 ESP-IDF 日志级别 */
    esp_log_level_t esp_level;
    switch (level) {
        case PLATFORM_LOG_ERROR:
            esp_level = ESP_LOG_ERROR;
            break;
        case PLATFORM_LOG_WARN:
            esp_level = ESP_LOG_WARN;
            break;
        case PLATFORM_LOG_INFO:
            esp_level = ESP_LOG_INFO;
            break;
        case PLATFORM_LOG_DEBUG:
            esp_level = ESP_LOG_DEBUG;
            break;
        case PLATFORM_LOG_VERBOSE:
            esp_level = ESP_LOG_VERBOSE;
            break;
        default:
            esp_level = ESP_LOG_INFO;
            break;
    }

    /* 使用 ESP-IDF 日志系统 */
    esp_log_writev(esp_level, tag, fmt, args);
}

/* ==================== 系统信息实现 ==================== */

const char* platform_get_chip_model(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    switch (chip_info.model) {
        case CHIP_ESP32:
            return "ESP32";
        case CHIP_ESP32S2:
            return "ESP32-S2";
        case CHIP_ESP32S3:
            return "ESP32-S3";
        case CHIP_ESP32C3:
            return "ESP32-C3";
        case CHIP_ESP32C2:
            return "ESP32-C2";
        case CHIP_ESP32C6:
            return "ESP32-C6";
        case CHIP_ESP32H2:
            return "ESP32-H2";
        default:
            return "Unknown";
    }
}

uint8_t platform_get_chip_revision(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    return chip_info.revision;
}

uint8_t platform_get_cpu_cores(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    return chip_info.cores;
}

uint32_t platform_get_free_heap_size(void)
{
    return esp_get_free_heap_size();
}

/* ==================== 系统控制实现 ==================== */

void platform_restart(void)
{
    ESP_LOGI(TAG, "Restarting system...");
    esp_restart();
}

/* 临界区需要静态初始化的 spinlock */
static portMUX_TYPE s_critical_mux = portMUX_INITIALIZER_UNLOCKED;

void platform_enter_critical(void)
{
    portENTER_CRITICAL(&s_critical_mux);
}

void platform_exit_critical(void)
{
    portEXIT_CRITICAL(&s_critical_mux);
}
