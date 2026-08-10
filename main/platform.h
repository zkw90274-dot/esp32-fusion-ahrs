/**
 * @file    platform.h
 * @brief   平台抽象层接口定义
 * @details 提供平台无关的 API，便于移植到不同 MCU
 *
 * 支持的功能：
 * - 延时函数
 * - 任务管理
 * - 日志输出
 * - 时间函数
 * - 互斥锁
 */

#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 日志级别定义 ==================== */

typedef enum {
    PLATFORM_LOG_ERROR = 0,
    PLATFORM_LOG_WARN,
    PLATFORM_LOG_INFO,
    PLATFORM_LOG_DEBUG,
    PLATFORM_LOG_VERBOSE
} platform_log_level_t;

/* ==================== 任务优先级定义 ==================== */

typedef enum {
    PLATFORM_TASK_PRIORITY_LOW = 1,
    PLATFORM_TASK_PRIORITY_NORMAL = 5,
    PLATFORM_TASK_PRIORITY_HIGH = 10,
    PLATFORM_TASK_PRIORITY_CRITICAL = 15
} platform_task_priority_t;

/* ==================== 互斥锁类型 ==================== */

typedef void* platform_mutex_t;

/* ==================== 任务函数类型 ==================== */

typedef void (*platform_task_func_t)(void *arg);

/* ==================== 延时函数 ==================== */

/**
 * @brief   毫秒延时
 * @param   ms  延时时间（毫秒）
 */
void platform_delay_ms(uint32_t ms);

/**
 * @brief   微秒延时
 * @param   us  延时时间（微秒）
 */
void platform_delay_us(uint32_t us);

/* ==================== 时间函数 ==================== */

/**
 * @brief   获取系统 tick 计数
 * @return  当前 tick 计数
 */
uint32_t platform_get_tick_count(void);

/**
 * @brief   获取 tick 周期（毫秒）
 * @return  每个 tick 对应的毫秒数
 */
float platform_get_tick_period_ms(void);

/**
 * @brief   获取系统运行时间（毫秒）
 * @return  系统启动后的毫秒数
 */
uint32_t platform_get_uptime_ms(void);

/* ==================== 任务管理 ==================== */

/**
 * @brief   创建任务
 * @param   func        任务函数
 * @param   name        任务名称
 * @param   stack_size  栈大小（字节）
 * @param   arg         任务参数
 * @param   priority    任务优先级
 * @param   handle      任务句柄（输出，可为 NULL）
 * @return  0=成功, -1=失败
 */
int platform_task_create(platform_task_func_t func,
                        const char *name,
                        uint32_t stack_size,
                        void *arg,
                        platform_task_priority_t priority,
                        void **handle);

/**
 * @brief   删除当前任务
 */
void platform_task_delete(void);

/**
 * @brief   任务让出 CPU
 */
void platform_task_yield(void);

/* ==================== 互斥锁 ==================== */

/**
 * @brief   创建互斥锁
 * @return  互斥锁句柄，NULL 表示失败
 */
platform_mutex_t platform_mutex_create(void);

/**
 * @brief   获取互斥锁
 * @param   mutex   互斥锁句柄
 * @param   timeout_ms  超时时间（毫秒），0xFFFFFFFF 表示永久等待
 * @return  0=成功, -1=超时
 */
int platform_mutex_lock(platform_mutex_t mutex, uint32_t timeout_ms);

/**
 * @brief   释放互斥锁
 * @param   mutex   互斥锁句柄
 * @return  0=成功, -1=失败
 */
int platform_mutex_unlock(platform_mutex_t mutex);

/**
 * @brief   删除互斥锁
 * @param   mutex   互斥锁句柄
 */
void platform_mutex_delete(platform_mutex_t mutex);

/* ==================== 日志函数 ==================== */

/**
 * @brief   设置日志级别
 * @param   level   日志级别
 */
void platform_log_set_level(platform_log_level_t level);

/**
 * @brief   输出日志
 * @param   level   日志级别
 * @param   tag     标签
 * @param   fmt     格式字符串
 * @param   ...     可变参数
 */
void platform_log(platform_log_level_t level, const char *tag, const char *fmt, ...);

/**
 * @brief   输出日志（va_list 版本）
 * @param   level   日志级别
 * @param   tag     标签
 * @param   fmt     格式字符串
 * @param   args    va_list 参数
 */
void platform_logv(platform_log_level_t level, const char *tag, const char *fmt, va_list args);

/* 便捷宏 */
#define PLATFORM_LOGE(tag, fmt, ...) platform_log(PLATFORM_LOG_ERROR, tag, fmt, ##__VA_ARGS__)
#define PLATFORM_LOGW(tag, fmt, ...) platform_log(PLATFORM_LOG_WARN, tag, fmt, ##__VA_ARGS__)
#define PLATFORM_LOGI(tag, fmt, ...) platform_log(PLATFORM_LOG_INFO, tag, fmt, ##__VA_ARGS__)
#define PLATFORM_LOGD(tag, fmt, ...) platform_log(PLATFORM_LOG_DEBUG, tag, fmt, ##__VA_ARGS__)
#define PLATFORM_LOGV(tag, fmt, ...) platform_log(PLATFORM_LOG_VERBOSE, tag, fmt, ##__VA_ARGS__)

/* ==================== 系统信息 ==================== */

/**
 * @brief   获取芯片型号
 * @return  芯片型号字符串
 */
const char* platform_get_chip_model(void);

/**
 * @brief   获取芯片版本
 * @return  芯片版本号
 */
uint8_t platform_get_chip_revision(void);

/**
 * @brief   获取 CPU 核心数
 * @return  CPU 核心数
 */
uint8_t platform_get_cpu_cores(void);

/**
 * @brief   获取空闲堆内存大小
 * @return  空闲堆内存（字节）
 */
uint32_t platform_get_free_heap_size(void);

/* ==================== 系统控制 ==================== */

/**
 * @brief   系统重启
 */
void platform_restart(void);

/**
 * @brief   进入临界区（禁用中断）
 */
void platform_enter_critical(void);

/**
 * @brief   退出临界区（使能中断）
 */
void platform_exit_critical(void);

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_H__ */
