/**
 * @file    hw_spi.h
 * @brief   硬件 SPI 驱动抽象接口
 * @details 平台无关的 SPI 接口定义
 *          移植到新平台时，只需实现 hw_spi.c 中的函数
 */

#ifndef __HW_SPI_H__
#define __HW_SPI_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 平台特定配置（需在实现文件中定义） ==================== */

/*
 * 以下宏定义在 hw_spi.c 中根据平台重新定义
 * 这里提供默认值，仅用于编译检查
 */
#ifndef HW_SPI_MAX_TRANSFER_SIZE
#define HW_SPI_MAX_TRANSFER_SIZE    64
#endif

/* ==================== 数据结构体 ==================== */

/**
 * @brief 硬件 SPI 配置结构体（平台无关）
 */
typedef struct {
    uint8_t host;               // SPI 主机号（平台特定含义）
    uint8_t sclk_pin;           // 时钟引脚号
    uint8_t mosi_pin;           // 主出从入引脚号
    uint8_t miso_pin;           // 主入从出引脚号
    uint8_t cs_pin;             // 片选引脚号
    uint32_t clock_speed_hz;    // 时钟频率（Hz）
    uint8_t mode;               // SPI 模式 (0-3)
} hw_spi_config_t;

/**
 * @brief 硬件 SPI 句柄结构体（平台无关）
 */
typedef struct {
    void *platform_handle;      // 平台特定句柄（如 ESP-IDF 的 spi_device_handle_t）
    uint8_t cs_pin;             // 片选引脚号
    uint8_t inited;             // 初始化标志
} hw_spi_t;

/* ==================== 默认配置 ==================== */

/*
 * 默认配置需要根据实际硬件定义
 * 这里提供示例，实际使用时在实现文件中定义
 */
#define HW_SPI_DEFAULT_CONFIG  { \
    .host = 2,                  /* SPI2 */ \
    .sclk_pin = 12,             /* GPIO12 */ \
    .mosi_pin = 11,             /* GPIO11 */ \
    .miso_pin = 10,             /* GPIO10 */ \
    .cs_pin = 9,                /* GPIO9 */ \
    .clock_speed_hz = 10000000, /* 10 MHz */ \
    .mode = 0                   /* CPOL=0, CPHA=0 */ \
}

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化硬件 SPI
 * @param   spi     SPI 句柄指针
 * @param   config  配置结构体指针
 * @return  0=成功, -1=失败
 */
int hw_spi_init(hw_spi_t *spi, const hw_spi_config_t *config);

/**
 * @brief   反初始化硬件 SPI，释放资源
 * @param   spi     SPI 句柄指针
 * @return  0=成功, -1=失败
 */
int hw_spi_deinit(hw_spi_t *spi);

/**
 * @brief   设置 CS 电平
 * @param   spi     SPI 句柄指针
 * @param   level   电平值 (0=低, 1=高)
 */
void hw_spi_set_cs(hw_spi_t *spi, uint8_t level);

/**
 * @brief   发送并接收一个字节
 * @param   spi     SPI 句柄指针
 * @param   data    发送的数据
 * @return  接收到的数据
 */
uint8_t hw_spi_transfer_byte(hw_spi_t *spi, uint8_t data);

/**
 * @brief   写入寄存器
 * @param   spi     SPI 句柄指针
 * @param   reg     寄存器地址
 * @param   value   写入的值
 */
void hw_spi_write_reg(hw_spi_t *spi, uint8_t reg, uint8_t value);

/**
 * @brief   读取寄存器
 * @param   spi     SPI 句柄指针
 * @param   reg     寄存器地址
 * @return  读取到的值
 */
uint8_t hw_spi_read_reg(hw_spi_t *spi, uint8_t reg);

/**
 * @brief   读取多个连续寄存器
 * @param   spi     SPI 句柄指针
 * @param   reg     起始寄存器地址
 * @param   buf     数据缓冲区指针
 * @param   len     读取长度
 */
void hw_spi_read_regs(hw_spi_t *spi, uint8_t reg, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __HW_SPI_H__ */
