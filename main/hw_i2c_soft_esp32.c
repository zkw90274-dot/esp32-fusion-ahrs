/**
 * @file    hw_i2c_soft_esp32.c
 * @brief   ESP32 软件 I2C 驱动实现
 * @details 基于 GPIO 模拟 I2C 协议
 *          适用于 ICM-42688-P 等传感器
 */

#include "hw_i2c_soft.h"
#include "platform.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "hw_i2c_soft";

/* ==================== 内部延时函数 ==================== */

/**
 * @brief   微秒级延时
 * @param   us  延时时间（微秒）
 */
static inline void i2c_delay_us(uint32_t us)
{
    /* 使用 platform 抽象层的延时函数 */
    platform_delay_us(us);
}

/* ==================== GPIO 操作函数 ==================== */

/**
 * @brief   设置 SCL 电平
 */
static void i2c_set_scl(hw_i2c_soft_t *i2c, uint8_t level)
{
    gpio_set_level(i2c->scl_pin, level);
}

/**
 * @brief   设置 SDA 电平
 */
static void i2c_set_sda(hw_i2c_soft_t *i2c, uint8_t level)
{
    gpio_set_level(i2c->sda_pin, level);
}

/**
 * @brief   读取 SDA 电平
 */
static uint8_t i2c_read_sda(hw_i2c_soft_t *i2c)
{
    return gpio_get_level(i2c->sda_pin);
}

/**
 * @brief   SDA 设为输出模式
 */
static void i2c_sda_output(hw_i2c_soft_t *i2c)
{
    gpio_set_direction(i2c->sda_pin, GPIO_MODE_OUTPUT);
}

/**
 * @brief   SDA 设为输入模式
 */
static void i2c_sda_input(hw_i2c_soft_t *i2c)
{
    gpio_set_direction(i2c->sda_pin, GPIO_MODE_INPUT);
}

/* ==================== I2C 协议实现 ==================== */

/**
 * @brief   I2C 起始条件
 */
static void i2c_start(hw_i2c_soft_t *i2c)
{
    i2c_sda_output(i2c);
    i2c_set_sda(i2c, 1);
    i2c_set_scl(i2c, 1);
    i2c_delay_us(i2c->delay_us);
    i2c_set_sda(i2c, 0);  /* SDA 下降沿 */
    i2c_delay_us(i2c->delay_us);
    i2c_set_scl(i2c, 0);
    i2c_delay_us(i2c->delay_us);
}

/**
 * @brief   I2C 停止条件
 */
static void i2c_stop(hw_i2c_soft_t *i2c)
{
    i2c_sda_output(i2c);
    i2c_set_sda(i2c, 0);
    i2c_set_scl(i2c, 1);
    i2c_delay_us(i2c->delay_us);
    i2c_set_sda(i2c, 1);  /* SDA 上升沿 */
    i2c_delay_us(i2c->delay_us);
}

/**
 * @brief   I2C 发送 ACK
 */
static void i2c_send_ack(hw_i2c_soft_t *i2c)
{
    i2c_sda_output(i2c);
    i2c_set_sda(i2c, 0);  /* SDA 低 = ACK */
    i2c_set_scl(i2c, 1);
    i2c_delay_us(i2c->delay_us);
    i2c_set_scl(i2c, 0);
    i2c_delay_us(i2c->delay_us);
}

/**
 * @brief   I2C 发送 NACK
 */
static void i2c_send_nack(hw_i2c_soft_t *i2c)
{
    i2c_sda_output(i2c);
    i2c_set_sda(i2c, 1);  /* SDA 高 = NACK */
    i2c_set_scl(i2c, 1);
    i2c_delay_us(i2c->delay_us);
    i2c_set_scl(i2c, 0);
    i2c_delay_us(i2c->delay_us);
}

/**
 * @brief   I2C 等待 ACK
 * @return  0=收到 ACK, -1=收到 NACK
 */
static int i2c_wait_ack(hw_i2c_soft_t *i2c)
{
    uint8_t ack;

    i2c_sda_input(i2c);  /* SDA 设为输入 */
    i2c_set_scl(i2c, 1);
    i2c_delay_us(i2c->delay_us);
    ack = i2c_read_sda(i2c);
    i2c_set_scl(i2c, 0);
    i2c_delay_us(i2c->delay_us);

    return (ack == 0) ? 0 : -1;
}

/**
 * @brief   I2C 写入一个字节
 * @param   data    要写入的数据
 * @return  0=收到 ACK, -1=收到 NACK
 */
static int i2c_write_byte(hw_i2c_soft_t *i2c, uint8_t data)
{
    i2c_sda_output(i2c);

    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80)
            i2c_set_sda(i2c, 1);
        else
            i2c_set_sda(i2c, 0);
        data <<= 1;
        i2c_set_scl(i2c, 1);
        i2c_delay_us(i2c->delay_us);
        i2c_set_scl(i2c, 0);
        i2c_delay_us(i2c->delay_us);
    }

    return i2c_wait_ack(i2c);
}

/**
 * @brief   I2C 读取一个字节
 * @param   ack     是否发送 ACK (1=ACK, 0=NACK)
 * @return  读取到的数据
 */
static uint8_t i2c_read_byte(hw_i2c_soft_t *i2c, uint8_t ack)
{
    uint8_t data = 0;

    i2c_sda_input(i2c);  /* SDA 设为输入 */

    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        i2c_set_scl(i2c, 1);
        i2c_delay_us(i2c->delay_us);
        if (i2c_read_sda(i2c)) {
            data |= 0x01;
        }
        i2c_set_scl(i2c, 0);
        i2c_delay_us(i2c->delay_us);
    }

    if (ack)
        i2c_send_ack(i2c);
    else
        i2c_send_nack(i2c);

    return data;
}

/* ==================== 公共 API 实现 ==================== */

int hw_i2c_soft_init(hw_i2c_soft_t *i2c, const hw_i2c_soft_config_t *config)
{
    if (i2c == NULL || config == NULL) {
        return -1;
    }

    /* 保存配置 */
    i2c->scl_pin = config->scl_pin;
    i2c->sda_pin = config->sda_pin;
    i2c->delay_us = config->delay_us;
    i2c->addr = config->addr;

    /* 配置 SCL 引脚为输出 */
    gpio_config_t scl_cfg = {
        .pin_bit_mask = (1ULL << config->scl_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&scl_cfg);

    /* 配置 SDA 引脚为开漏输出 */
    gpio_config_t sda_cfg = {
        .pin_bit_mask = (1ULL << config->sda_pin),
        .mode = GPIO_MODE_OUTPUT_OD,  /* 开漏模式 */
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&sda_cfg);

    /* 设置初始状态：SCL 和 SDA 都为高 */
    gpio_set_level(config->scl_pin, 1);
    gpio_set_level(config->sda_pin, 1);

    i2c->inited = 1;

    PLATFORM_LOGI(TAG, "Soft I2C init OK (SCL=GPIO%d, SDA=GPIO%d, addr=0x%02X, delay=%dus)",
             config->scl_pin, config->sda_pin, config->addr, config->delay_us);

    return 0;
}

int hw_i2c_soft_deinit(hw_i2c_soft_t *i2c)
{
    if (i2c == NULL || !i2c->inited) {
        return -1;
    }

    /* 复位引脚 */
    gpio_reset_pin(i2c->scl_pin);
    gpio_reset_pin(i2c->sda_pin);

    i2c->inited = 0;

    PLATFORM_LOGI(TAG, "Soft I2C deinit OK");

    return 0;
}

int hw_i2c_soft_write_reg(hw_i2c_soft_t *i2c, uint8_t reg, uint8_t value)
{
    if (i2c == NULL || !i2c->inited) {
        return -1;
    }

    i2c_start(i2c);

    /* 发送从机地址 + 写 */
    if (i2c_write_byte(i2c, i2c->addr << 1) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 发送寄存器地址 */
    if (i2c_write_byte(i2c, reg) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 发送数据 */
    if (i2c_write_byte(i2c, value) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    i2c_stop(i2c);

    return 0;
}

int hw_i2c_soft_read_reg(hw_i2c_soft_t *i2c, uint8_t reg, uint8_t *value)
{
    if (i2c == NULL || !i2c->inited || value == NULL) {
        return -1;
    }

    i2c_start(i2c);

    /* 发送从机地址 + 写 */
    if (i2c_write_byte(i2c, i2c->addr << 1) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 发送寄存器地址 */
    if (i2c_write_byte(i2c, reg) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 重复起始条件 */
    i2c_start(i2c);

    /* 发送从机地址 + 读 */
    if (i2c_write_byte(i2c, (i2c->addr << 1) | 0x01) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 读取数据（发送 NACK） */
    *value = i2c_read_byte(i2c, 0);

    i2c_stop(i2c);

    return 0;
}

int hw_i2c_soft_read_regs(hw_i2c_soft_t *i2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (i2c == NULL || !i2c->inited || buf == NULL || len == 0) {
        return -1;
    }

    i2c_start(i2c);

    /* 发送从机地址 + 写 */
    if (i2c_write_byte(i2c, i2c->addr << 1) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 发送寄存器地址 */
    if (i2c_write_byte(i2c, reg) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 重复起始条件 */
    i2c_start(i2c);

    /* 发送从机地址 + 读 */
    if (i2c_write_byte(i2c, (i2c->addr << 1) | 0x01) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 读取数据 */
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = i2c_read_byte(i2c, (i < len - 1) ? 1 : 0);  /* 最后一个发 NACK */
    }

    i2c_stop(i2c);

    return 0;
}

int hw_i2c_soft_write_regs(hw_i2c_soft_t *i2c, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    if (i2c == NULL || !i2c->inited || buf == NULL || len == 0) {
        return -1;
    }

    i2c_start(i2c);

    /* 发送从机地址 + 写 */
    if (i2c_write_byte(i2c, i2c->addr << 1) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 发送寄存器地址 */
    if (i2c_write_byte(i2c, reg) != 0) {
        i2c_stop(i2c);
        return -1;
    }

    /* 发送数据 */
    for (uint16_t i = 0; i < len; i++) {
        if (i2c_write_byte(i2c, buf[i]) != 0) {
            i2c_stop(i2c);
            return -1;
        }
    }

    i2c_stop(i2c);

    return 0;
}

int hw_i2c_soft_probe(hw_i2c_soft_t *i2c)
{
    if (i2c == NULL || !i2c->inited) {
        return -1;
    }

    i2c_start(i2c);

    /* 发送从机地址 + 写 */
    if (i2c_write_byte(i2c, i2c->addr << 1) != 0) {
        i2c_stop(i2c);
        return -1;  /* 设备不存在 */
    }

    i2c_stop(i2c);

    return 0;  /* 设备存在 */
}
