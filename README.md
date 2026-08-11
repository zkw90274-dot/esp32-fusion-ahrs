# ESP32-S3 ICM42688 Fusion AHRS

基于 ESP32-S3 + ICM-42688-P 六轴传感器的姿态解算系统，使用 [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) 库。

## 功能

- **Fusion AHRS 算法**：Madgwick 改进版，6轴融合
- **运行时零漂补偿**：FusionBias 实时估计陀螺仪偏移
- **加速度拒绝**：动态抑制运动加速度干扰
- **统一驱动架构**：支持 SPI 和 I2C 无缝切换
- **HAL 抽象层**：换 MCU 只需实现 HAL 接口，驱动层和应用层零修改

## 架构设计

```
┌─────────────────────────────────────────────────────────┐
│                    app_main.c (应用层)                    │
│  GPIO 宏定义 → IMU 创建 → AHRS 计算 → 欧拉角输出        │
└─────────────────────────┬───────────────────────────────┘
                          │ icm42688_create_spi/i2c()
                          │ icm42688_read_all()
┌─────────────────────────▼───────────────────────────────┐
│              icm42688_unified.c (驱动层)                  │
│  传感器初始化 → 寄存器读写 → 数据转换                     │
│  通过 HAL 接口访问硬件，零平台依赖                        │
└────────┬────────────────────────────────┬───────────────┘
         │ hal_spi_read_regs()            │ hal_i2c_read_regs()
┌────────▼────────┐            ┌─────────▼─────────┐
│ hal_spi_esp32.c │            │ hal_i2c_esp32.c   │
│ ESP32 SPI 实现   │            │ ESP32 I2C 实现     │
└─────────────────┘            └───────────────────┘
```

## 目录结构

```
main/
├── hal/                          # HAL 抽象层（换 MCU 只改这里）
│   ├── hal_i2c.h                 # I2C 接口定义
│   ├── hal_spi.h                 # SPI 接口定义
│   └── esp32/
│       ├── hal_i2c_esp32.c       # ESP32 I2C 实现
│       └── hal_spi_esp32.c       # ESP32 SPI 实现
│
├── icm42688_unified.h            # 统一传感器驱动接口
├── icm42688_unified.c            # 统一传感器驱动实现
│
├── platform.h / platform_esp32.c # 平台抽象层
├── Fusion*.h / Fusion*.c         # Fusion 算法库
└── app_main.c                    # 应用层（零平台依赖）
```

## 硬件接线

### SPI 模式

| ICM-42688-P | ESP32-S3 | 说明 |
|-------------|----------|------|
| VDD | 3.3V | 电源 |
| GND | GND | 地线 |
| SCLK | GPIO12 | SPI 时钟 |
| MOSI (SDI) | GPIO11 | 主出从入 |
| MISO (SDO) | GPIO10 | 主入从出 |
| CS | GPIO9 | 片选 |
| AD0 | GND | 地址 = 0x68 |
| INT1 | GPIO13 | 数据就绪中断（可选） |

### I2C 模式

| ICM-42688-P | ESP32-S3 | 说明 |
|-------------|----------|------|
| VDD | 3.3V | 电源 |
| GND | GND | 地线 |
| SCL | GPIO12 | 时钟线 (需 4.7k 上拉) |
| SDA | GPIO10 | 数据线 (需 4.7k 上拉) |
| CS | 3.3V | **必须接高电平** |
| AD0 | GND | 地址 = 0x68 |
| INT1 | GPIO13 | 数据就绪中断（可选） |

## 快速开始

### 1. 编译

```bash
idf.py build
```

### 2. 烧录

```bash
idf.py -p COMx flash
```

### 3. 监听

```bash
idf.py -p COMx monitor
```

### 4. 输出示例

```
-0.53,-0.80,0.01
-0.53,-0.80,0.01
-0.53,-0.80,0.01
```

格式：`roll,pitch,yaw`（单位：度）

## 配置说明

### 接口选择

在 `app_main.c` 中修改：

```c
/* 接口选择：0=SPI, 1=I2C */
#define USE_I2C         0
```

### 采样参数

```c
#define SAMPLE_RATE     100     /* Hz */
#define SAMPLE_DT_MS    10      /* 轮询周期 (ms) */
```

### Fusion 参数

```c
#define GYRO_RANGE      2000.0f     /* 陀螺仪量程 (dps) */
#define AHRS_GAIN       0.3f        /* 融合增益 (0-1) */
#define ACC_REJECTION   10.0f       /* 加速度拒绝阈值 (度) */
```

## 移植到其他 MCU

只需 3 步：

### 1. 复制 HAL 目录

```bash
cp -r main/hal/esp32 main/hal/stm32
```

### 2. 实现 HAL 接口

```c
// hal/stm32/hal_spi_stm32.c
hal_spi_t *hal_spi_create(const hal_spi_config_t *config) { ... }
void hal_spi_destroy(hal_spi_t *spi) { ... }
int hal_spi_write_reg(hal_spi_t *spi, uint8_t reg, uint8_t value) { ... }
int hal_spi_read_reg(hal_spi_t *spi, uint8_t reg, uint8_t *value) { ... }
int hal_spi_read_regs(hal_spi_t *spi, uint8_t reg, uint8_t *buf, uint16_t len) { ... }
```

### 3. 更新 CMakeLists.txt

```cmake
idf_component_register(SRCS
    "platform_stm32.c"
    "hal/stm32/hal_spi_stm32.c"
    "hal/stm32/hal_i2c_stm32.c"
    "icm42688_unified.c"
    ...
)
```

**驱动层和应用层零修改！**

## 依赖

- ESP-IDF v5.4+
- [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion)（已包含）

## 参考

- [ICM-42688-P Datasheet](https://invensense.tdk.com/wp-content/uploads/2020/11/ds-000347-icm-42688-p-datasheet.pdf)
- [Fusion 库文档](https://github.com/xioTechnologies/Fusion)

## 许可证

MIT License
