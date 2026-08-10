# ESP32-S3 ICM42688 Fusion AHRS

基于 ESP32-S3 + ICM-42688-P 六轴传感器的姿态解算系统，使用 [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) 库。

## 功能

- **Fusion AHRS 算法**：Madgwick 改进版，6轴融合
- **运行时零漂补偿**：FusionBias 实时估计陀螺仪偏移
- **加速度拒绝**：动态抑制运动加速度干扰
- **实际采样周期**：自动测量并补偿采样率偏差
- **双通信接口**：支持 SPI 和软件 I2C 模式
- **平台抽象层**：解耦架构，便于移植到不同 MCU

## 硬件

### SPI 模式

| 功能 | GPIO |
|------|------|
| SPI_SCLK | 12 |
| SPI_MOSI | 11 |
| SPI_MISO | 10 |
| SPI_CS | 9 |

### I2C 模式

| 功能 | GPIO | 说明 |
|------|------|------|
| I2C_SCL | 12 | 时钟线 (需 4.7k 上拉) |
| I2C_SDA | 10 | 数据线 (需 4.7k 上拉) |
| CS | 9 | **必须接 VDDIO (3.3V)** |
| AD0 | 11 | 地址选择 (接 GND = 0x68) |

> **重要**: I2C 模式下 CS 必须接高电平，否则芯片进入 SPI 模式。

目标芯片：ESP32-S3

## 构建与烧录

需要 [ESP-IDF v5.4.3](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html) 环境。

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COM9 flash monitor
```

## 串口输出格式

```
roll,pitch,yaw
```

单位：度

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 采样率 | 200Hz | 5ms 周期 |
| 陀螺仪量程 | +/-2000 dps | 必须与实际配置匹配 |
| AHRS 增益 | 0.5 | 融合增益 |
| 加速度拒绝 | 10 deg | 运动抑制阈值 |

## 项目结构

```
main/
├── platform.h              # 平台抽象接口 (不改)
├── platform_esp32.c        # ESP32 平台实现 (移植时重写)
├── hw_spi.h                # 硬件 SPI 接口 (不改)
├── hw_spi.c                # ESP32 SPI 实现 (移植时重写)
├── hw_i2c_soft.h           # 软件 I2C 接口 (不改)
├── hw_i2c_soft_esp32.c     # ESP32 软件 I2C 实现 (移植时重写)
├── icm42688.h/c            # ICM-42688 SPI 驱动
├── icm42688_i2c.h/c        # ICM-42688 I2C 驱动
├── hello_world_main.c      # 应用层 (不改)
├── Fusion*.c/h             # Fusion 算法库
└── CMakeLists.txt
```

## 架构设计

```
┌─────────────────────────────────────┐
│         应用层 (不改)                │
│    hello_world_main.c               │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│       设备驱动层 (不改)              │
│    icm42688_i2c.c / icm42688.c      │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│       硬件抽象层 (不改)              │
│    hw_i2c_soft.h / hw_spi.h         │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│     平台实现层 (移植时重写)          │
│  hw_i2c_soft_esp32.c / platform.c   │
└─────────────────────────────────────┘
```

### 移植到其他 MCU

只需重写两个文件：
1. `platform_xxx.c` - 延时、日志、任务管理
2. `hw_i2c_soft_xxx.c` - GPIO 操作 (SDA/SCL 读写)

应用层和驱动层完全不用修改。

## 依赖

- ESP-IDF v5.4.3
- [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) (已包含)

## 参考

- [ICM-42688-P Datasheet](https://invensense.tdk.com/wp-content/uploads/2020/11/ds-000347-icm-42688-p-datasheet.pdf)
- [Fusion 库文档](https://github.com/xioTechnologies/Fusion)
- [VOFA+ 上位机](https://vofa.plus/)
