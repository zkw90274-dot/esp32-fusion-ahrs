# ESP32-S3 ICM42688 Fusion AHRS

基于 ESP32-S3 + ICM-42688-P 六轴传感器的姿态解算系统，使用 [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) 库。

## 功能

- **Fusion AHRS 算法**：Madgwick 改进版，6轴融合
- **运行时零漂补偿**：FusionBias 实时估计陀螺仪偏移
- **加速度拒绝**：动态抑制运动加速度干扰
- **实际采样周期**：自动测量并补偿采样率偏差
- **VOFA 上位机输出**：FireWater 协议

## 硬件

| 功能 | GPIO |
|------|------|
| SPI_SCLK | 12 |
| SPI_MOSI | 11 |
| SPI_MISO | 10 |
| SPI_CS | 9 |

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

单位：度（°）

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 采样率 | 200Hz | 5ms 周期 |
| 陀螺仪量程 | ±2000 dps | 必须与实际配置匹配 |
| AHRS 增益 | 0.5 | 融合增益 |
| 加速度拒绝 | 10° | 运动抑制阈值 |

## 项目结构

```
main/
├── hello_world_main.c   # 应用入口，Fusion AHRS 主循环
├── hw_spi.c/h           # ESP32 硬件 SPI 驱动
├── icm42688.c/h         # ICM-42688-P 传感器驱动
├── Fusion*.c/h          # Fusion 库源码
└── CMakeLists.txt
```

## 依赖

- ESP-IDF v5.4.3
- [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) (已包含)

## 参考

- [ICM-42688-P Datasheet](https://invensense.tdk.com/wp-content/uploads/2020/11/ds-000347-icm-42688-p-datasheet.pdf)
- [Fusion 库文档](https://github.com/xioTechnologies/Fusion)
- [VOFA+ 上位机](https://vofa.plus/)
