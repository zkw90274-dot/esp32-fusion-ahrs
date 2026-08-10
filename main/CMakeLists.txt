idf_component_register(SRCS "platform_esp32.c"        # 平台抽象层
                            "hw_spi.c"                 # 硬件 SPI 驱动
                            "hw_i2c_soft_esp32.c"      # 软件 I2C 驱动
                            "icm42688.c"               # IMU SPI 驱动
                            "icm42688_i2c.c"           # IMU I2C 驱动
                            "FusionAhrs.c"             # Fusion AHRS 算法
                            "FusionBias.c"             # Fusion 零漂补偿
                            "FusionCompass.c"          # Fusion 罗盘算法
                            "hello_world_main.c"       # 主程序
                    PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_spi esp_timer
                    INCLUDE_DIRS ".")
