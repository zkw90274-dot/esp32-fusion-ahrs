idf_component_register(SRCS
                            # 平台抽象层
                            "platform_esp32.c"

                            # HAL 层（可替换为其他 MCU 实现）
                            "hal/esp32/hal_spi_esp32.c"
                            "hal/esp32/hal_i2c_esp32.c"

                            # 传感器驱动（统一接口）
                            "icm42688_unified.c"

                            # Fusion 算法库
                            "FusionAhrs.c"
                            "FusionBias.c"
                            "FusionCompass.c"

                            # 应用层
                            "app_main.c"
                    PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_spi esp_driver_i2c esp_timer
                    INCLUDE_DIRS "." "hal")
