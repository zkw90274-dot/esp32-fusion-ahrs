@echo off
chcp 65001 >nul 2>&1

set IDF_TOOLS_PATH=E:\Tools\esp32idf\esp32_5_4\mytools
set IDF_PATH=E:\Tools\esp32idf\esp32_5_4\v5.4.3\esp-idf
set ESP_ROM_ELF_DIR=E:\Tools\esp32idf\esp32_5_4\mytools\tools\esp-rom-elfs\20241011

set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\xtensa-esp-elf\esp-14.2.0_20250730\xtensa-esp-elf\bin;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\cmake\3.30.2\bin;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\ninja\1.12.1;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\ccache\4.8\bin;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\dfu-util\0.11\bin;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\esp32ulp-elf\esp32ulp-elf-2.38_20240113\esp32ulp-elf\bin;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\openocd-esp32\v0.12.0-esp32-20241016\openocd-esp32\bin;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\idf-git\2.39.2\cmd;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\idf-exe\1.0.3;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\xtensa-esp-elf-gdb\16.3_20250913\xtensa-esp-elf-gdb\bin;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\riscv32-esp-elf\esp-14.2.0_20250730\riscv32-esp-elf\bin;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\mytools\tools\riscv32-esp-elf-gdb\16.3_20250913\riscv32-esp-elf-gdb\bin;%PATH%"
set "PATH=C:\Users\wwwzk\.espressif\python_env\idf5.4_py3.14_env\Scripts;%PATH%"
set "PATH=E:\Tools\esp32idf\esp32_5_4\v5.4.3\esp-idf\components\espcoredump;E:\Tools\esp32idf\esp32_5_4\v5.4.3\esp-idf\components\partition_table;E:\Tools\esp32idf\esp32_5_4\v5.4.3\esp-idf\components\app_update;%PATH%"

cd /d E:\Project\espidf_prj\Fusion

idf.py -p COM9 flash monitor
