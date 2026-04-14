# DAP_LINK_TEST 固件说明

`DAP_LINK_TEST` 是当前 MSPM0G3507 固件目标，最终生成的程序名为 `dap_link_test`。它负责初始化 SysConfig 外设配置，并运行电机、编码器、ICM-20948、UART0 DMA 和 ST7789 状态屏任务。

更完整的构建、烧录、引脚和排障说明请看仓库根目录的 `README.md`。

## 运行模块

- `cmsis_dsp_empty.c`：程序入口和主循环调度。
- `app/lcd_status.c`：ST7789 TFT 状态屏，显示运行时间、UART RX、左右编码器计数/速度和 IMU 姿态角。
- `app/uart_display.c`：UART0 DMA 接收拼帧，遇到换行或 40 ms 空闲后刷新到屏幕。
- `modules/icm20948.c`：ICM-20948 I2C 驱动、初始化重试、陀螺仪校准和 roll/pitch/yaw 更新。
- `modules/st7789.c`：ST7789 屏幕驱动和绘图/文字接口。
- `modules/uart0_dma.c`：UART0 DMA 收发。
- `modules/encoder.c`：左右正交编码器计数和速度计算。
- `modules/motor.c`：左右电机 PWM 控制。
- `common/timer.c`、`common/delay.c`：1 ms 计数和延时工具。

## 构建

从仓库根目录执行：

```powershell
cmake --preset gcc-debug
cmake --build --preset gcc-debug
```

GCC 构建产物：

```text
build-gcc/DAP_LINK_TEST/dap_link_test.elf
```

可选 TI Arm Clang 备用预设：

```powershell
cmake --preset ticlang-debug
cmake --build --preset ticlang-debug
```

对应产物：

```text
build-ticlang/DAP_LINK_TEST/dap_link_test.out
```

## 备注

- 当前状态屏使用 ST7789，不使用 `modules/oled.c` 直接驱动 SSD1306。
- `modules/OLED_Data.c` 仍会参与构建，因为 ST7789 文本显示复用了其中的字库。
- 上电初始化后默认执行 `Motor_SetLeft(100)` 和 `Motor_SetRight(100)`，接电机调试前请注意安全。
