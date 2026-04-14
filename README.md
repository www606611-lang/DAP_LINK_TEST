# MSPM0G3507 姿态显示与电机状态 Demo

这是一个基于 TI MSPM0G3507 的嵌入式固件工程。当前主程序会初始化电机 PWM、正交编码器、ICM-20948 姿态传感器、UART0 DMA 收发以及 ST7789 TFT 屏，并在屏幕上显示运行时间、串口接收内容、左右编码器计数/速度和 IMU 姿态角。

> 说明：仓库里仍保留了 SSD1306/OLED 驱动文件，但当前 CMake 构建没有编译 `modules/oled.c`。当前状态显示走的是 `modules/st7789.c`，并复用 `modules/OLED_Data.c` 里的字库数据。

## 功能概览

- 主控：TI MSPM0G3507，Cortex-M0+。
- 姿态传感器：ICM-20948，I2C0 通信，自动尝试 7 位地址 `0x69` 和 `0x68`。
- 姿态解算：roll/pitch 使用加速度计和陀螺仪互补融合，yaw 仅使用陀螺仪积分，并带静止状态下的零偏跟踪。
- 显示：ST7789 TFT，分辨率按 `320 x 170` 可视区处理，SPI1 驱动。
- 串口：UART0，`115200 8N1`，DMA 接收/发送，接收内容显示在 TFT 的 RX 行。
- 编码器：左右两路正交编码器，GPIOB 中断计数，定时计算速度。
- 电机：左右两路 PWM 输出，命令范围 `-1000` 到 `+1000`，支持滑行、刹车、方向反相和慢衰减/快衰减。
- 时间基准：TIMG0 产生 1 ms 计数，SysTick 用于阻塞延时。
- 调试/烧录：仓库包含 DAPLink + OpenOCD 配置，也保留了 CCS/Theia 的 target config。

## 项目结构

```text
.
├── CMakeLists.txt                  # 顶层 CMake 工作区
├── CMakePresets.json               # GCC/TI Arm Clang 构建预设
├── cmake/                          # 交叉编译工具链文件
├── DAP_LINK_TEST/
│   ├── CMakeLists.txt              # 固件目标 dap_link_test
│   ├── cmsis_dsp_empty.c           # main() 和主循环任务调度
│   ├── cmsis_dsp_empty.syscfg      # TI SysConfig 外设/引脚配置源文件
│   ├── app/                        # 应用层显示与串口拼帧逻辑
│   ├── common/                     # timer/delay 基础工具
│   ├── modules/                    # 传感器、屏幕、电机、编码器、UART DMA 驱动
│   ├── openocd/                    # DAPLink + OpenOCD 配置
│   └── targetConfigs/              # CCS/Theia 目标配置
├── .vscode/                        # VS Code 构建、烧录、调试配置
└── build-gcc/、build-ticlang/       # 本地构建产物，已被 .gitignore 忽略
```

## 主要模块

| 路径 | 作用 |
| --- | --- |
| `DAP_LINK_TEST/cmsis_dsp_empty.c` | 程序入口。调用 SysConfig 初始化、启动电机/编码器/IMU/LCD/UART，并在主循环中轮询任务。 |
| `DAP_LINK_TEST/app/lcd_status.c` | ST7789 状态屏页面，显示计时、RX 行、左右编码器 CNT/SPD、IMU 的 R/P/Y 或错误提示。 |
| `DAP_LINK_TEST/app/uart_display.c` | 从 UART0 DMA FIFO 取数据，遇到换行或 40 ms 空闲后刷新到屏幕。 |
| `DAP_LINK_TEST/modules/icm20948.c` | ICM-20948 I2C 驱动、初始化重试、原始数据读取、陀螺仪校准和姿态角更新。 |
| `DAP_LINK_TEST/modules/st7789.c` | ST7789 初始化、绘图、ASCII/中文显示、数字/浮点显示。 |
| `DAP_LINK_TEST/modules/uart0_dma.c` | UART0 DMA 发送和单字节流式接收 FIFO。 |
| `DAP_LINK_TEST/modules/encoder.c` | 左右正交编码器中断计数、20 ms 采样、速度/方向计算。 |
| `DAP_LINK_TEST/modules/motor.c` | 左右电机 PWM 控制，支持正反转、滑行、刹车、衰减模式和反相。 |
| `DAP_LINK_TEST/common/timer.c` | TIMG0 1 ms 系统计数。 |
| `DAP_LINK_TEST/common/delay.c` | SysTick 延时与 CPU cycle 延时。 |
| `DAP_LINK_TEST/modules/OLED_Data.c` | 字库数据，当前被 ST7789 文本显示复用。 |

## 硬件连接

外设和引脚来自 `DAP_LINK_TEST/cmsis_dsp_empty.syscfg`，构建时会由 SysConfig 生成 `ti_msp_dl_config.h/c`。

| 功能 | 外设 | 引脚 |
| --- | --- | --- |
| 外部高频晶振 HFXT | SYSCTL | `PA5` HFXIN，`PA6` HFXOUT，配置为 40 MHz |
| ICM-20948 I2C | I2C0，400 kHz | `PA0` SDA，`PA1` SCL |
| UART0 | UART0，115200 8N1 | `PA10` TX，`PA11` RX |
| UART DMA | DMA | `DMA_CH0` RX，`DMA_CH1` TX |
| ST7789 SPI | SPI1，8 MHz | `PB8` MOSI/PICO，`PB9` SCLK |
| ST7789 控制线 | GPIOB | `PB10` RES，`PB11` DC，`PB14` CS，`PB26` BLK |
| 左编码器 | GPIOB 中断 | `PB0` A，`PB1` B |
| 右编码器 | GPIOB 中断 | `PB2` A，`PB3` B |
| 左电机 PWM | TIMG6 | `PA21` CCP0，`PA22` CCP1 |
| 右电机 PWM | TIMG7 | `PA17` CCP0，`PA18` CCP1 |
| 用户 LED | GPIOB | `PB22` |
| 1 ms 定时器 | TIMG0 | 内部定时，无外部引脚 |

上电后 `main()` 会调用：

```c
Motor_SetLeft(100);
Motor_SetRight(100);
```

也就是说左右电机会默认以较小 PWM 启动。接电机和电源前请先确认小车悬空或传动部分安全，调试时可以把这两行改成 `Motor_Stop()` 或较小占空比。

## 软件依赖

建议环境：

- Windows + PowerShell。
- CMake `>= 3.22`。
- Ninja。
- Arm GNU Toolchain for `arm-none-eabi`，当前预设使用 `arm-gnu-toolchain-15.2.rel1`。
- TI MSPM0 SDK，当前预设使用 `C:/TI/mspm0_sdk_2_10_00_04`。
- TI SysConfig，当前预设使用 `C:/TI/sysconfig_1.26.2/sysconfig_cli.bat`。
- 可选：TI Arm Clang，当前预设使用 `ti-cgt-armllvm_4.0.4.LTS`。
- 可选：OpenOCD，VS Code 中用于 DAPLink 烧录/调试。

`CMakePresets.json` 和 `.vscode/settings.json` 中包含本机绝对路径。如果你的安装路径不同，需要修改这些路径，或者在配置时用 `-D` 覆盖：

```powershell
cmake --preset gcc-debug `
  -DARM_GCC_DIR="D:/path/to/arm-gnu-toolchain" `
  -DMSPM0_SDK_DIR="C:/TI/mspm0_sdk_2_10_00_04" `
  -DSYSCONFIG_CLI="C:/TI/sysconfig_1.26.2/sysconfig_cli.bat" `
  -DCMAKE_MAKE_PROGRAM="C:/path/to/ninja.exe"
```

## 构建

推荐使用 GCC 预设：

```powershell
cmake --preset gcc-debug
cmake --build --preset gcc-debug
```

构建成功后主要产物位于：

```text
build-gcc/DAP_LINK_TEST/dap_link_test.elf
build-gcc/DAP_LINK_TEST/dap_link_test.map
build-gcc/compile_commands.json
build-gcc/DAP_LINK_TEST/syscfg/
```

TI Arm Clang 预设保留为可选备用：

```powershell
cmake --preset ticlang-debug
cmake --build --preset ticlang-debug
```

对应输出后缀为 `.out`：

```text
build-ticlang/DAP_LINK_TEST/dap_link_test.out
```

## 烧录与调试

### VS Code

仓库已经提供 `.vscode/tasks.json` 和 `.vscode/launch.json`：

- `Configure (GCC)`：执行 CMake 配置。
- `Build (GCC)`：构建 `gcc-debug`，也是默认 build task。
- `Flash (OpenOCD + DAPLink)`：先构建，再通过 OpenOCD 烧录 `dap_link_test.elf`。
- `MSPM0G3507 Debug (DAPLink)`：使用 Cortex-Debug + OpenOCD 启动调试，运行到 `main`。

如果 OpenOCD 或 GDB 路径不同，请修改 `.vscode/settings.json` 中的：

```json
"mspm0.openocdPath": ".../openocd.exe",
"mspm0.openocdScriptsDir": ".../openocd/scripts",
"mspm0.armGdbPath": ".../arm-none-eabi-gdb.exe"
```

### 命令行 OpenOCD

也可以手动执行类似命令：

```powershell
openocd `
  -s "<OpenOCD scripts 目录>" `
  -f "DAP_LINK_TEST/openocd/daplink_ti_mspm0.cfg" `
  -c "program {build-gcc/DAP_LINK_TEST/dap_link_test.elf} verify reset exit"
```

`DAP_LINK_TEST/openocd/daplink_ti_mspm0.cfg` 当前内容为 CMSIS-DAP 接口、SWD 传输、1 MHz adapter speed 和 `target/ti_mspm0.cfg`。

## 运行现象

正常启动后：

1. ST7789 背光打开并清屏。
2. 屏幕顶部显示 `TFT STATUS` 和运行时间。
3. `RX:` 行显示 UART0 最近一帧接收内容。
4. 编码器区域显示左右轮的计数 `CNT` 和速度 `SPD`，单位为 pulses per second。
5. IMU 区域显示 `R:`、`P:`、`Y:`，分别为 roll、pitch、yaw。
6. UART0 会发送一次 `UART0 DMA OK\r\n`。
7. 左右电机默认以 PWM `100` 启动。

如果 ICM-20948 初始化失败，屏幕 IMU 区域会显示类似：

```text
ICM:<错误码>
ADDR68
CHK I2C
```

这表示需要优先检查 I2C 接线、上拉、电源、模块地址和传感器是否响应 `WHO_AM_I = 0xEA`。

## 关键时序与参数

| 项目 | 当前值 |
| --- | --- |
| CPU 主频 | `80 MHz` |
| I2C0 速率 | `400 kHz` |
| UART0 | `115200 8N1` |
| SPI1 LCD 速率 | `8 MHz` |
| TIMG0 系统计数 | `1 ms` |
| SysTick 延时节拍 | `1 ms` |
| ICM-20948 姿态更新周期 | `10 ms` |
| ICM-20948 初始化失败重试周期 | `1000 ms` |
| ICM-20948 陀螺仪上电校准样本数 | `500`，每样本约延时 `2 ms` |
| 编码器速度采样周期 | `20 ms` |
| LCD 编码器刷新限频 | `100 ms` |
| LCD IMU 刷新限频 | `50 ms` |
| UART 接收空闲成帧时间 | `40 ms` |
| UART DMA TX 缓冲区 | `128 bytes` |
| UART DMA RX FIFO | `128 bytes` |
| 应用层 UART 单帧缓存 | `64 bytes` |
| 电机 PWM 命令范围 | `-1000` 到 `+1000` |

## SysConfig 说明

源文件是：

```text
DAP_LINK_TEST/cmsis_dsp_empty.syscfg
```

CMake 构建时会调用 `SYSCONFIG_CLI`，把生成文件输出到构建目录，例如：

```text
build-gcc/DAP_LINK_TEST/syscfg/ti_msp_dl_config.c
build-gcc/DAP_LINK_TEST/syscfg/ti_msp_dl_config.h
build-gcc/DAP_LINK_TEST/syscfg/device_linker.lds
build-gcc/DAP_LINK_TEST/syscfg/device.lds.genlibs
build-gcc/DAP_LINK_TEST/syscfg/device.opt
```

请不要直接修改构建目录中的生成文件。需要改外设、引脚、时钟或 DMA 配置时，修改 `.syscfg` 文件，或者用 SysConfig GUI 打开后再保存。

## 常见问题

### CMake 找不到工具链或 Ninja

检查 `CMakePresets.json` 中的这些变量：

- `ARM_GCC_DIR`
- `MSPM0_SDK_DIR`
- `SYSCONFIG_CLI`
- `CMAKE_MAKE_PROGRAM`

路径和你的本机安装目录不一致时，修改预设或用 `cmake --preset gcc-debug -D...` 覆盖。

### SysConfig 生成失败

确认 `SYSCONFIG_CLI` 指向 `sysconfig_cli.bat`，并且 `MSPM0_SDK_DIR/.metadata/product.json` 存在。当前 `.syscfg` 声明的 SDK 产品版本是 `mspm0_sdk@2.10.00.04`。

### 屏幕不亮或显示异常

检查 ST7789 的供电、背光、SPI 和控制线：

- `PB8` MOSI/PICO
- `PB9` SCLK
- `PB10` RES
- `PB11` DC
- `PB14` CS
- `PB26` BLK

如果使用的屏幕可视区偏移不同，需要查看 `modules/st7789.c` 里的 `ST7789_X_OFFSET`、`ST7789_Y_OFFSET` 和 `ST7789_MADCTL_VALUE`。

### IMU 一直报错

检查：

- ICM-20948 电源和 GND 是否稳定。
- I2C0 的 `PA0` SDA、`PA1` SCL 是否接反。
- SDA/SCL 是否有合适上拉。
- 模块地址是否为 `0x68` 或 `0x69`。
- `WHO_AM_I` 是否能读到 `0xEA`。

### yaw 角会漂移

当前 yaw 只做陀螺仪积分，没有使用磁力计做航向修正，因此长时间运行会有漂移。代码里有静止检测和零偏跟踪，可以减慢静止时的漂移，但不能替代磁力计/外部航向参考。

### 串口内容没有刷新到屏幕

检查 UART 参数是否为 `115200 8N1`，TX/RX 是否交叉连接。应用层在遇到 `\n` 或接收空闲超过 `40 ms` 后才会把一帧内容刷新到 `RX:` 行。

### 编码器方向反了

可以调用：

```c
Encoder_SetInverted(ENCODER_LEFT, true);
Encoder_SetInverted(ENCODER_RIGHT, true);
```

或者调整 A/B 相接线。

### 电机方向反了

可以调用：

```c
Motor_SetLeftInverted(true);
Motor_SetRightInverted(true);
```

也可以调整驱动输入线或电机线。

## 开发备注

- 新增 `.c` 文件后，需要同步加入 `DAP_LINK_TEST/CMakeLists.txt` 的源文件列表。
- 当前 `gcc-debug` 是主要构建路径，`ticlang-debug` 只是保留的备用方案。
- `build-gcc/` 和 `build-ticlang/` 是本地产物目录，已经在 `.gitignore` 中忽略。
- `modules/oled.c` 当前未参与构建，如果后续要重新使用 SSD1306，需要在 CMake 源文件列表里加入它，并处理它和 ICM-20948 对 I2C0 的共用关系。
- 源码中有一部分注释可能在某些终端编码下显示异常；如果编辑器使用 UTF-8 打开，一般可以正常阅读。

## 版权与许可

工程中包含 TI SDK、SysConfig 生成代码和 TI 示例文件，它们按各自文件头中的版权与许可声明使用。仓库目前没有单独声明整体项目许可证；如果后续要公开发布或给他人复用，建议补充明确的 LICENSE 文件。
