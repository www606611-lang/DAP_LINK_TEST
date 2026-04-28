# MSPM0 K230 Rectangle Tracking Platform

这是一个基于 `TI MSPM0G3507` 的下位机项目，当前目标是配合 `K230` 上位机完成矩形目标识别、串口传输、双轴步进电机追踪，以及 LCD 状态显示。

当前工程已经从最初的传感器/显示 Demo，演进为一套可联调的视觉追踪固件：
- `K230` 识别矩形并通过 `UART2` 发送中心点坐标
- `MSPM0` 解析数据包，计算误差
- 外环 `PID` 根据误差输出两轴步进速度
- 两个闭环步进电机通过 `CAN` 总线控制
- `ST7789` LCD 实时显示 `K230 / IMU / 编码器 / 步进 RPM`

## 当前功能

- `K230 -> MSPM0` 串口数据链路已打通
- 下位机支持矩形中心坐标追踪
- 步进电机采用双轴速度控制
- 步进电机控制链路带有基础 `PID` 外环框架，后续可继续扩展速度环、位置环
- `LCD` 已做过刷新优化，支持 `SPI DMA`
- 当 `K230` 串口数据超时丢失时，下位机会自动将目标判为失效并停机

## 数据流

```text
K230矩形检测
  -> UART2 发包 @valid,cx,cy#
  -> MSPM0 串口解析
  -> 计算 err_x / err_y
  -> track_control 外环 PID
  -> CAN 速度命令
  -> 双轴闭环步进电机
  -> LCD 显示当前状态
```

## 主要目录

```text
.
├─ CMakeLists.txt
├─ CMakePresets.json
├─ README.md
├─ 矩形识别+串口发.py              # K230/OpenMV 风格脚本
├─ cmake/
├─ build-gcc/                      # 本地构建输出，已忽略
├─ build-ticlang/                  # 本地构建输出，已忽略
└─ DAP_LINK_TEST/
   ├─ CMakeLists.txt
   ├─ cmsis_dsp_empty.c            # main 与主循环调度
   ├─ cmsis_dsp_empty.syscfg       # SysConfig 源配置
   ├─ PID/
   │  ├─ pid.c
   │  └─ pid.h
   ├─ app/
   │  ├─ k230_uart.c               # K230 串口收包与解析
   │  ├─ lcd_status.c              # LCD 状态界面
   │  ├─ track_control.c           # 追踪控制与 PID 外环
   │  └─ uart_display.c            # UART0 调试显示
   ├─ common/
   └─ modules/
      ├─ zdt_stepper.c             # CAN 步进驱动
      ├─ st7789.c                  # LCD 驱动，含 DMA 发屏
      ├─ encoder.c
      ├─ icm20948.c
      ├─ uart0_dma.c
      ├─ motor.c
      └─ key.c
```

## 串口协议

`K230` 发给下位机的数据包格式：

```text
@valid,cx,cy#
```

示例：

```text
@1,203,117#
@0,000,000#
```

字段说明：
- `valid`：`0/1`，是否检测到有效矩形
- `cx`：矩形中心点 `x`
- `cy`：矩形中心点 `y`

下位机会在 [DAP_LINK_TEST/app/k230_uart.c](DAP_LINK_TEST/app/k230_uart.c) 中将其解析为目标数据，并按当前画面中心计算：

```text
err_x = cx - 200
err_y = cy - 120
```

当前还带有串口失联保护：
- 超过 `150 ms` 没收到新包，自动判定 `K230` 离线
- 离线后目标置无效，追踪控制自动停机

## 追踪控制

控制主逻辑位于 [DAP_LINK_TEST/app/track_control.c](DAP_LINK_TEST/app/track_control.c)。

当前策略：
- 上电延时约 `1 s` 后自动进入追踪，不再依赖按键解锁
- 外环使用 `PID`，输出为两轴目标转速
- 两个步进电机内部本身已闭环，下位机主要负责视觉外环
- `X/Y` 轴参数可分开调，便于处理横向和纵向视野不一致的问题

当前已经加入的保护：
- 目标无效时立即停机
- 串口超时失联时立即停机
- 速度变化有限幅，防止单次命令跳变过大

## LCD 显示

LCD 状态页位于 [DAP_LINK_TEST/app/lcd_status.c](DAP_LINK_TEST/app/lcd_status.c)，当前大致布局如下：

- 左上：编码器
- 右上：IMU 姿态角
- 左下：`K230` 状态、`X/Y`
- 右下：两轴步进电机 `RPM`

显示驱动位于 [DAP_LINK_TEST/modules/st7789.c](DAP_LINK_TEST/modules/st7789.c)：
- `SPI1`
- `20 MHz`
- `TX DMA`
- 动态文本区域已做批量刷屏优化

## 关键接口与引脚

以 [DAP_LINK_TEST/cmsis_dsp_empty.syscfg](DAP_LINK_TEST/cmsis_dsp_empty.syscfg) 为准，当前关键接口如下：

| 功能 | 外设 | 引脚 |
| --- | --- | --- |
| 步进 CAN | `MCAN0` | `PA12 TX`, `PA13 RX` |
| K230 串口 | `UART2` | `PA21 TX`, `PA22 RX` |
| 调试串口 | `UART0` | 见 `.syscfg` |
| LCD SPI | `SPI1` | `PB8 MOSI`, `PB9 SCLK` |
| LCD 控制 | GPIO | `PB10 RES`, `PB11 DC`, `PB14 CS`, `PB26 BLK` |
| 用户按键 | GPIO | `PB21` |

说明：
- 两个步进电机接在同一条 `CANH/CANL` 总线上
- 电机地址当前使用 `1` 和 `2`
- 如果后续硬件改线，优先改 `.syscfg`，不要直接改生成文件

## 构建

推荐使用 GCC 预设：

```powershell
cmake --preset gcc-debug
cmake --build --preset gcc-debug
```

如需使用 TI Clang：

```powershell
cmake --preset ticlang-debug
cmake --build --preset ticlang-debug
```

构建输出示例：

```text
build-gcc/DAP_LINK_TEST/dap_link_test.elf
build-gcc/DAP_LINK_TEST/dap_link_test.map
build-gcc/DAP_LINK_TEST/syscfg/
```

## 调试建议

- 如果 `LCD` 有坐标但电机不动，先查：
  - `CANH/CANL`
  - 电机供电
  - 电机地址
  - 杜邦线和端子压接
- 如果摄像头断开后电机仍转，先确认是否烧录了带串口超时保护的新固件
- 如果画面已经稳定识别，但追踪超调：
  - 先调 `track_control.c` 里的 `KP / KD / DEADBAND / MAX_RPM`
  - 上下轴与左右轴建议分开调
- 如果 LCD 刷新发卡，优先检查：
  - 是否使用了当前 `SPI DMA` 版本
  - 是否有过于频繁的动态文字重绘

## 当前工程状态

这份仓库当前更接近“可持续迭代的联调版本”，而不是最终比赛定版。

已经完成：
- `K230` 矩形识别与串口发包
- `MSPM0` 收包、显示、追踪控制
- `CAN` 步进驱动封装
- `PID` 骨架搭建
- `LCD` 状态显示与 DMA 优化

后续仍适合继续扩展：
- 更完整的步进状态反馈
- 更细的外环/内环参数整定
- 编码器电机速度环、位置环复用 `PID`
- 更完善的异常保护与调试信息显示

## 说明

- 构建、引脚、DMA、时钟等硬件配置请以 `.syscfg` 和生成的 `ti_msp_dl_config.*` 为准
- 新增源文件后，记得同步更新 [DAP_LINK_TEST/CMakeLists.txt](DAP_LINK_TEST/CMakeLists.txt)
- 本仓库使用 UTF-8 文本；如果终端编码不对，中文可能显示异常
