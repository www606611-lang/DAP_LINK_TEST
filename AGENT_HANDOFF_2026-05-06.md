# Agent Handoff - 2026-05-17

## 2026-05-17 最新交接：NMI / CPU LOCKUP 追踪

### 当前结论
- 当前最新复位日志不是普通的软复位，而是 `SYSRST_CPU_LOCKUP_VIOLATION` 相关的异常链。
- 之前抓到的现场里，`PC/LR` 已经落到 `memchr` / `_svfprintf_r`，所以更像是字符串格式化 / 显示输出链触发了异常，而不是速度环本体。
- 为了进一步缩小范围，已给 `fault_capture` 增加 NMI 源号记录，下一版启动会打印 `N:<source>`。
- 新收到的启动串口是 `RST:BOR_SUPPLY_FAILURE`，这说明芯片自己检测到了电源掉压/欠压，优先级已经高于软件异常。
- 当前 `Motor_Init` 和速度环初始化路径本身不会主动给电机大功率，若上电就触发 BOR，更像是供电瞬态、接触不良、驱动板拉电流过大，或者 MCU/电机共电源下电压跌落。

### 已完成改动
- `DAP_LINK_TEST/modules/fault_capture.c/.h`
  - 保存 HardFault / NMI 现场。
  - 新增 `nmi_iidx`，用于区分 `BORLVL`、`WWDT0`、`LFCLK_FAIL`、`FLASH_DED`、`SRAM_DED` 等来源。
- `DAP_LINK_TEST/cmsis_dsp_empty.c`
  - 启动时继续打印 `RST:` 和故障信息。
  - 若为 NMI，会额外打印 `N:<source>`。

### 下一步
1. 重新烧录最新版本。
2. 观察第一条 `RST:` 串口，重点看 `N:`。
3. 现在要优先查供电链，不要先调 PID。
4. 重点验证：单独给 MCU 供电是否还会复位、接上电机电源是否立刻掉压、启动瞬间 3.3V 是否塌陷。
5. 如果后续还看到 `N:`，再按对应 NMI 源继续缩小范围。

## 2026-05-17 最新交接：八路灰度循迹与左右轮平衡

### 当前已完成

- 已在当前项目中接入八路灰度循迹模块。
- 八路灰度模块使用 `I2C1`，当前引脚为 `PA16/PA17`。
- 灰度底层读取已经调通，相关文件：
  - `DAP_LINK_TEST/modules/line_sensor_i2c.c`
  - `DAP_LINK_TEST/modules/line_sensor_i2c.h`
- 注意：灰度底层 I2C 读写逻辑已经可用，后续不要随意改 `line_sensor_i2c.c/.h`。

- 已新增循迹控制模块：
  - `DAP_LINK_TEST/PID/line_tracking_control.c`
  - `DAP_LINK_TEST/PID/line_tracking_control.h`
- 循迹控制当前作为外层 PID，串速度环使用。
- 速度环和位置环是已调好的基座，后续循迹调参不要改：
  - `encoder_speed_control.c/.h`
  - `encoder_position_control.c/.h`
- 循迹层可以调用速度环 API，例如 `EncoderSpeedControl_SetTargetPps()`，但不要改速度环参数、采样逻辑、PWM 限幅、位置环参数。

- 当前 `main` 文件为：
  - `DAP_LINK_TEST/cmsis_dsp_empty.c`
- 当前主循环包含：
  - `Encoder_Task`
  - `ICM20948_Task`
  - `Key_Task`
  - `LineTrackingControl_Task`
  - `uart_display_task`
  - `lcd_status_screen_task`
- 当前按键 `KEY_ID_B21` 用于切换循迹开关。

- LCD 当前显示八路灰度原始状态、有效数量、循迹误差和开关状态。
- LCD 不要堆太多调试字段，之前串口/LCD 调试过重会造成 LCD 卡顿。

- 已增加轻量 VOFA 文本输出，便于继续观察左右轮平衡。
- 当前 VOFA 输出格式：

```text
d:leftTarget,rightTarget,leftActual,rightActual,lineErr,turnPps
```

- 当前 VOFA 输出位置：
  - `DAP_LINK_TEST/cmsis_dsp_empty.c`
- 当前输出周期：
  - `100 ms`
- 当前实现已经避免 `snprintf`，改为手写轻量整数拼接。
- 注意：之前尝试过 JustFloat 二进制帧，但用户当前 VOFA 使用的是文本 `d:` 格式，因此不要再改成二进制。

- 已经为 VOFA 调试前的稳定状态做过本地 checkpoint：
  - commit: `da6b070 checkpoint: stable line tracking before vofa debug`
  - tag: `line-tracking-before-vofa-debug-2026-05-17`

### 当前循迹参数状态

当前主要调参文件：

- `DAP_LINK_TEST/PID/line_tracking_control.c`

当前循迹层大致思路：

- 灰度误差 `line_error` 经过循迹 PID 输出 `turn_correction_pps`
- 左右轮目标速度由 `base_speed ± turn_correction_pps` 得到
- 右轮由于机械/电机差异和死区更大，在循迹层做右轮单独补偿
- 注意：右轮补偿只能放在循迹层，不要为了循迹去改速度环/位置环基座

最近一次根据 VOFA 日志做出的判断：

- 之前右轮补偿过猛，出现了“右轮不跟目标、实际速度卡在约 800pps”的现象。
- 典型数据：
  - `rightTarget=2015` 时 `rightActual≈800`
  - `rightTarget=325` 时 `rightActual≈800`
  - `rightTarget=1170` 时 `rightActual≈820`
- 这说明当时右轮不是目标没给够，而是最小驱动地板太硬，导致右轮低速不可控。

因此最近一版已把循迹层调成更可控的版本：

- 基础速度从 `900` 降到 `800`
- 最大转向从 `650` 降到 `420`
- 循迹 `Kp` 从 `35` 降到 `18`
- 循迹 `Kd` 从 `3.0` 降到 `1.5`
- 右轮最小驱动从 `380` 降到 `320`
- 最小驱动参考从 `0` 改回 `800`
- 右轮目标增益从 `1.30` 降到 `1.15`

### 接下来需要继续完善

1. 继续烧录当前版本，看 VOFA 波形。
2. 重点观察：
   - `leftTarget` 和 `rightTarget`
   - `leftActual` 和 `rightActual`
   - `lineErr`
   - `turnPps`
3. 先确认右轮是否重新跟随目标：
   - `rightTarget` 较低时，`rightActual` 应该能降下来
   - 不能再出现 `rightTarget=300~500` 但 `rightActual≈800` 的硬地板现象
4. 直行时观察：
   - `lineErr≈0`
   - 左右 actual 是否接近
   - 如果右轮仍弱，可以小步增加 `LINE_TRACKING_RIGHT_SPEED_GAIN`
   - 如果右轮低速仍起不来，可以小步增加右轮 `MIN_PWM`
5. 转弯时观察：
   - `turnPps` 是否频繁打满
   - 如果经常打满，优先继续降低 `Kp` 或 `MAX_TURN_PPS`
   - 如果跟线慢，再逐步增加 `Kp`
6. 不要一次同时改很多参数。
7. 不要再改速度环和位置环。
8. 不要再改灰度 I2C 底层。
9. 如果 VOFA 输出导致 LCD 卡顿：
   - 先增加输出周期，例如 `100ms -> 200ms`
   - 或减少字段
   - 不要恢复 `snprintf`
   - 不要改成二进制 JustFloat，除非用户明确切换 VOFA 协议

### 当前重要边界

- 速度环、位置环已经是项目基座，默认不要改。
- 循迹 PID 可以串速度环，但只能调用速度环 API。
- 右轮死区大、左右电机不对称，这是已确认的机械/电机特性。
- 补偿应该在外层控制中做，尤其当前循迹调参只改 `line_tracking_control.c`。
- 每次调参后必须跑：

```powershell
cmake --build build-gcc --target dap_link_test
```

- 默认只跑 `build-gcc`，不要默认跑 `build-ticlang`。

> 这个文件是给后续 Codex / Agent 接手项目时先读的中文交接文档。当前项目还在搭基础能力阶段，后面遇到真实赛题/真实任务时，应当在这些基础模块上继续发挥，而不是重新从零乱改。

## 先读这个：用户协作习惯

- 用户希望我先建立全局认识，再判断问题，不要“盲目改参数、改一版试一版”。
- 用户很重视真实硬件反馈：串口日志、VOFA 波形、LCD 表现、上电/复位/电机实际动作都要一起看。
- 用户会直接指出现象，比如“右轮不动”“IMU 消失”“MSPM0 不断复位”“波形黄色线抖”，需要我先分析可能原因，再决定是否改代码。
- 用户不喜欢把临时测试逻辑堆进 `main` / `cmsis_dsp_empty.c`。主循环要干净，只保留模块初始化和周期任务调用。
- 临时测试可以做，但测完要收束成正式 API，删除 debug/test app，避免后续项目越来越乱。
- 调 PID 时要尊重波形细节：先确认方向、采样周期、实际量纲，再谈 KP/KI/KD。
- 串口输出要为当前调试目的服务，不需要的通道及时删掉，避免 VOFA 窗口压力和误判。
- LCD 显示要简洁，不要塞太多调试文本；速度环参数、临时测试参数测完就清掉。
- 硬件异常不能只怪代码：IMU 读不出、MCU 复位、电机一动 LCD 卡顿时，要同时考虑供电、电机纹波、接线、插拔状态和外设总线。
- 代码修改后尽量同时跑 `build-ticlang` 和 `build-gcc`，确认两套构建都过。

## 当前主线状态

当前重点已经从速度环测试切到位置环基础能力。速度环已经基本调通，并被封装成 API；位置环也已经完成第一版串级 PID，并通过按键 + VOFA 做过阶跃测试，波形效果可用。

当前 `main` 文件：

- `DAP_LINK_TEST/cmsis_dsp_empty.c`

当前主循环保持简洁：

- 初始化：`timer_common_init`、`ICM20948_TaskInit`、`Motor_Init`、`Encoder_Init`、`EncoderPositionControl_Init`、`lcd_status_screen_init`、`uart_display_init`
- 周期任务：`Encoder_Task`、`ICM20948_Task`、`uart_display_task`、`lcd_status_screen_task`、`EncoderPositionControl_Task`

注意：`main` 当前已经不再调用速度环测试，也不再调用位置环 debug app。

## 本轮完成的核心工作

### 1. 速度环从测试代码整理为正式模块

原来的：

- `DAP_LINK_TEST/app/encoder_speed_test.c`
- `DAP_LINK_TEST/app/encoder_speed_test.h`

已经删除。

现在保留正式速度环模块：

- `DAP_LINK_TEST/PID/encoder_speed_control.c`
- `DAP_LINK_TEST/PID/encoder_speed_control.h`

主要 API：

```c
EncoderSpeedControl_Init(now_ms);
EncoderSpeedControl_Task(now_ms);
EncoderSpeedControl_SetTargetPps(left_pps, right_pps);
EncoderSpeedControl_GetTargetPps(&left_pps, &right_pps);
EncoderSpeedControl_GetSpeedTunings(&left_pid, &right_pid);
EncoderSpeedControl_Stop();
```

当前速度环参数：

- 左轮：`KP=0.08, KI=0.06, KD=0.0`
- 右轮：`KP=0.06, KI=0.11, KD=0.0`
- PWM 限幅：`330`
- 速度死区：`12 pps`
- 左轮积分限幅：`3200`
- 右轮积分限幅：`4000`
- 前馈当前为 `0`

重要经验：

- 右轮机械/电机/编码器特性和左轮明显不一样，右轮需要更大的积分补偿。
- 速度采样和 PID 更新必须对齐到 `ENCODER_SAMPLE_INTERVAL_MS`，避免 PID 在旧速度数据上重复更新。
- 之前出现过右轮不动、左右同 PWM 速度差很多、速度环一开 MCU/LCD 掉电等现象，后续再遇到类似问题要先分层验证：开环 PWM -> 编码器方向 -> 单轮闭环 -> 双轮闭环。

### 2. 新增通用编码器电机 PID 封装

文件：

- `DAP_LINK_TEST/PID/encoder_motor_pid.c`
- `DAP_LINK_TEST/PID/encoder_motor_pid.h`

作用：

- 封装单个“编码器 + 电机”的 PID 控制。
- 支持速度模式和位置模式。
- 位置模式是串级结构：位置外环输出速度目标，速度内环输出 PWM。

当前位置环结构：

```text
目标位置 -> 位置 PID -> 目标速度 -> 速度 PID -> PWM -> 电机
```

重要实现点：

- `EncoderMotorPID_Update()` 只有在 `elapsed_ms >= ENCODER_SAMPLE_INTERVAL_MS` 时才更新。
- 内部保存 `encoder_snapshot_t`，便于上层读取 count / speed / pwm / cascade speed。
- `encoder_motor_pid_limit_to_target_direction()` 会限制 PWM 方向与速度目标一致；如果后续做更强的位置保持/反向制动，可能需要重新审视这里。

### 3. 新增位置环正式模块

文件：

- `DAP_LINK_TEST/PID/encoder_position_control.c`
- `DAP_LINK_TEST/PID/encoder_position_control.h`

主要 API：

```c
EncoderPositionControl_Init(now_ms);
EncoderPositionControl_Task(now_ms);
EncoderPositionControl_SetTargetCount(left_count, right_count);
EncoderPositionControl_AddTargetCount(left_delta_count, right_delta_count);
EncoderPositionControl_GetTargetCount(&left_target, &right_target);
EncoderPositionControl_GetCurrentCount(&left_count, &right_count);
EncoderPositionControl_ZeroPosition(now_ms);
EncoderPositionControl_GetState(&left_state, &right_state);
EncoderPositionControl_GetPositionTunings(&left_pid, &right_pid);
EncoderPositionControl_Stop();
```

当前参数：

- 位置外环左轮：`KP=4.0, KI=0.0, KD=0.0`
- 位置外环右轮：`KP=4.0, KI=0.0, KD=0.0`
- 速度内环沿用速度环参数
- 位置外环输出最大速度：`1600 pps`
- PWM 限幅：`330`
- 位置死区：`4 count`
- 速度死区：`12 pps`

测试结果：

- 通过按键每次增加 `300 count` 做过测试。
- VOFA 波形显示目标位置阶跃后，实际位置能较平滑贴近目标，过冲不明显，作为基础版本可以接受。
- 测试用的 `encoder_position_debug.c/.h` 已经删除，不再保留在 app 层。

### 4. PID 基础库增强

文件：

- `DAP_LINK_TEST/PID/pid.c`
- `DAP_LINK_TEST/PID/pid.h`

当前能力：

- 支持 KP/KI/KD。
- 支持输出限幅。
- 支持积分限幅。
- 支持 deadband。
- 使用标准 anti-windup：输出饱和且误差还在推向饱和方向时，不继续累积积分。
- D 项基于 measurement 变化，降低目标阶跃时的 derivative kick。

经验：

- 不要一开始就上 KD。当前电机编码器速度量化明显，D 项很容易放大采样噪声。
- 先确认方向和采样，再调 KP；有稳态误差再加 KI；只有明确需要抑制过冲且速度测量足够干净时再考虑 KD。

### 5. 编码器采样调整

文件：

- `DAP_LINK_TEST/modules/encoder.h`
- `DAP_LINK_TEST/modules/encoder.c`

当前编码器速度采样周期：

```c
#define ENCODER_SAMPLE_INTERVAL_MS 50U
```

速度单位：

- `pps`，即 pulses per second。
- 不是 RPM。

重要经验：

- 采样周期越短响应越快，但速度量化越明显。
- 当前速度曲线最低变化粒度仍与编码器脉冲和采样周期相关，这是硬件/采样本身决定的，不是 VOFA 显示问题。

### 6. ST7789 / 串口显示清理

文件：

- `DAP_LINK_TEST/app/lcd_status.c`
- `DAP_LINK_TEST/app/uart_display.c`

当前 LCD：

- 顶部保留 UART RX 内容和计时。
- 中部显示 IMU 姿态角 R/P/Y。
- 底部显示编码器速度 L/R。
- 已清理速度环 KP/KI/KD 参数显示。

当前串口：

- 已删除 `UART0 DMA OK` 启动提示。
- 位置环 debug 串口输出已经随 debug app 删除。
- 后续如需 VOFA 输出，要针对当前调试目标临时加，测完再删或封装到明确 debug 模块。

## 硬件与调试经验

### IMU / ICM20948

之前出现过：

- `E:001`
- `69:0/3 FF`
- `68:1/255 FF/60`
- IMU 姿态角不显示

最后重新插拔后恢复，说明当时很可能有硬件/接触/上电状态问题。后续不要一看到 IMU 失败就马上大改驱动。排查顺序：

1. 确认供电和插拔状态。
2. 看 I2C 地址 0x68 / 0x69 是否能读到正确 whoami。
3. 再检查最近代码是否改过 I2C/ICM 驱动。
4. 最后才考虑驱动回退。

### MCU 复位 / LCD 灭

出现过一上电电机动、MCU 和 LCD 跟着灭、再也醒不来的现象。高概率与电机供电冲击、电压纹波、地线、电源容量有关，也可能与速度环输出过猛有关。

排查顺序：

1. 不接速度环，只开环 PWM 小值测试。
2. 单轮测试。
3. 双轮开环测试。
4. 单轮闭环测试。
5. 双轮闭环测试。
6. 再看是否和 IMU/LCD 同时工作相关。

### 电机/编码器方向

当前项目经验：

- 左编码器需要反向：`Encoder_SetInverted(ENCODER_LEFT, true)`
- 右电机输出需要反向：`Motor_SetRightInverted(true)`

这套方向目前在速度环和位置环中可用。后续如果换接线/换电机/换驱动板，要先重新验证方向，不要直接调 PID。

## 推荐后续路线

### 近期

1. 先提交当前基础版本，避免后续真实题目开发时丢失稳定状态。
2. 给位置环补一个简洁的应用层调用示例，但不要常驻在 main。
3. 如果要继续调位置环：
   - 小步长：`300 count`
   - 中步长：`1000 count`
   - 大步长：`3000 count`
   - 分别观察过冲、到达时间、稳态误差。
4. 如果位置保持不够强，再考虑：
   - 位置 KP 微调
   - 位置死区调整
   - 是否允许接近目标时反向制动

### 后续真实题目

后续真实任务可以基于现有能力继续扩展：

- 直线距离控制：直接调用位置环 API。
- 转向角/车体角度控制：用 IMU yaw 或左右轮差速，外层角度环输出左右位置/速度差。
- 视觉追踪：K230 输出误差，外层视觉 PID 输出速度/角度命令，再交给速度环或位置环。
- 组合动作：把位置环 API 封装成“前进 N count / 后退 N count / 原地转 N count / 停止 / 归零”。

建议控制层级：

```text
任务层/状态机
  -> 位置/角度/视觉外环
  -> EncoderPositionControl 或 EncoderSpeedControl
  -> EncoderMotorPID
  -> Motor PWM
```

不要让任务层直接 `Motor_SetLeft/Right()` 长期控制电机，除非是在开环诊断。

## 2026-05-15 追加：Yaw 角度环测试

新增正式 yaw 角度环模块：

- `DAP_LINK_TEST/PID/yaw_angle_control.c`
- `DAP_LINK_TEST/PID/yaw_angle_control.h`

当前 yaw 控制采用：

```text
目标 yaw -> yaw 角度 PID -> 左右轮相反速度目标 -> EncoderSpeedControl -> 电机 PWM
```

主要 API：

```c
YawAngleControl_Init(now_ms);
YawAngleControl_Task(now_ms);
YawAngleControl_SetTargetDeg(yaw_deg);
YawAngleControl_AddTargetDeg(delta_yaw_deg);
YawAngleControl_HoldCurrentYaw();
YawAngleControl_GetTargetDeg(&yaw_deg);
YawAngleControl_ZeroYaw(now_ms);
YawAngleControl_GetState(&state);
YawAngleControl_GetTunings(&pid);
YawAngleControl_Stop();
```

Yaw 调试模块已经清理：

- `DAP_LINK_TEST/app/yaw_angle_debug.c`
- `DAP_LINK_TEST/app/yaw_angle_debug.h`

都已删除，不再保留按键和串口输出测试代码。

注意：

- yaw 原地转向需要左右轮速度一正一负，所以 `EncoderSpeedControl_SetTargetPps()` 已允许负速度目标。
- 当前 yaw 外环参数：`KP=18, KI=0, KD=0`，最大转向速度 `900 pps`，死区 `1 deg`。
- yaw 角度环内部对左右轮正反方向做了单独最小驱动补偿，用于处理左右电机/正反转不对称。
- 如果后续车转反方向，优先改 `YAW_CONTROL_SIGN`，不要先动 PID。
- 当前 `main` 已清理，不再默认调用速度环、位置环或 yaw 角度环；这些闭环模块保留 API，后续任务层需要时再显式调用。

## 项目内 skill

已新增项目内 skill：

- `skills/mspm0-control-debug/SKILL.md`

用途：

- 后续遇到 MSPM0 电机、编码器、速度环、位置环、IMU、ST7789、VOFA 调试时，先读这个 skill。
- 它总结了本项目的调试顺序和禁忌：先分层验证，再改控制；串口只输出当前目标；测试完要收束成正式 API。

## 构建状态

最近一次代码整合后，两套构建均通过：

```powershell
cmake --build build-ticlang --target dap_link_test
cmake --build build-gcc --target dap_link_test
```

输出分别生成：

- `DAP_LINK_TEST/dap_link_test.out`
- `DAP_LINK_TEST/dap_link_test.elf`

## 当前特别提醒

- 当前工作区有大量本轮修改，建议尽快 git commit。
- 不要恢复旧的 `encoder_speed_test`。
- 不要恢复 `encoder_position_debug`，位置环已正式封装到 `PID/encoder_position_control.*`。
- 后续要看 VOFA 时，新建明确的临时 debug 模块或短期串口输出，测完清理。
- `main` 要继续保持干净。
