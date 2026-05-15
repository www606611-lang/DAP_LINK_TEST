# Agent Handoff - 2026-05-06

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
