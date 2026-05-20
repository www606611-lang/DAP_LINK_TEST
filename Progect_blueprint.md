# Project Blueprint

## Git State

Current branch:

```text
checkpoint/position-cascade-20260519
```

Current `HEAD` is also labeled:

```text
位置环_OK
```

This means the project is currently continuing from the `位置环_OK`
checkpoint, with local uncommitted changes on top.

Recent Git graph:

```text
* 741dbdc  HEAD -> checkpoint/position-cascade-20260519, 位置环_OK
* d8cb773  Checkpoint cascaded position tuning setup
* da6b070  tag: line-tracking-before-vofa-debug-2026-05-17, main
* cbc3acc  origin/main  feat: add yaw angle control api
* 0bde7ce  tag: snapshot-Speed_loop_position_loop_over_2026_05_15
* 5c2e5bf  Speed_loop_position_loop_over_2026_05_15
* 9658bdc  stable encoder imu working state
...
* c63e81f  tag: IMU_OK
* b2f034c  tag: UART_OK
* 105166d  tag: OELD_OK
* 666b4df  tag: v0.1.0
```

Current uncommitted changes:

```text
M  DAP_LINK_TEST/cmsis_dsp_empty.c
M  DAP_LINK_TEST/PID/yaw_angle_control.c
M  DAP_LINK_TEST/PID/line_tracking_control.c/.h
M  DAP_LINK_TEST/PID/encoder_speed_control.h
M  DAP_LINK_TEST/app/pid_console.c
M  DAP_LINK_TEST/app/pid_tuning_store.c
M  DAP_LINK_TEST/CMakeLists.txt
?? DAP_LINK_TEST/app/app_diagnostics.c/.h
?? DAP_LINK_TEST/app/line_tracking_app.c/.h
```

## Project Structure

```text
mspm0_Project
├─ AGENT.md                         Project notes and handoff context
├─ AGENT_HANDOFF_2026-05-06.md       Historical handoff record
├─ CMakeLists.txt                    Top-level CMake entry
├─ build-gcc/                        GCC build directory
├─ build-ticlang/                    TIClang build directory
└─ DAP_LINK_TEST/
   ├─ cmsis_dsp_empty.c              Current main.c / app loop
   ├─ cmsis_dsp_empty.syscfg         SysConfig peripheral setup
   ├─ CMakeLists.txt                 Firmware source list
   │
   ├─ common/
   │  ├─ timer.c/.h                  Millisecond time base
   │  └─ delay.c/.h                  Blocking delay helpers
   │
   ├─ modules/                       Low-level hardware drivers
   │  ├─ motor.c/.h                  Left/right motor PWM, direction, right-wheel mapping
   │  ├─ encoder.c/.h                Encoder count and speed
   │  ├─ icm20948.c/.h               IMU angle estimation
   │  ├─ key.c/.h                    Button input
   │  ├─ line_sensor_i2c.c/.h        8-channel line sensor
   │  ├─ uart0_dma.c/.h              UART0 DMA
   │  ├─ st7789.c/.h                 LCD driver
   │  └─ zdt_stepper.c/.h            Stepper motor driver
   │
   ├─ PID/                           Control loops
   │  ├─ pid.c/.h                    Generic PID controller
   │  ├─ encoder_motor_pid.c/.h      Motor PID core
   │  ├─ encoder_speed_control.c/.h  Speed loop
   │  ├─ encoder_position_control.c/.h Position loop
   │  ├─ yaw_angle_control.c/.h      Yaw / angle loop
   │  └─ line_tracking_control.c/.h  Line tracking loop
   │
   └─ app/                           Application-level wrappers
      ├─ app_diagnostics.c/.h        Reset cause and fault-stage diagnostics
      ├─ pid_console.c/.h            Bluetooth/UART PID command parser
      ├─ pid_tuning_store.c/.h       Flash save/load for PID parameters
      ├─ bluetooth_uart.c/.h         Bluetooth UART
      ├─ uart_display.c/.h           UART0 display and command entry
      ├─ lcd_status.c/.h             LCD status screen
      ├─ line_tracking_app.c/.h      Line tracking app wrapper
      ├─ k230_uart.c/.h              K230 communication
      └─ track_control.c/.h          Vision/track control related code
```

## Current Main Loop

The current firmware entry is `DAP_LINK_TEST/cmsis_dsp_empty.c`.

Initialization flow:

```text
main / cmsis_dsp_empty.c
├─ SYSCFG_DL_init
├─ timer_common_init
├─ ICM20948_TaskInit
├─ Motor_Init
├─ Encoder_Init
├─ Key_Init
├─ AppDiagnostics_ReportResetCause
├─ EncoderSpeedControl_Init
├─ LineTrackingControl_Init
├─ YawAngleControl_Init
├─ EncoderPositionControl_Init
├─ EncoderPositionControl_SyncSpeedFromCurrent
├─ lcd_status_screen_init
├─ PidTuningStore_LoadApply
├─ YawAngleControl_Stop
├─ uart_display_init
└─ bluetooth_uart_init
```

Runtime loop:

```text
while (1)
├─ Encoder_Task
├─ ICM20948_Task
├─ Key_Task
├─ uart_display_task
├─ bluetooth_uart_task
├─ YawAngleControl_Task
└─ lcd_status_screen_task
```

Current control status:

```text
Speed loop:      initialized and used by yaw loop
Position loop:   initialized, available through API / PID console
Line tracking:   initialized only, task is not running
Yaw loop:        active task in main loop, ready for Bluetooth tuning
Bluetooth PID:   still enabled
LCD status:      still enabled
```

## Yaw Tuning Commands

Useful Bluetooth/UART commands:

```text
pid yaw zero
pid yaw hold
pid yaw target 90
pid yaw add 10
pid yaw stop
pid yaw kp 18
pid yaw ki 0
pid yaw kd 0
pid yaw out 900
pid yaw ilim 60
pid yaw db 1
pid yaw minturn 200
pid yaw maxturn 900
pid yaw mindrive 145 165 380 300 400
pid save
pid show yaw
```

## Notes

- The right-wheel motor mapping is in `DAP_LINK_TEST/modules/motor.c`.
- Do not change the right-wheel mapping unless explicitly needed.
- Line tracking has been tuned and is currently not running from `main`.
- Yaw loop is now the main loop being tuned.
- Build command used for verification:

```text
cmake --build build-gcc --target dap_link_test
```

