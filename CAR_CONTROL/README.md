# CAR_CONTROL

Formal MSPM0G3507 car-control firmware, rebuilt from the clean PB21/LCD bring-up
baseline. The older `DAP_LINK_TEST` and `DIANSAI_MOTOR_DRIVER_BOARD` projects are
reference sources only; they are not copied wholesale into this target.

## Current safety baseline

- PB21, SW2/PB4, and SW1/PB5 are active-low with a 20 ms software
  debounce. PB4/PB5 use the board's 200 kOhm pull-ups and 100 nF capacitors
  plus MCU internal pull-ups.
- ST7789 SPI display support is enabled.
- Motor control nets are confirmed as:
  - channel A: `PA29 -> AIN1`, `PA30 -> AIN2`
  - channel B: `PA23 -> BIN1`, `PA24 -> BIN2`
- TIMG6 provides 20 kHz PWM for motor A, and TIMG7 provides 20 kHz PWM for
  motor B. Both channels are enabled only during a supervised motion mode.
- All four motor pins return to high impedance at startup, when any of the
  three buttons is pressed during motion, after the speed-command lease
  expires, at the test endpoint, and on every emergency stop.
- Motor A/E0 is the validated left wheel and motor B/E1 is the validated right
  wheel. For ground testing, PB21 commands `+4000` encoder counts, SW2/PB4
  commands `-2000`, and SW1/PB5 commands `+2000`. The commands use the 50 Hz
  position outer loop and verified 100 Hz speed inner loop. Pressing any key
  during motion stops immediately; commands are never stacked while moving.
  Remote `pos run` and Stress 24 retain the separately tunable 1060-count
  default.
- Motor B drive polarity is inverted at the board configuration layer so a
  positive command produces forward motion and positive E1 feedback, matching
  motor A/E0. This sign was confirmed in the powered right-wheel retest.
- PB0/PB1 and PB2/PB3 run as raw encoder channels 0 and 1 in shadow mode.
  Motor A is paired with encoder channel 0, and motor B is paired with encoder
  channel 1. The board configuration normalizes both encoder channels so
  vehicle-forward rotation produces positive counts. A five-revolution hand
  calibration established 1060 quadrature counts per wheel revolution.
- Reset cause, control mode, block reason, and motor-safe state are visible on
  the LCD and as debugger globals.
- UART3 provides detached speed-loop tuning at 9600 baud: PA26 is MCU TX and
  PA25 is MCU RX. Applying parameters does not start either motor.

Direct mode requests remain blocked. Verified outer controllers enter
position, yaw, or line-tracking mode only through the supervised speed-loop
owner API, so unimplemented modules cannot arm the motors by changing a mode.

## Reusable wheel-drive API

`bsp/board_wheel_drive.h` is the single product-facing motor-output API:

```c
BoardWheelDrive_Init();
BoardWheelDrive_SetCommands(left_permille, right_permille);
BoardWheelDrive_SetZero();
```

Commands range from `-1000` to `1000`; positive always means vehicle-forward
for both wheels. The BSP owns motor A/B mapping and the right-motor polarity
correction. Both arguments are validated before output, and every rejected or
failed command zeros both wheels. The caller must still pass non-OK results to
`ControlSupervisor_EmergencyStop` to restore hardware high impedance.

Application and control modules must not call `AT8236_MotorSetCommand` or
`MotorPwm_SetDuty` directly. Both open-loop diagnostics and the speed controller
use this wheel-drive API.

## Reusable speed-control API

`control/wheel_speed_control.h` owns the two 100 Hz PI loops. Encoder speed is
updated every 10 ms through a four-sample moving average:

```c
WheelSpeedControl_Start(now_ms);
WheelSpeedControl_StartForMode(CAR_CONTROL_MODE_YAW, now_ms);
WheelSpeedControl_SetTunings(&tunings);
WheelSpeedControl_SetTargets(left_pps, right_pps, now_ms);
WheelSpeedControl_Task(now_ms);
WheelSpeedControl_Stop(reason);
WheelSpeedControl_GetSnapshot(&snapshot);
```

Targets are encoder pulses per second and are limited to `-6000..6000`. A
successful update submits both PWM commands together. Exactly one of `SPEED`,
`POSITION`, `YAW`, or `LINE_TRACKING` owns the inner loop. The owner must submit
fresh targets at least every 100 ms; the inner loop refreshes a separate 200 ms
hardware supervisor lease only while it remains healthy. Invalid targets,
stale outer-loop commands, encoder access failure, output failure, or ownership
loss return the board to high impedance.

## Reusable position-control API

`control/wheel_position_control.h` owns the dual-wheel position outer loop:

```c
WheelPositionControl_SetConfig(&config);
WheelPositionControl_StartRelative(left_delta, right_delta, timeout_ms, now_ms);
WheelPositionControl_StartAbsolute(left_target, right_target, timeout_ms, now_ms);
WheelPositionControl_Task(now_ms);
WheelPositionControl_Stop(reason);
WheelPositionControl_GetSnapshot(&snapshot);
```

Position uses raw quadrature counts and converts left/right errors to speed
targets. The validated symmetric configuration is `Kp=3.0`, maximum speed
`2000 pps`, tolerance `24 counts`, settle speed `120 pps`, and settle time
`200 ms`. Equal relative wheel moves also use straight-line cross-coupling:
`sync Kp=2.0 pps/count` with a `400 pps` correction limit. The correction uses
the left-minus-right progress since the current move started, applies equal and
opposite speed changes, and disengages near the endpoint. Unequal wheel targets
keep independent control for future turning motions. A completed move and every
stop or fault path return both motor channels to high impedance. Position
targets never bypass the speed inner loop.
Position ownership uses a `4000 pps/s` speed-target slew. If a wheel remains
below `40 pps` for 300 ms while still outside tolerance, the controller issues
a bounded recovery request, never exceeding the configured maximum speed.

The position bring-up application also provides
`PositionBringupTest_RequestMove(delta_counts)` for one-shot physical-button
moves without changing the remote single-run or stress-profile target.

## Bluetooth speed tuner

`tools/speed_tuner/Launch-SpeedTuner.cmd` opens the Windows GUI. Connect a
3.3 V compatible Bluetooth UART module as follows:

```text
PA26 / UART3 TX -> Bluetooth RX
PA25 / UART3 RX <- Bluetooth TX
GND             -- common GND
```

The tool reads and atomically applies symmetric Kp/Ki/Kd, target speed, and
output limit values held in RAM. It can start or stop the supervised five
second speed test and displays both measured speeds, outputs, invalid encoder
transitions, result code, and final high-impedance state. A reset restores the
compiled defaults. The TCP bridge also accepts `pos get`, `pos set`, `pos run`,
`pos run stress`, `pos stop`, and `pos stat`. The GUI has separate speed and
position tabs with position configuration including straight-line sync gain and
limit, live progress, recovery totals, and a 24-segment stress button. During a
position run, VOFA+ receives the
six-channel `position` group and the tool records `latest_position_wave.json`
plus `latest_position_telemetry.csv` under `tools/speed_tuner/runtime`.

## Build and flash

From the repository root:

```powershell
cmake --preset gcc-debug
cmake --build build-gcc --target car_control -j
& "$env:USERPROFILE\SEGGER\JLink\JLink.exe" -NoGui 1 `
  -CommandFile CAR_CONTROL\tools\flash_gcc.jlink
```

In VS Code, run `Terminal -> Run Task -> Build + Flash (J-Link)`.

## Debug globals

```text
g_car_pb21_pressed
g_car_pb21_press_count
g_car_pb4_pressed
g_car_pb4_press_count
g_car_pb5_pressed
g_car_pb5_press_count
g_car_last_button_move_counts
g_car_reset_cause
g_car_control_mode
g_car_control_block_reason
g_car_motor_high_impedance
g_car_encoder_shadow_active
g_car_encoder_0_count
g_car_encoder_0_speed_pps
g_car_encoder_0_edges
g_car_encoder_0_invalid
g_car_encoder_1_count
g_car_encoder_1_speed_pps
g_car_encoder_1_edges
g_car_encoder_1_invalid
g_car_speed_test_state
g_car_speed_test_run_count
g_car_speed_left_target_pps
g_car_speed_right_target_pps
g_car_speed_left_error_pps
g_car_speed_right_error_pps
g_car_speed_left_output_permille
g_car_speed_right_output_permille
g_car_speed_update_count
g_car_speed_last_result
g_car_position_test_state
g_car_position_test_run_count
g_car_position_left_target_count
g_car_position_right_target_count
g_car_position_left_error_count
g_car_position_right_error_count
g_car_position_left_speed_target_pps
g_car_position_right_speed_target_pps
g_car_position_sync_error_count
g_car_position_sync_correction_pps
g_car_position_sync_active
g_car_position_update_count
g_car_position_last_result
g_car_position_settled
g_car_encoder_count_difference
g_car_encoder_speed_difference_pps
```

See `HARDWARE_MAP.md` before enabling any additional peripheral and
`ARCHITECTURE.md` for the staged integration order. Bench observations and the
remaining manual checks are recorded in `BRINGUP_LOG.md`.
