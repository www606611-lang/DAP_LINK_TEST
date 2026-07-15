# CAR_CONTROL

Formal MSPM0G3507 car-control firmware, rebuilt from the clean PB21/LCD bring-up
baseline. The older `DAP_LINK_TEST` and `DIANSAI_MOTOR_DRIVER_BOARD` projects are
reference sources only; they are not copied wholesale into this target.

## Current safety baseline

- PB21 is active-low with a 20 ms software debounce.
- ST7789 SPI display support is enabled.
- Motor control nets are confirmed as:
  - channel A: `PA29 -> AIN1`, `PA30 -> AIN2`
  - channel B: `PA23 -> BIN1`, `PA24 -> BIN2`
- TIMG6 provides 20 kHz PWM for motor A, and TIMG7 provides 20 kHz PWM for
  motor B. Both channels are enabled only during the supervised dual-motor
  test.
- All four motor pins return to high impedance at startup, on a second PB21
  press, after the supervised test lease expires, and on every emergency stop.
- Motor A/E0 is the validated left wheel and motor B/E1 is the validated right
  wheel. Pressing PB21 once starts both motors under one supervisor lease.
  Both motors use the same 500 ms ramp and time base so they start together,
  target 70%, stop automatically at 3 seconds, and stop immediately if PB21 is
  pressed again.
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

The supervisor already defines future modes for open-loop, speed, position,
yaw, and line tracking, but every non-idle request is blocked until its hardware
bring-up milestone is explicitly enabled.

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
`MotorPwm_SetDuty` directly. The open-loop bring-up test now uses the same
wheel-drive API that the future speed loop will use.

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
g_car_motor_test_state
g_car_motor_test_command
g_car_motor_test_command_a
g_car_motor_test_command_b
g_car_motor_test_run_count
g_car_motor_test_channel
g_car_encoder_count_difference
g_car_encoder_speed_difference_pps
```

See `HARDWARE_MAP.md` before enabling any additional peripheral and
`ARCHITECTURE.md` for the staged integration order. Bench observations and the
remaining manual checks are recorded in `BRINGUP_LOG.md`.
