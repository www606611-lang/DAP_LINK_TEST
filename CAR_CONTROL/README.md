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
- All four motor pins are configured as GPIO inputs and explicitly forced to
  high impedance at startup.
- No PWM peripheral is present in this build.
- PB21 only updates diagnostics. It cannot arm or start a motor.
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
```

See `HARDWARE_MAP.md` before enabling any additional peripheral and
`ARCHITECTURE.md` for the staged integration order. Bench observations and the
remaining manual checks are recorded in `BRINGUP_LOG.md`.
