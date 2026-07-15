# Firmware Architecture

```text
app/main
  -> control/control_supervisor
  -> diagnostics/reset_diagnostics
  -> bsp/board_button
  -> bsp/board_motor_safe
  -> drivers/mcu/encoder_input
  -> drivers/device/at8236 -> drivers/mcu/motor_pwm
  -> drivers/device/st7789

future control loops
  -> speed loop
  -> position loop -> speed loop
  -> yaw loop -> speed loop
  -> line loop -> speed loop
  -> motor device driver -> MCU PWM driver -> SysConfig/platform
```

## Ownership

- `bsp`: board wiring, physical channel names, safe pin states.
- `control`: reusable PID and mutually exclusive motion modes.
- `diagnostics`: reset and first-fault evidence without initiating a second
  software reset.
- `drivers/device`: external devices such as ST7789 and the current AT8236
  dual-channel command layer. ICM20948 and line sensors remain future work.
- `drivers/mcu`: MCU-facing encoder GPIO/interrupt capture and shared TIMG6/
  TIMG7 motor PWM output. I2C, UART, and CAN remain future work.
- `app`: scheduling, display, commands, and mode transitions.

## Integration gates

1. Safe base: PB21/LCD/reset reporting, all motor pins high impedance.
2. Encoder shadow mode: count and speed measurement with motor output disabled.
3. Single-channel open loop A: motor A/E0 is validated as the left wheel with
   press-to-run, second-press stop, PWM ramp, supervised lease, and automatic
   high-impedance stop.
4. Single-channel open loop B: motor B/E1 is confirmed as the stable right
   wheel without reset. Board-level motor polarity is corrected and a logical
   positive command now produces positive E1 feedback.
5. Dual-channel open loop: arm both channels under one lease, apply one shared
   ramp time base, and compare E0/E1 signs, speeds, counts, invalid transitions,
   and reset behavior.
6. Speed loop: migrate the tuned loop through the new motor/encoder APIs.
7. Position loop: cascade position output into the verified speed loop.
8. Yaw loop: IMU validation, then yaw output into the speed loop.
9. Line tracking: line sensor validation, then steering correction into the
   speed loop.
10. Mode arbitration: only one outer loop may own speed targets at a time.

The old tuned PID values are reference calibration data. They are promoted only
after the corresponding new driver path produces matching units, signs, sample
periods, and saturation behavior.
