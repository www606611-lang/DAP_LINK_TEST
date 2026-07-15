# Firmware Architecture

```text
app/main
  -> control/control_supervisor
  -> diagnostics/reset_diagnostics
  -> bsp/board_button
  -> bsp/board_motor_safe
  -> drivers/mcu/encoder_input
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
- `drivers/device`: external devices such as ST7789, AT8236, ICM20948, and line
  sensors.
- `drivers/mcu`: MCU-facing encoder GPIO/interrupt capture. PWM, I2C, UART,
  and CAN remain future work.
- `app`: scheduling, display, commands, and mode transitions.

## Integration gates

1. Safe base: PB21/LCD/reset reporting, all motor pins high impedance.
2. Encoder shadow mode: count and speed measurement with motor output disabled.
3. Single-channel open loop: explicit arming, current-limited PWM and ramp.
4. Dual-channel mapping: motor A/E0 and B/E1 pairing and encoder signs are
   confirmed; establish chassis left/right naming during limited powered test.
5. Speed loop: migrate the tuned loop through the new motor/encoder APIs.
6. Position loop: cascade position output into the verified speed loop.
7. Yaw loop: IMU validation, then yaw output into the speed loop.
8. Line tracking: line sensor validation, then steering correction into the
   speed loop.
9. Mode arbitration: only one outer loop may own speed targets at a time.

The old tuned PID values are reference calibration data. They are promoted only
after the corresponding new driver path produces matching units, signs, sample
periods, and saturation behavior.
