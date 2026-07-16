# Firmware Architecture

```text
app/main
  -> app/speed_bringup_test
  -> app/speed_tuning_console -> drivers/mcu/bluetooth_uart
  -> control/control_supervisor
  -> control/wheel_speed_control -> bsp/board_wheel_drive
  -> diagnostics/reset_diagnostics
  -> bsp/board_button
  -> bsp/board_motor_safe
  -> bsp/board_wheel_drive
  -> drivers/mcu/encoder_input
  -> bsp/board_wheel_drive -> drivers/device/at8236
     -> drivers/mcu/motor_pwm
  -> drivers/device/st7789

control cascade
  -> speed loop -> board wheel drive
  -> position loop -> speed loop
  -> yaw loop -> speed loop
  -> line loop -> speed loop
  -> board wheel drive -> motor device driver -> MCU PWM driver
     -> SysConfig/platform
```

## Ownership

- `bsp`: board wiring, physical channel names, safe pin states, and the public
  left/right wheel-drive API. It owns A/B mapping and board polarity.
- `control`: reusable PID and mutually exclusive motion modes.
  `wheel_speed_control` owns the verified-unit left/right inner loops.
- `diagnostics`: reset and first-fault evidence without initiating a second
  software reset.
- `drivers/device`: external devices such as ST7789 and the current AT8236
  dual-channel command layer. ICM20948 and line sensors remain future work.
- `drivers/mcu`: MCU-facing encoder GPIO/interrupt capture, shared TIMG6/TIMG7
  motor PWM output, and UART3 interrupt-RX/nonblocking-TX transport. I2C and CAN
  remain future work.
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
   and reset behavior. Ten consecutive tests completed without reset.
6. Wheel-drive API: promote left/right forward-positive commands into BSP,
   migrate the open-loop test to it, and repeat the bench test before reuse.
   The regression passed and was committed as `3be24c2`.
7. Speed loop: the 100 Hz, pps-based dual PI API, four-sample encoder-speed
   filter, supervised bench profiles, Bluetooth tuning, and full-battery
   regression are verified.
8. Position loop: cascade position output into the verified speed loop.
9. Yaw loop: IMU validation, then yaw output into the speed loop.
10. Line tracking: line sensor validation, then steering correction into the
   speed loop.
11. Mode arbitration: only one outer loop may own speed targets at a time. The
    owner must refresh its target command within 100 ms; the speed loop then
    refreshes the independent 200 ms hardware lease.

Product code must submit wheel commands through `BoardWheelDrive_SetCommands`.
Direct `AT8236_MotorSetCommand` and `MotorPwm_SetDuty` calls are internal to the
BSP/device-driver path and are not control-loop APIs.

The old tuned PID values are reference calibration data. They are promoted only
after the corresponding new driver path produces matching units, signs, sample
periods, and saturation behavior.
