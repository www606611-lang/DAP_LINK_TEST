# Firmware Architecture

```text
app/main
  -> app/position_bringup_test
  -> app/speed_bringup_test
  -> app/speed_tuning_console -> drivers/mcu/bluetooth_uart
  -> control/control_supervisor
  -> control/wheel_position_control -> control/wheel_speed_control
  -> control/wheel_speed_control -> bsp/board_wheel_drive
  -> diagnostics/reset_diagnostics
  -> bsp/board_button
  -> bsp/board_motor_safe
  -> bsp/board_wheel_drive
  -> drivers/mcu/encoder_input
  -> drivers/device/icm20948 -> drivers/mcu/i2c0_polling
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
- `drivers/device`: external devices such as ST7789, ICM20948, and the current
  AT8236 dual-channel command layer. The ICM20948 owns register-bank selection,
  sensor setup, calibration, units, and attitude estimates. Line sensors remain
  future work.
- `drivers/mcu`: MCU-facing encoder GPIO/interrupt capture, shared TIMG6/TIMG7
  motor PWM output, UART3 interrupt-RX/nonblocking-TX transport, and the polling
  I2C0 transaction layer used by the IMU. CAN remains future work.
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
8. Position loop: a 50 Hz proportional position outer loop now converts each
   wheel's encoder-count error into a bounded speed target. One-revolution
   forward/reverse and a 24-segment multi-distance alternating suspended-wheel
   regression are validated. Position-specific slew and bounded stall recovery
   handle low-speed static friction without changing the verified speed mode.
9. IMU: ICM20948 identity, 100 Hz accelerometer/gyro sampling, startup bias
   calibration, fresh-sample reporting, and stationary zero drift are verified.
   Yaw sign under a deliberate chassis turn remains the gate before motorized
   yaw-loop work.
10. Yaw loop: verified IMU yaw output into the speed loop.
11. Line tracking: line sensor validation, then steering correction into the
   speed loop.
12. Mode arbitration: only one outer loop may own speed targets at a time. The
    owner must refresh its target command within 100 ms; the speed loop then
    refreshes the independent 200 ms hardware lease.

Product code must submit wheel commands through `BoardWheelDrive_SetCommands`.
Direct `AT8236_MotorSetCommand` and `MotorPwm_SetDuty` calls are internal to the
BSP/device-driver path and are not control-loop APIs.

The old tuned PID values are reference calibration data. They are promoted only
after the corresponding new driver path produces matching units, signs, sample
periods, and saturation behavior.
