# Firmware Architecture

```text
bootloader/main -> bootloader/boot_uart + bootloader/boot_flash
  -> drivers/utility/crc32 -> platform/mspm0g3507 memory layout

app/main
  -> app/car_app interaction policy
  -> app/display/car_display -> drivers/device/st7789
  -> app/services/firmware_update -> SRAM boot mailbox
     -> drivers/mcu/system_watchdog
  -> app/bringup/position_bringup_test
  -> app/bringup/speed_bringup_test
  -> app/bringup/yaw_bringup_test + app/bringup/heading_bringup_test
  -> app/bringup/line_tracking_bringup_test
  -> app/tuning/speed_tuning_console
     -> tuning_codec + tuning_status + tuning_wave + tuning_wire
     -> drivers/mcu/bluetooth_uart
  -> app/diagnostics/debug_snapshot
  -> control/control_supervisor
  -> control/wheel_position_control -> control/wheel_speed_control
  -> control/wheel_yaw_control -> control/wheel_speed_control
  -> control/wheel_heading_control -> control/wheel_speed_control
  -> control/wheel_line_tracking_control -> control/wheel_speed_control
  -> control/wheel_speed_control -> bsp/board_wheel_drive
  -> diagnostics/reset_diagnostics
  -> bsp/board_button
  -> bsp/board_motor_safe
  -> bsp/board_wheel_drive
  -> drivers/mcu/encoder_input
  -> drivers/mcu/system_watchdog
  -> drivers/device/icm20948 -> drivers/mcu/i2c0_polling
     -> drivers/utility/imu_attitude_estimator
  -> drivers/device/line_sensor -> drivers/mcu/i2c1_polling
  -> drivers/device/k230 -> drivers/mcu/vision_uart
  -> bsp/board_wheel_drive -> drivers/device/at8236
     -> drivers/mcu/motor_pwm
  -> drivers/device/st7789

control cascade
  -> speed loop -> board wheel drive
  -> position loop -> speed loop
  -> yaw loop -> speed loop
  -> heading loop -> speed loop
  -> line loop -> speed loop
  -> board wheel drive -> motor device driver -> MCU PWM driver
     -> SysConfig/platform
```

## Ownership

- `bsp`: board wiring, physical channel names, safe pin states, and the public
  left/right wheel-drive API. It owns A/B mapping and board polarity.
- `control`: reusable PID and mutually exclusive motion modes.
  `wheel_speed_control` owns the verified-unit left/right inner loops.
- `diagnostics`: reset, timing, and first-fault evidence without initiating a
  second software reset.
- `drivers/device`: external devices such as ST7789, ICM20948, and the current
  AT8236 dual-channel command layer. The ICM20948 owns register-bank selection,
  sensor setup, calibration, units, and attitude estimates. The line-sensor
  device driver owns the external eight-channel protocol and weighted error.
  The K230 vision-link driver owns framed target validation, coordinate bounds,
  link age, resynchronization, and the read-only target snapshot.
- `drivers/mcu`: MCU-facing encoder GPIO/interrupt capture, shared TIMG6/TIMG7
  motor PWM output, UART2 interrupt-RX/end-of-transmission-paced TX transport
  for JDY-31, and the polling I2C0/I2C1 transaction layers used by the IMU and
  line sensor. The independent UART3 PA13/PA14 interrupt-RX ring buffer belongs
  to the K230 transport and does not share JDY-31 ownership. It also owns
  WWDT0 initialization, refresh, fault injection, and the required handoff that
  disables the running application watchdog before entering the resident
  Bootloader. CAN remains future work.
- `drivers/utility`: hardware-independent helpers and algorithms. The IMU
  attitude estimator owns quaternion integration, gravity-vector correction,
  Euler conversion, and relative-yaw tracking; it has no I2C or board access.
- `app`: scheduling, display, commands, and mode transitions. `car_app` is the
  hardware-independent top-level interaction policy; `main` adapts its actions
  to the temporary bring-up workflows without embedding button priority or
  stop-before-start rules in the scheduler. LCD rendering, runtime bring-up
  workflows, services, and tuning protocol code are grouped in dedicated
  subdirectories. Application diagnostics own the debugger-facing `g_car_*`
  mirror and the compact display snapshot assembled from live workflow and
  control state. The tuning console owns command routing only; parsing, status
  serialization, waveform selection, and numeric wire encoding are separate
  modules with unchanged external protocol text.
- `experiments`: code retained for historical or learning value but excluded
  from production targets. The superseded open-loop motor bring-up workflow is
  archived here instead of remaining in the active application directory.
- `platform/mspm0g3507`: immutable Flash/SRAM partition constants and linker
  layouts shared by the application and resident Bootloader.
- `bootloader`: isolated UART2 115200-baud JDY-31 update protocol, safe motor-pin
  state, Flash erase/program, image validation, and application handoff.

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
9. IMU: ICM20948 identity, nominal 100 Hz accelerometer/gyro scheduling,
   startup bias calibration, six-axis quaternion estimation, fresh-sample
   reporting, and stationary zero drift are verified. With the full diagnostic
   TFT page active, the measured physical update rate is about 90 Hz.
   Dynamic hand turns and post-turn stationary locking are verified. The
   the vehicle Yaw sign convention are verified.
10. Yaw loop: the 100 Hz relative-angle controller, Yaw-owner speed slew,
    low-speed stiction recovery, Bluetooth tuning, `+/-2..135 degree` matrix,
    and ten-run alternating ground stress test are verified.
11. Line tracking: the eight-channel sensor, 100 Hz supervised outer loop,
    acute-corner recovery, optional IMU Yaw-rate assist, and the fixed
    five-corner route are validated at a requested `1400 pps`. Overall
    corner-to-straight flow remains an optimization target.
12. Mode arbitration: only one outer loop may own speed targets at a time. The
    owner must refresh its target command within 100 ms; the speed loop then
    refreshes the independent 200 ms hardware lease.
13. Hardware watchdog: WWDT0 expires after 2 seconds without a completed main
    loop pass. A watchdog reset enters suspicious-reset lockout, and the normal
    wireless updater explicitly powers down WWDT0 before its software-reset
    handoff so Bootloader erase/program operations cannot be interrupted.
14. Application coordinator: top-level `LOCKED`, `READY`, `SERVICE`, and
    `MOTION_ACTIVE` interaction states are host-tested. Any physical button
    stops the active workflow before another command can start; service and
    suspicious-reset states reject new physical motion requests.
15. Formal line mission: `app/mission/line_follow_mission` owns continuous
    product operation separately from the timed bring-up profile. It restores
    the accepted `1400 pps` route configuration on every start, refreshes the
    line-controller command lease, and runs until an explicit stop or safety
    fault returns the drive to high impedance.
16. Composite motion: `app/motion/motion_supervisor` owns the dedicated
    `MOTION` speed owner for relative-distance plus Heading commands. It is the
    composition point for future route sequencing; standalone outer loops must
    not be started concurrently to create a cascade by accident.
17. K230 vision link: UART3 PA13/PA14 receives bounded 400 x 240 target frames
    into a read-only device snapshot. Parser, timeout, resync, and overflow
    behavior are host-tested; live prediction and sustained frame progress are
    physically accepted without arming the chassis.

Host tests under `CAR_CONTROL/tests` cover pure application policies and the
tuning text codec without linking MCU drivers. They also test the K230 device
parser against a mock UART transport. Hardware-dependent control and safety
paths still require the supervised bench and ground procedures recorded in
`BRINGUP_LOG.md`.

Product code must submit wheel commands through `BoardWheelDrive_SetCommands`.
Direct `AT8236_MotorSetCommand` and `MotorPwm_SetDuty` calls are internal to the
BSP/device-driver path and are not control-loop APIs.

The old tuned PID values are reference calibration data. They are promoted only
after the corresponding new driver path produces matching units, signs, sample
periods, and saturation behavior.
