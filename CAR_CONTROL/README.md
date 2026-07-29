# CAR_CONTROL

Formal MSPM0G3507 car-control firmware, rebuilt from the clean PB21/LCD bring-up
baseline. The older `DAP_LINK_TEST` and `DIANSAI_MOTOR_DRIVER_BOARD` projects are
reference sources only; they are not copied wholesale into this target.

The selected competition task is the official 2026 Jiangsu TI Cup
`车载平衡滚球运动控制系统（H题）`. Exact requirements, ownership decisions,
engineering margins, and physical acceptance gates are recorded in
[`H_PROBLEM_EXECUTION_PLAN.md`](H_PROBLEM_EXECUTION_PLAN.md). That plan
overrides older generic competition assumptions; existing line, motion, K230,
and wireless work is reusable infrastructure rather than proof of an H score.

## Source layout

```text
app/main.c + app/car_app.*   top-level scheduler and interaction policy
app/bringup/                 optional validation workflows, excluded by default
app/diagnostics/             application/debugger state mirror
app/display/                 sliced ST7789 application dashboard
app/services/                application services such as firmware update
app/tuning/                  Bluetooth routing, codec, status, and wave protocol
control/                     reusable control algorithms and cascaded loops
bsp/                         board wiring, wheel mapping, and safe resources
drivers/device/              external-device protocols
drivers/mcu/                 MCU peripheral drivers
drivers/utility/             hardware-independent helpers
diagnostics/                 reset and runtime timing evidence
experiments/                 code excluded from production targets
```

The superseded open-loop motor workflow is retained under
`experiments/legacy_motor_bringup` for historical reference and is not linked
into `car_control`.

## Current safety baseline

- PB21, SW2/PB4, and SW1/PB5 are active-low with a 20 ms software
  debounce. PB4/PB5 use the board's 200 kOhm pull-ups and 100 nF capacitors
  plus MCU internal pull-ups.
- ST7789 SPI display support is enabled.
- PA2/TIMG8_CCP1 controls an external active-high MOS switch for a 5 V
  electromagnet. `Electromagnet_Grip()` applies full GPIO power continuously;
  `Electromagnet_Release()` immediately forces the output low. Intermediate
  hold duties remain available for later hardware characterization, but the
  validated default does not reduce the holding force. Bluetooth
  commands `mag grip`, `mag release`, `mag get`, `mag set PULL_MS HOLD`, and
  `mag stat` expose the product workflow. `mag on` and `mag pulse <ms>` remain
  diagnostic commands. Activation from the tuning console is accepted only
  while the wheel motors are in `HIGH-Z`; startup and wireless update always
  release the electromagnet.
- Motor control nets are confirmed as:
  - channel A: `PA29 -> AIN1`, `PA30 -> AIN2`
  - channel B: `PA23 -> BIN1`, `PA24 -> BIN2`
- TIMG6 provides 20 kHz PWM for motor A, and TIMG7 provides 20 kHz PWM for
  motor B. Both channels are enabled only during a supervised motion mode.
- All four motor pins return to high impedance at startup, when any of the
  three buttons is pressed during motion, after the speed-command lease
  expires, at the test endpoint, and on every emergency stop.
- Motor A/E0 is the validated left wheel and motor B/E1 is the validated right
  wheel. PB21 still starts the supervised `1400 pps` line-following mission for
  chassis-only validation; that endless mission is not the H-route state
  machine and is not competition acceptance. PB4/PB5 no longer start the old
  relative Yaw demonstrations from idle. Pressing any key during motion stops
  immediately; commands are never stacked while moving.
- PB21, PB4, and PB5 use both-edge GPIOB interrupts with 5 ms press and 30 ms
  release debounce. The longer release qualification prevents switch bounce
  from re-arming a second command. The board-level GPIOB dispatcher drains
  every pending source and routes PB0-PB3 to the encoder driver.
- IMU integration uses the measured sample interval through display and UART
  stalls instead of replacing every interval above 20 ms with 10 ms. Only an
  abnormal interval above 100 ms is capped, preventing systematic Yaw loss.
- The physical-button Yaw path enters a short `ARM` state so button vibration
  can settle before motor start; it no longer rejects the command just because
  the IMU temporarily reports motion at the press edge. The LCD dedicates its
  main area to current Yaw, target, error, angular rate, elapsed test time,
  control state, physical-button state, relative command, and motor safety.
- The provisional Z-gyro scale correction is `1.0588`, derived from the first
  post-timing-fix observation of 85 degrees for a physical 90-degree turn.
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
- WWDT0 provides a 2 second hardware watchdog. The main loop refreshes it only
  after completing one full scheduling pass and immediately before sleeping.
  A watchdog reset is classified as suspicious and boots with motion locked;
  debugger halt pauses the watchdog.
- UART2 provides detached speed-loop tuning at 115200 baud: PB17 is MCU TX and
  PA22 is MCU RX. Applying parameters does not start either motor.

Direct mode requests remain blocked. Verified outer controllers enter
position, yaw, heading, or line-tracking mode only through the supervised
speed-loop owner API, so unimplemented modules cannot arm the motors by
changing a mode.

## Application coordinator

`app/car_app.h` is the hardware-independent top-level interaction policy. It
tracks `LOCKED`, `READY`, `SERVICE`, and `MOTION_ACTIVE` states and reports the
currently active bring-up workflow. A suspicious reset stays locked, while
JDY-31 configuration and firmware update enter service state and reject new
physical-button motion requests.

When a supervised motion workflow is active, any of the three board buttons
produces only `STOP_ACTIVE`; it cannot immediately start a different motion.
When idle, PB4/PB5 produce no motor action. PB21 retains the supervised manual
line mission only until the H2-H6 coordinator replaces it. `main.c` remains a
thin adapter from policy actions to validated workflows. The read-only
`app stat` command and CLI `AppStatus` action expose state, workflow, last
action, transition count, and motor high-Z state.

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
`POSITION`, `YAW`, `HEADING`, or `LINE_TRACKING` owns the inner loop. The owner
must submit fresh targets at least every 100 ms; the inner loop refreshes a
separate 200 ms hardware supervisor lease only while it remains healthy.
Invalid targets, stale outer-loop commands, encoder access failure, output
failure, or ownership loss return the board to high impedance.

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

## Reusable heading-control API

`control/wheel_heading_control.h` combines a linear base-speed command with a
differential Yaw correction while retaining the validated wheel-speed inner
loop:

```c
WheelHeadingControl_SetConfig(&config);
WheelHeadingControl_StartHoldCurrent(base_speed_pps, now_ms);
WheelHeadingControl_SetCommand(target_yaw_deg, base_speed_pps, now_ms);
WheelHeadingControl_Task(now_ms);
WheelHeadingControl_Stop(reason);
WheelHeadingControl_GetSnapshot(&snapshot);
```

The mixer uses `left = base - correction` and
`right = base + correction`. Unlike the one-shot pivot Yaw controller, Heading
control is continuous, has no minimum correction, and does not stop merely
because it reaches the target. Its caller must refresh the target and base
speed within 100 ms. A stale command, stale IMU sample, inner-loop ownership
loss, or invalid target stops both wheels through the supervisor.

The ground-validated defaults are `Kp=30`, `Ki=3`, `Kd=1.5`, a `0.5 degree`
deadband, and a `400 pps` maximum correction. The bring-up profile drives at
`1200 pps` for `6 s` with a `650 permille` PWM limit. Bluetooth commands are
`heading get/set/run/stat/stop`; CLI runs save `latest_heading_status.json` and
`latest_heading_telemetry.csv`. During Heading motion, the existing seven
VOFA+ Yaw channels carry target, current, error, rate, correction, and the two
measured wheel speeds.

## Reusable line-sensor API

`drivers/device/line_sensor/line_sensor.h` owns the external eight-channel
module protocol and uses the polling I2C1 driver on `PA16/PA17`:

```c
LineSensor_Init(now_ms);
LineSensor_Task(now_ms);
LineSensor_RequestCalibration(now_ms);
LineSensor_GetSnapshot(&snapshot);
```

The board connector is `5 V, PA17/SCL, PA16/SDA, GND`. The module uses 7-bit
address `0x12`, control register `0x01`, and data register `0x30`. Channels are
active-low and use weights `-35, -25, -15, -5, 5, 15, 25, 35`, so left is
negative and right is positive. Sampling runs every `10 ms`; calibration and
boot delays are nonblocking. `line_seen` is the validity flag for
`line_error`: when the line is lost, the last error is intentionally retained
for a future recovery policy and must not be treated as a current observation.

Bluetooth commands `line stat` and `line cal` expose sensor and I2C health
without arming the motors. The CLI actions are `LineStatus` and `LineCal`, and
the latest parsed state is written to `latest_line_status.json`. At idle the
LCD footer shows the eight active bits, signed error, and `LINE`, `MISS`,
`CAL`, or `ERR`.

## Reusable chassis-radio shadow API

`drivers/device/chassis_radio/chassis_radio_link.h` owns the read-only wireless
health state. UART3 uses PA13 RX and PA14 TX at 115200 baud to an ESP32-C3
bridge; both UART and UDP use the bounded binary frame documented in
`K230_CAN_INTEGRATION_PLAN.md`. The public API is:

```c
ChassisRadioLink_Init(now_ms);
ChassisRadioLink_SetStatusFlags(CHASSIS_RADIO_STATUS_HIGH_Z);
ChassisRadioLink_Task(now_ms);
ChassisRadioLink_GetSnapshot(&snapshot);
```

The snapshot exposes ESP32 and K230 health separately, independent sequence
validation, frame age, CRC/length/version errors, UART overflows, duplicates,
out-of-order frames, timeouts, and shadow-command count. `k230 stat` and CLI
action `K230Status` retain their compatibility names but now report this radio
state. The LCD health row shows ESP32/K230 status and link age. CONTROL and
EMERGENCY_STOP IDs remain shadow-only until a later supervised owner is
physically accepted.

The bidirectional shadow path was accepted on production chassis 5 V power on
2026-07-24. K230 reported `esp=1` and `chassis=1` for 30 seconds while receiving
289 frames and sending 109 frames with zero CRC, length, duplicate,
out-of-order, or socket errors. The independent endpoint-reset matrix was
accepted on 2026-07-25: K230 power cycling recovered in about 4.2 seconds after
the monitor started, ESP32-C3 reset recovered in about 9.0 seconds, and
Tianmengxing reset recovered the complete path in about 2.1 seconds. The K230
receiver rebases only a role whose accepted frame age has exceeded the offline
timeout, preventing a restarted peer's reset sequence from being rejected
forever while retaining normal duplicate and out-of-order rejection. Every
case ended `READY / HIGH-Z` with zero protocol errors. This acceptance does not
grant wireless motion ownership.

## Reusable line-tracking API

`control/wheel_line_tracking_control.h` owns the supervised line-tracking
outer loop and submits left/right targets only through the validated speed
controller:

```c
WheelLineTrackingControl_SetConfig(&config);
WheelLineTrackingControl_Start(base_speed_pps, line_error, active_count,
    line_seen, observation_ms, now_ms);
WheelLineTrackingControl_SetCommand(base_speed_pps, line_error, active_count,
    line_seen, observation_ms, now_ms);
WheelLineTrackingControl_Task(now_ms);
WheelLineTrackingControl_Stop(reason);
WheelLineTrackingControl_GetSnapshot(&snapshot);
```

The promoted defaults are `Kp=30`, `Ki=0`, `Kd=0`, a `900 pps` correction
limit, and a `2`-unit deadband. The bring-up profile uses a `1400 pps` base,
`750 permille` output limit, and `30000 ms` minimum duration. A 100 ms command
lease, 60 ms observation-age limit, non-corner line-loss stop, direction-locked
acute-corner recovery, and board-button stop remain active. A test reports
`DONE` only after the line is stably centered; expiry of its three-second finish
grace reports `ABORT` and returns to high impedance.

## Formal continuous line-following mission

`app/mission/line_follow_mission.h` is the product workflow above the reusable
line controller. Unlike the 30-second bring-up profile, it has no normal time
limit and continues until an operator stop or a supervised fault. Every start
restores the accepted `1400 pps`, `Kp=30`, `Ki=0`, `Kd=0`, `900 pps` maximum
correction, `750 permille` output limit, and `2`-unit deadband baseline. It
never starts automatically after boot or a wireless update.

```text
mission start
mission stop
mission stat
```

`mission stat` returns an `MSTAT` record. The existing `LSTAT` bring-up status
and seven-channel `linewave` VOFA+ format remain unchanged. Any board button
stops an active mission before another button command can start.

## Composite motion API

`app/motion/motion_supervisor.h` is the first shared upper-layer motion
interface. It owns the speed loop through the dedicated `MOTION` supervisor
mode, so a position command cannot silently fight a Heading, Yaw, or line
mission owner. The first supported action is relative encoder distance with
either an absolute target heading or the heading captured at start:

```text
motion start DELTA_COUNTS HEADING_DEG MAX_SPEED_PPS TIMEOUT_MS
motion start DELTA_COUNTS hold MAX_SPEED_PPS TIMEOUT_MS
motion stop
motion stat
```

The action applies a bounded distance correction and Heading correction at the
same outer layer, refreshes the existing speed command lease, requires fresh
encoder and IMU snapshots, and stops at the distance/heading settle window.
It is intentionally explicit and does not start from boot or from a board
button. Future route sequences and competition logic should call this API
instead of starting the standalone position and Heading owners together.

## Reusable ICM20948 API

`drivers/device/icm20948/icm20948.h` owns the external IMU protocol and uses
the polling I2C0 driver on `PA0/PA1`:

```c
ICM20948_Init(now_ms);
ICM20948_Task(now_ms);
ICM20948_GetSnapshot(&snapshot);
ICM20948_IsReady();
ICM20948_ResetYaw();
```

Initialization probes `0x69` and then `0x68`, requires `WHO_AM_I=0xEA`, and
performs a 400-sample stationary gyro calibration. The application nominally
schedules samples every 10 ms. Snapshot acceleration is in g, angular rate in
degrees per second, temperature in degrees Celsius, and roll/pitch/yaw in
degrees. A hardware-independent six-axis quaternion estimator in
`drivers/utility/imu_attitude_estimator` fuses the accelerometer and gyro. Its
accelerometer correction is reduced when the measured gravity magnitude is
implausible, limiting false tilt during chassis acceleration. The public yaw is
relative to startup or the last `imu zero`; after stationary detection it is
held while the online gyro-bias estimate continues converging. This preserves
the validated zero-drift behavior without pretending that a six-axis estimate
is a magnetic heading.

Startup, configuration, calibration, estimator validity, stationary state,
quaternion, sample age, and read failures remain visible through explicit
snapshot fields. The driver retries an offline device once per second. The
Bluetooth console accepts `imu stat` and `imu zero`; these commands do not arm
either motor. The SPI display is intentionally a simple user view: three
large, centered Roll/Pitch/Yaw values plus `IMU READY/ERROR`, `YAW
LOCKED/MOVING`, and motor `HIGH-Z/ARMED`. Acceleration, gyro, bias, quaternion,
sample-age, and error diagnostics remain available over the console and debug
globals without crowding the physical screen. Whole-line DMA text drawing
keeps the physical sensor task near its nominal update rate.

## Bluetooth speed tuner

`tools/speed_tuner/Launch-SpeedTuner.cmd` opens the Windows GUI. Connect a
3.3 V compatible Bluetooth UART module as follows:

```text
PB17 / UART2 TX -> Bluetooth RX
PA22 / UART2 RX <- Bluetooth TX
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
The command-line companion also provides `-Action ImuStatus` and
`-Action ImuZero` through the same TCP bridge or by direct serial access.
The GUI also has a Yaw-loop tab for all outer-loop parameters, live angle,
Yaw rate, wheel targets/speeds, output, result, and high-impedance state. It
forwards the seven-channel `yaw` FireWater group to VOFA+ and records the
latest Yaw wave, status, and telemetry under the same runtime directory.
The line tab exposes the promoted controller and route-profile parameters.
Line status JSON/CSV includes target Yaw rate, measured Yaw rate, correction
boost, and IMU-valid state without changing the seven-channel VOFA+ format.
The TCP bridge also accepts `wdt stat` while the application is idle. It
reports whether refreshes are active, the refresh count, and motor high-Z
state. `wdt test` is a deliberate reset-injection command and is accepted only
while the motors are already high impedance; after about 2 seconds the board
must reboot into suspicious-reset lockout. The CLI actions are
`WatchdogStatus` and `WatchdogTest`.

## Build and flash

The normal GCC and TIClang builds are competition-safe by default and set
`CAR_ENABLE_BRINGUP=OFF`. The validated control and driver APIs remain linked;
only their temporary test workflows are replaced by disabled entry points.

```powershell
# Normal product build.
cmake --preset gcc-debug
cmake --build build-gcc --target car_control -j

# Explicit bring-up build for a bounded laboratory retest.
cmake --preset gcc-debug -D CAR_ENABLE_BRINGUP=ON
cmake --build build-gcc --target car_control -j

# Restore the product-safe cache before wireless installation.
cmake --preset gcc-debug -D CAR_ENABLE_BRINGUP=OFF
cmake --build build-gcc --target car_control -j

# Isolated product directory remains available for release comparison.
cmake --preset gcc-product
cmake --build build-product-gcc --target car_control -j
```

The matching TIClang presets are `ticlang-debug` and `ticlang-product`.
Yaw, speed, position, Heading, and timed line bring-up applications are all
controlled by the same option. The production scheduler calls the line-sensor
driver directly; the former forwarding-only `line_sensor_bringup` wrapper was
removed. Product builds reject excluded test commands without arming motors.

The normal development path is the resident JDY-31 updater on `COM6`. From the
repository root:

```powershell
cmake --preset gcc-debug
cmake --build build-gcc --target car_control -j
powershell -NoProfile -ExecutionPolicy Bypass `
  -File CAR_CONTROL\tools\FirmwareUpdater.ps1 `
  -Port COM6 -Image build-gcc\CAR_CONTROL\car_control.bin
```

In VS Code, use the wireless build/update task. The tuner must first report
`HIGH-Z` and release COM6; restart it after the updater completes.

The resident wireless updater uses this protected Flash layout:

```text
0x00000000..0x00002FFF  car_bootloader (12 KiB, J-Link install only)
0x00003000..0x0001FBFF  relocatable car_control application
0x0001FC00..0x0001FFFF  update state, image size, and CRC32
```

Both supported toolchains compile the application and Bootloader with `-Os`.
The linker memory regions remain the hard size limit, so an oversized image
fails the build instead of overlapping the Bootloader or metadata page.

`Install Bootloader + App (J-Link, One Time)` performs the one-time full-chip
installation. After that, close any program that owns COM6 and run `Build +
Wireless Update (COM6)` from the VS Code task list. The current approximately
73.5 KiB GCC image transfers through the JDY-31 at 115200 baud in about
14 seconds. `fw update`
is accepted only while all motor outputs are high impedance. Each 1024-byte
frame and the completed image have independent CRC32 checks; an interrupted
update leaves the Bootloader resident and ready for the same task to be run
again.

Before writing the SRAM update mailbox and requesting the software reset, the
application resets and powers down WWDT0. This is required because an MSPM0
watchdog can otherwise remain active across the software reset and interrupt
the resident Bootloader during Flash erase or programming. WWDT0 is configured
again when the new application starts.

`flash_gcc.jlink` is now application-only: it erases `0x3000..0x1FFFF` and
cannot erase the resident Bootloader. Do not replace it with a full-chip erase
for normal development.

## Debug globals

```text
g_car_pb21_pressed
g_car_pb21_press_count
g_car_pb21_interrupt_count
g_car_pb4_pressed
g_car_pb4_press_count
g_car_pb4_interrupt_count
g_car_pb5_pressed
g_car_pb5_press_count
g_car_pb5_interrupt_count
g_car_last_button_id
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
g_car_imu_ready
g_car_imu_state
g_car_imu_result
g_car_imu_address7
g_car_imu_who_am_i
g_car_imu_sample_count
g_car_imu_read_error_count
g_car_imu_sample_age_ms
g_car_imu_ax_mg
g_car_imu_ay_mg
g_car_imu_az_mg
g_car_imu_gx_mdps
g_car_imu_gy_mdps
g_car_imu_gz_mdps
g_car_imu_roll_mdeg
g_car_imu_pitch_mdeg
g_car_imu_yaw_mdeg
g_car_imu_yaw_rate_mdps
g_car_imu_accel_norm_mg
g_car_imu_bias_x_mdps
g_car_imu_bias_y_mdps
g_car_imu_bias_z_mdps
g_car_imu_quaternion_w_million
g_car_imu_quaternion_x_million
g_car_imu_quaternion_y_million
g_car_imu_quaternion_z_million
g_car_imu_attitude_valid
g_car_imu_stationary
g_car_line_sensor_state
g_car_line_sensor_raw
g_car_line_sensor_active_mask
g_car_line_sensor_active_count
g_car_line_sensor_error
g_car_line_sensor_sample_count
g_car_line_sensor_read_error_count
g_car_line_sensor_calibration_count
g_car_line_sensor_last_result
g_car_line_sensor_seen
g_car_line_sensor_ready
g_car_encoder_count_difference
g_car_encoder_speed_difference_pps
g_car_app_state
g_car_app_active_workflow
g_car_app_last_action
g_car_app_transition_count
```

See `HARDWARE_MAP.md` before enabling any additional peripheral and
`ARCHITECTURE.md` for the staged integration order. Bench observations and the
remaining manual checks are recorded in `BRINGUP_LOG.md`.
The ordered productization and acceptance workflow is in
`OPTIMIZATION_PLAN.md`.

## Host regression tests

Pure application policies that do not require MCU hardware are built and run
separately with the Windows host compiler:

```powershell
cmake -S CAR_CONTROL/tests -B build-host-tests `
  -G "Visual Studio 17 2022" -A x64
cmake --build build-host-tests --config Debug
ctest --test-dir build-host-tests -C Debug --output-on-failure
```

The current suite verifies strict line-test completion, center-stability reset,
grace timeout failure, 32-bit millisecond-counter wraparound, application-state
transitions, service/reset lockout, deterministic workflow priority, physical
button action mapping, stop-before-start behavior, tuning tokenization, numeric
limits, and every supported bring-up profile keyword.
