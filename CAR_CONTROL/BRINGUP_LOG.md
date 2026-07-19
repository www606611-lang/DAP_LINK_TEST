# Bring-up Log

## 2026-07-15: safe base

- GNU Arm, TI Clang CMake, and CCS headless builds passed.
- CCS result: 0 errors, 0 warnings.
- J-Link programmed and verified the GNU ELF.
- Runtime state after approximately 62 seconds:
  - PB21 released, press count 0
  - control mode `SAFE_IDLE`
  - block reason `HW_UNVERIFIED`
  - motor outputs high impedance
  - CPU in thread mode, no active fault

## 2026-07-15: encoder shadow mode

Both old firmware trees configure PB0-PB3 as encoders. An early shadow build
using those pins produced activity while the wheels were stationary:

```text
legacy channel 0: count -1596, edges 1954, invalid 78
legacy channel 1: count 0,     edges 892,  invalid 0
```

That activity was initially treated as evidence that PB0-PB3 were wrong. This
was an incorrect conclusion: disconnected or unpowered encoder inputs can
float. A later local export was also misinterpreted as the physical MCU pin
mapping, resulting in this invalid test configuration:

```text
invalid channel 0: PA21 A, PB20 B
invalid channel 1: PA18 A, PB24 B
```

The observation at approximately 14.7 seconds on that invalid configuration
was:

```text
channel 0: count 0, speed 0 pps, edges 12, invalid 0
channel 1: count 0, speed 0 pps, edges 0,  invalid 0
motor high impedance: 1
encoder shadow active: 1
```

At approximately 29.7 seconds, both counts, speeds, invalid totals, and edge
totals were unchanged. This result is not encoder validation.

## 2026-07-15: physical encoder mapping correction

The physical board and both legacy SysConfig files agree on the four encoder
inputs:

```text
channel 0 / legacy left:  PB0 A, PB1 B
channel 1 / legacy right: PB2 A, PB3 B
PB4: legacy direction key `up`, not an encoder input
```

At that stage, `CAR_CONTROL` used PB0-PB3 on one GPIOB interrupt group, while
motor-channel pairing and positive direction still required a hand-turn test.

## 2026-07-15: powered encoder hand-turn validation

Both wheels were turned in the vehicle-forward direction. The display showed:

```text
channel 0 PB0/PB1: count -1033, speed 0 pps, edges 1057, invalid 0
channel 1 PB2/PB3: count  1058, speed 0 pps, edges 1068, invalid 0
uptime: 32 s, reset cause: BOOT_EXTERNAL_NRST
```

Both encoder channels, both-edge GPIO interrupts, and quadrature decoding are
working. Zero invalid transitions indicate a clean sequence. Speed was zero
because the wheels had stopped before the photo. Channel 0 is inverted in the
board configuration so the control convention is now forward-positive for
both channels; channel 1 keeps its native sign.

## 2026-07-15: five-revolution encoder calibration

After applying the channel-0 direction normalization, both wheels were turned
forward by five revolutions from zero:

```text
channel 0 PB0/PB1: count 5310, edges 5322, invalid 0
channel 1 PB2/PB3: count 5289, edges 5291, invalid 0
uptime: 69 s, reset cause: BOOT_EXTERNAL_NRST
```

The measured values are 1062.0 and 1057.8 counts per wheel revolution, with a
mean of 1059.9. The board calibration constant is therefore 1060 quadrature
counts per wheel revolution. Both forward signs are positive and neither
channel reported an invalid transition.

## 2026-07-15: motor and encoder channel pairing

The physical motor-to-encoder pairing was confirmed:

```text
motor channel A -> encoder channel 0 (PB0/PB1)
motor channel B -> encoder channel 1 (PB2/PB3)
```

Chassis left/right naming remains intentionally separate from this electrical
channel pairing.

## 2026-07-15: initial supervised motor-A open-loop firmware

The first motor output stage used these conservative test parameters:

```text
motor channel: A only
PWM peripheral: TIMG6, 20 kHz
pins: PA29 AIN1, PA30 AIN2
command: fast-decay forward, 0 to 600 permille over 400 ms
hold: 600 permille for 200 ms
maximum active time: 600 ms
supervisor lease: 700 ms
motor B PA23/PA24: input high impedance throughout
```

The bench test produced a brief motor reaction but no sustained rotation. This
matches the earlier observation that the motor begins moving near 60%: the
firmware reached that threshold only at the end of the 400 ms ramp and held it
for approximately 200 ms.

## 2026-07-15: simplified motor-A test

The interaction and timing were revised:

```text
first PB21 press: start motor-A test
second PB21 press while running: immediate high-impedance stop
ramp: 0 to 700 permille over 500 ms
total run time: 3000 ms
independent supervisor lease: 3200 ms
motor B PA23/PA24: input high impedance throughout
```

Reaching the 3 second endpoint, exceeding the 3.2 second supervisor lease, or
encountering an output error immediately forces all motor pins back to high
impedance. BOR, flash ECC, watchdog, clock, parity, and CPU-lockup reset causes
still lock the test at boot.

### First bench procedure

1. Suspend the wheels and connect only the confirmed motor-A power path.
2. Verify VM, 5 V, 3.3 V, VREF, and common ground before pressing PB21. Do not
   power the same rail from J-Link and MP1584 simultaneously.
3. Press PB21 once. The display should show `RUN`, then `DONE` after 3 seconds.
   Pressing PB21 again during `RUN` must show `ABORT` and stop immediately.
4. Confirm motor A runs briefly, E0 becomes positive, E1 remains unchanged,
   `INVALID` stays zero, and the display returns to `HIGH-Z` without rebooting.
5. If the display restarts, record the new reset cause before another attempt.

### Motor-A bench result

The user confirmed sustained normal operation of the left wheel. The physical
mapping is therefore:

```text
motor A / TIMG6 / PA29-PA30 -> encoder E0 / PB0-PB1 -> left wheel
```

## 2026-07-15: supervised motor-B test firmware

The PWM and AT8236 layers were generalized for both channels. The active test
target is now motor B only:

```text
target: motor B / TIMG7 / PA23-PA24
feedback: encoder E1 / PB2-PB3
ramp: 0 to 700 permille over 500 ms
total run time: 3000 ms
independent supervisor lease: 3200 ms
motor A PA29/PA30: high impedance throughout
```

One PB21 press starts the test, a second press stops immediately, and every
stop path disables both PWM timers before returning all four motor inputs to
high impedance.

### Motor-B bench procedure

1. Suspend both wheels and verify VM, 5 V, 3.3 V, VREF, and common ground.
2. Press PB21 once. The header and motor line must identify `M:B`, and the
   state should show `RUN` followed by `DONE` after 3 seconds.
3. Confirm only the right wheel moves, E1 changes while E0 remains unchanged,
   and `INVALID` stays zero.
4. Record whether forward wheel rotation produces positive or negative E1.
5. Confirm the display does not reboot; if it does, record the reset cause.

### Motor-B bench result

The right wheel ran normally and the controller did not reset. Under the
initial positive motor command, E1 counted negative. Because the powered wheel
direction was forward and the hand-turn encoder convention was already
forward-positive, the correction belongs to motor-B drive polarity:

```text
motor B / TIMG7 / PA23-PA24 -> encoder E1 / PB2-PB3 -> right wheel
motor-B logical forward command: hardware polarity inverted
reset during test: none
```

The board configuration now inverts motor B commands. The powered retest
confirmed that the right wheel moves forward, E1 counts positive, and no reset
occurs.

## 2026-07-15: supervised dual-motor test firmware

Both single-channel paths now have confirmed motor/encoder pairing, logical
forward direction, positive feedback, and stable reset behavior. The active
test therefore advances to both channels:

```text
motor A / left:  ramp 0 to 700 permille from 0 to 500 ms
motor B / right: ramp 0 to 700 permille from 0 to 500 ms
automatic stop: 3000 ms from test start
supervisor lease: 3200 ms from test start
```

Both channels are armed under the same supervisor request. A second PB21 press,
an output error, the 3 second endpoint, or the independent lease expiry stops
both PWM timers and returns all four AT8236 input pins to high impedance. The
LCD shows both commands plus E0-E1 count and speed differences.

### Dual-motor bench procedure

1. Suspend both wheels and verify VM, 5 V, 3.3 V, VREF, and common ground.
2. Press PB21 once. Confirm both wheels start together in the vehicle-forward
   direction and the display reaches `A:700 B:700`.
3. During the common 70% hold interval, record E0/E1 speeds and their displayed
   difference, then compare the final accumulated counts.
4. Confirm both counts are positive, `INV` remains `0/0`, the test reaches
   `DONE`, all motor pins return to `HIGH-Z`, and uptime does not restart.
5. Press PB21 again during another run and confirm both wheels stop immediately
   with `ABORT` and `USER_STOP` shown.

### Dual-motor bench result

Ten consecutive tests completed without a reset. Both encoder signs and motor
directions were correct. The initial firmware deliberately delayed motor B by
200 ms to reduce startup current, but that caused visible chassis yaw at every
start and is not acceptable for vehicle control. With the power path now proven
stable, the delay has been removed and both motors share the same ramp command
and start time. Remaining wheel-to-wheel mechanical mismatch will be handled by
the speed loop, not by an open-loop start delay.

## 2026-07-15: wheel-drive API promotion

The verified open-loop behavior was promoted into a reusable board-facing API:

```text
BoardWheelDrive_SetCommands(left, right)
range: -1000 to 1000 permille
sign: positive is vehicle-forward for both wheels
mapping: left -> motor A, right -> motor B
failure behavior: zero both wheel commands and return an explicit result
```

Motor A/B selection and the motor-B polarity inversion now belong to the BSP,
not the bring-up application. `MotorBringupTest` uses this public API, while
AT8236 and TIMG6/TIMG7 functions remain internal driver details. Emergency stop
zeros the public command state before disabling both PWM peripherals and
returning all four motor pins to high impedance.

The API regression completed normally: full-run and operator-stop paths kept
both feedback signs positive, `INV=0/0`, high-impedance stop, and stable uptime.
The milestone was committed and pushed as `3be24c2`.

## 2026-07-15: initial supervised speed-loop firmware

The active firmware now exercises the formal dual-wheel speed-control API. It
uses the existing 50 ms encoder snapshots and preserves the previously tuned
pps-domain gains as reference calibration:

```text
left PI(D):  Kp 0.072, Ki 0.095, Kd 0.001, integral limit +/-5000
right PI(D): Kp 0.13,  Ki 0.48,  Kd 0.001, integral limit +/-2500
deadband: 12 pps
formal target range: -6000 to 6000 pps
formal default output range: -1000 to 1000 permille
```

The first bench application adds conservative test-only limits:

```text
target: both wheels ramp 0 to 3500 pps over 1000 ms
output cap: both wheels 700 permille
total run time: 5000 ms
speed-command lease: 200 ms, refreshed after each valid control update
```

The PI implementation, PB21 workflow, and display are separate modules. The
test application only changes targets and decides when to stop; PID state,
encoder feedback, paired output, and command-lease refresh belong to
`WheelSpeedControl`.

### Initial speed-loop bench procedure

1. Suspend both wheels. Do not perform this first test on the floor because the
   inherited left/right gains and motor dead zones may start at different times.
2. Verify VM, 5 V, 3.3 V, VREF, common ground, and `HIGH-Z`, then press PB21.
3. Confirm the display enters `SPEED/RUN`, both targets reach 3500, `RES` stays
   zero, both encoder signs remain positive, and `INV` remains `0/0`.
4. During the final two seconds, record both `V`, `E`, and `O` values. Note any
   delayed start, output pinned at 700, oscillation, or persistent speed error.
5. Confirm the test reaches `DONE/TEST_DONE` after 5 seconds without reset and
   returns to `HIGH-Z`.
6. Run again and press PB21 during motion. Both wheels must stop immediately
   with `ABORT/USER_STOP` and stable uptime.

### Inherited-tuning bench result

The first speed-loop run was not acceptable. The left wheel lagged severely
and peaked near 2000 pps. The right wheel approached the target more closely
but oscillated. The inherited gains were intentionally not committed:

```text
left:  Kp 0.072, Ki 0.095, Kd 0.001
right: Kp 0.13,  Ki 0.48,  Kd 0.001
```

This board and corrected PWM path require fresh identification. The next build
uses a symmetric P-only baseline on both wheels:

```text
Kp 0.18, Ki 0, Kd 0
target: 3500 pps
output cap: 700 permille
```

At zero speed the proportional command is approximately 630 permille, just
above the observed starting region. Steady-state error is expected; this test
only compares the two wheel plants and identifies a usable proportional gain.

### Symmetric P-only bench result

Both wheels started and moved with similar timing, confirming that a symmetric
baseline is appropriate for the current board. The response remained near the
600-permille dead-zone boundary and could not continue toward the target. This
is expected because proportional output falls as measured speed rises.

The next build keeps `Kp=0.18` and `Kd=0` on both wheels and adds the same
conservative `Ki=0.10`. With an integral limit of 5000, integral contribution
is capped at 500 permille. No per-wheel compensation is introduced in this
iteration.

## 2026-07-15: detached Bluetooth speed tuner

The current working baseline was advanced to symmetric `Kp=0.25`, `Ki=0.10`,
and `Kd=0`. A RAM-only tuning path now avoids reflashing for every change:

```text
UART3: PA26 TX -> Bluetooth RX, PA25 RX <- Bluetooth TX
format: 9600 8N1, matching the legacy UART3 generated configuration
commands: spd get, spd set, spd run, spd stop, spd stat
runtime fields: Kp, Ki, Kd, target pps, output limit permille
```

Configuration is range-checked in full before it is applied to both loops.
Changing configuration never starts motion and is rejected while a run is
active. PB21 operator stop, remote stop, the five-second endpoint, the 200 ms
lease, control faults, and suspicious-reset lockout retain their existing
high-impedance behavior. The Windows tool is in `tools/speed_tuner`.

This implementation has passed the GCC build and SysConfig validation. The
Bluetooth link and resulting motor response remain pending bench confirmation.

### UART3 sustained-command correction

The first Bluetooth GUI test received valid status frames mixed with repeated
command errors. Raw-line diagnostics showed each failed `spd stat` arrived as
hex `746174`, or only the final `tat`. The polling receiver lost the leading
bytes while the main loop was occupied by LCD updates and the four-entry UART
FIFO overflowed.

UART3 RX now uses the one-entry FIFO interrupt threshold and a 128-byte MCU
driver ring buffer. Parsing remains in the application task, and TX uses the
existing nonblocking 512-byte queue. A six-second 400 ms polling stress test
returned one valid configuration and 15 complete status frames with no command
errors, `INV=0/0`, `RES=0`, and `HIGH-Z` throughout.

## 2026-07-16: verified 100 Hz speed loop and cascade contract

The final suspended-wheel regression uses symmetric `Kp=0.12`, `Ki=0.05`,
`Kd=0`, a signed `487 + 0.031 * abs(target_pps)` feedforward term, 100 Hz PID
updates, and a four-sample encoder-speed moving average. Step, reverse, and
6000 pps sweep profiles completed on a full battery without reset or invalid
encoder transitions. Stable platform peak-to-peak speed was 28 to 125 pps.

The reusable inner loop now records one owner mode: `SPEED`, `POSITION`, `YAW`,
or `LINE_TRACKING`. Every target submission includes `now_ms` and renews a
100 ms outer-command lease. The healthy inner loop independently refreshes the
200 ms hardware lease. Ownership loss, a stale outer target, or a control fault
stops both channels and restores high impedance.

## 2026-07-16: initial position-loop cascade firmware

The first formal position outer loop is isolated from PWM and submits bounded
left/right speed targets to the validated speed controller:

```text
outer update: 50 Hz
position gain: Kp 2.4 pps/count, P only
relative test target: +1060/+1060 counts (one wheel revolution)
maximum speed target: 800 pps
speed-loop output cap: 650 permille
position tolerance: 24 counts
settle condition: both speeds <= 120 pps for 200 ms while in tolerance
motion timeout: 8000 ms
```

PB21 starts the relative move and a second press stops it. Completion, timeout,
owner loss, encoder failure, speed-loop failure, or operator stop returns both
motor channels to high impedance. GCC and TIClang builds pass.

### Initial position-loop tuning result

The first `Kp=4`, `1200 pps`, `12-count` run completed but overshot by 123
counts on the left and 145 counts on the right before reversing. Reducing the
outer loop to `Kp=2`, `800 pps` removed overshoot but left the left wheel 22
counts short, where a 44 pps target could not overcome static friction before
the eight-second timeout.

The selected suspended-wheel baseline is:

```text
Kp: 2.4 pps/count
maximum speed: 800 pps
tolerance: 24 counts
settle speed/time: 120 pps / 200 ms
```

At this setting the positive one-revolution test finished at `+21/+10` counts
of remaining error, and the negative test finished at `-21/-13`. Both tests
reached `DONE`, reported zero terminal speed, and restored high impedance.
There was no corrective direction reversal. The 50 Hz outer rate is retained:
the failed cases were bounded by mechanical overshoot and low-speed static
friction, not by the 20 ms position update interval.

After writing these values into the compiled defaults, a cold-start positive
run finished at `+21/+13` counts of remaining error. Three subsequent
alternating runs finished at `-23/+1`, `+21/+13`, and `-20/-10`. Every run
reached `DONE`, ended at `0/0 pps`, reported zero invalid encoder transitions,
and restored `HIGH-Z`. The RAM test target was returned to the compiled
positive `+1060` default after the sequence.

### Position response and endurance regression

The first 8-segment pressure profile exposed long low-speed waits even though
all segments eventually completed:

```text
reverse half revolution:    6.96 s
reverse quarter revolution: 5.08 s
```

The position owner now uses a `4000 pps/s` target slew. A wheel that remains at
or below 40 pps for 300 ms while outside tolerance receives a bounded 800 pps
recovery request, also capped by the configured position maximum. Recovery ends
as soon as motion resumes. The pressure inter-segment high-impedance pause is
100 ms.

With this change, the same two segments completed in 1.95 s and 1.88 s. A final
24-segment run repeated the `+1/-1/+0.5/-0.5/+2/-2/+0.25/-0.25` revolution
sequence three times and produced:

```text
completed moves: 24/24
worst final error: 24 counts
final error: left -21, right -16 counts
recovery count: left 9, right 0
invalid encoder transitions: 0/0
terminal speed: 0/0 pps
terminal state: DONE, HIGH-Z
```

No segment timed out and the board did not reset. The 50 Hz position outer-loop
rate remains appropriate relative to the 100 Hz speed inner loop; the observed
response limit was low-speed static friction rather than the 20 ms outer update.

### Higher-speed position baseline

The user-observed low motion intensity came from the conservative 800 pps cap,
not the position-loop update rate. Online tests raised the cap and position gain:

```text
position Kp: 3.0 pps/count
maximum cascade speed: 2000 pps
speed output limit: 650 permille
tolerance: 24 counts
```

A one-revolution run reached the 2000 pps target, completed in approximately
1.78 s, and stopped with only `3/2` counts of error. The subsequent 24-segment
stress run completed in approximately 27 s with worst error 24 counts, final
error `-23/-15`, recovery count `1/0`, invalid transitions `0/0`, and terminal
`DONE/HIGH-Z`. These values replace the initial 800 pps compiled baseline.

## 2026-07-16: schematic-backed ground-test buttons

The supplied board schematic confirms two additional active-low keys:

```text
SW2 -> PB4 -> -2000 encoder counts
SW1 -> PB5 -> +2000 encoder counts
PB21       -> +4000 encoder counts
```

SW1 and SW2 each have a 200 kOhm external pull-up to 3.3 V and a 100 nF
capacitor to ground. MCU pull-ups and the existing 20 ms software debounce are
also enabled. Each idle press requests one relative dual-wheel position move.
If either the speed or position test is running, any of the three keys requests
an immediate supervised stop instead, so position commands do not accumulate
during motion. The remote tuner and Stress 24 keep their 1060-count base target.
The flashed ground test confirmed that all three commands and supervised stop
behavior work. The vehicle shows a small left/right transient mismatch and
corresponding heading drift while moving. This is consistent with ordinary
motor, gearbox, wheel-diameter, and ground-friction differences combined with
two independent wheel loops; endpoint position success alone does not enforce
matched wheel progress throughout the move. A later straight-line
cross-coupling or heading loop will correct this behavior.

## 2026-07-16: straight-line wheel cross-coupling

The position outer loop now records the encoder count at the start of each
move. For equal nonzero left/right deltas it calculates:

```text
sync error = left progress - right progress
left speed target  -= sync Kp * sync error
right speed target += sync Kp * sync error
default sync Kp: 2.0 pps/count
default correction limit: 400 pps
```

Both corrected targets retain the original travel direction and the configured
position speed limit. Cross-coupling disengages when either wheel enters the
endpoint tolerance, allowing the independent position loops to settle without
reversing a completed wheel. Unequal deltas do not enable synchronization, so
future differential turns remain available. The Bluetooth position command
accepts optional sync gain and limit fields; the previous five-field command
remains compatible. Ground improvement and Stress 24 remain pending after the
new firmware is flashed.

### Ground straight-line result

The user confirmed that the cross-coupled position build produces an almost
perfect straight line in the physical ground test. The default `sync Kp=2.0`
and `400 pps` correction limit are therefore accepted as the initial chassis
baseline. Position-loop tuning is considered complete for straight relative
moves. One synchronized Stress 24 run remains as a regression check before the
position module is frozen for higher-loop integration; it is not expected to
require further tuning unless that regression exposes a fault.

## 2026-07-16: ICM20948 I2C0 bring-up

The plugged-in ICM20948 is now served by a device driver over a dedicated
polling I2C0 layer. SysConfig assigns `PA0` to SDA and `PA1` to SCL. The device
driver probes both legal AD0 addresses, validates identity, configures the
accelerometer and gyro, collects a 400-sample stationary gyro bias, and then
updates at 100 Hz. Motor outputs remain high impedance throughout IMU bring-up.

The flashed board returned:

```text
state: READY (2)
I2C address: 0x69
WHO_AM_I: 0xEA
read errors: 0
sample count: 7736 -> 7906 -> 8082
sample age: 6 ms -> 0 ms -> 0 ms
stationary acceleration: approximately (-0.28, -0.15, +1.01) g
stationary angular rate: within approximately +/-0.15 dps
```

Acceleration magnitude was approximately 1.06 g at the board's resting tilt,
which is physically plausible. Roll/pitch and integrated yaw are exposed for
observation, but no angle controller is allowed to arm the motors yet. The
remaining sensor gate is to rotate the chassis deliberately in both directions
and record the sign of `gz` and yaw before defining vehicle-positive yaw.

### Six-axis attitude estimator and TFT diagnostic page

The original reference project did not contain a quaternion or magnetometer
fusion algorithm. Its good stationary yaw behavior came from startup gyro-bias
calibration, stationary detection, slow online bias tracking, deadbands, and
locking angular integration after 20 stationary samples. Those validated
properties remain in the new driver.

A hardware-independent six-axis estimator now integrates a normalized
quaternion and uses the measured gravity direction for proportional roll/pitch
correction. Accelerometer correction is weighted down as acceleration magnitude
moves away from 1 g. Public yaw accumulates quaternion yaw increments while the
board moves and freezes while stationary, so gravity correction cannot create
stationary heading drift. `imu zero` changes only the public yaw reference.
This is relative inertial heading; the ICM20948 magnetometer is intentionally
not mixed in before chassis magnetic-interference and calibration tests.

The 320 x 170 ST7789 initially presented every IMU diagnostic, but that made
the physical page harder to understand. The final user view contains only
large centered Roll, Pitch, and Yaw values, simple IMU and Yaw state text, and
the motor `HIGH-Z/ARMED` state. Acceleration, gyro, bias, quaternion, sample
age, identity, and errors remain available to engineering tools. Dynamic text
uses an added whole-line DMA formatting path to protect sensor scheduling.

After the optimized page was flashed, five stationary samples over 20 seconds
reported:

```text
sample count: 1175 -> 1626 -> 2076 -> 2527 -> 2982
effective update rate: approximately 90.3 Hz
read errors: 0 throughout
attitude valid / stationary: 1 / 1 throughout
yaw: +0.001 degrees throughout
processed yaw rate: 0 dps throughout
roll range: -8.461 to -8.307 degrees
pitch range: +14.540 to +14.647 degrees
quaternion norm: approximately 1.000
```

The static zero-drift and estimator-validity gates pass. Manual clockwise and
counterclockwise chassis turns remain required to confirm vehicle yaw sign and
dynamic return-to-zero behavior before the angle controller is allowed to own
the speed loop.

### Manual rotation capture

A subsequent hand-rotation capture kept `att=1` for all 45 queried samples.
Measured body-Z and quaternion-derived yaw rates followed the same sign and
similar magnitude:

```text
raw GZ range:              -59.565 to +70.896 dps
quaternion yaw-rate range: -57.912 to +67.848 dps
one reverse return:        -72.686 to -2.882 degrees
I2C read errors:           0
invalid attitude samples:  0
```

After the chassis stopped at approximately -89 degrees, four samples over nine
seconds all reported exactly `yaw=-89.104 degrees`, `yaw_rate=0`, `still=1`,
and `att=1`. This verifies that the stationary yaw lock engages again after a
dynamic turn rather than only at startup. The motion sign is internally
consistent; the final vehicle convention still needs the operator to label the
observed physical clockwise direction before the yaw controller is enabled.

### Yaw loop bring-up and bidirectional stress test

The operator confirmed the vehicle convention: clockwise rotation is negative
Yaw and counterclockwise rotation is positive Yaw. The Yaw controller owns the
validated wheel-speed loop and always returns both motor channels to high
impedance on completion, timeout, operator stop, ownership loss, or IMU error.

Initial `500 pps` tests were mechanically weak and repeatedly stopped just
outside the angle tolerance. Telemetry showed that the wheel-speed loop was
commanding only `23..68 pps` near the target and could not overcome chassis
stiction. The controller now preserves its integral while temporarily inside
the settling band and applies a configurable minimum turn speed only when its
output still points toward an out-of-tolerance target. Opposite-direction PID
braking is not clamped by this compensation.

The first data-only candidate was:

```text
Kp / Ki / Kd:       30.0 / 1.5 / 1.5
maximum turn speed: 1000 pps
minimum turn speed:  300 pps
PWM limit:            650 permille
angle tolerance:        1.5 degrees
settle rate:             4.0 dps
settle time:            300 ms
timeout:               6000 ms
```

A single clockwise run reached `-43.853 degrees` with a `-1.147 degree`
target error, approximately `70.5 dps` peak rate, no target crossing, and a
`DONE / HIGH-Z` terminal state. A single counterclockwise run reached
`+43.521 degrees` with a `+1.479 degree` error, approximately `84.0 dps` peak
rate, no target crossing, and `DONE / HIGH-Z`.

Four additional alternating `-45/+45/-45/+45` runs all completed with result
zero and high impedance. Their final angles were `-43.533`, `+43.507`,
`-43.959`, and `+43.500` degrees. This passed the protocol and safety gates but
failed physical acceptance: the operator reported weak turning, stop-and-go
motion, and unacceptable errors clustered at the `1.5 degree` completion
boundary. The candidate was not promoted to compiled defaults.

The next revision therefore raises the Yaw-owner speed-target slew rate from
`2000` to `6000 pps/s`, adds a configurable Yaw-only feedforward boost for the
extra tire-scrub load of pivot turns, and restricts minimum-speed compensation
to near-stationary recovery. Accuracy must be retested with a tighter tolerance
before any final defaults are accepted.

### Final ground-tuned Yaw loop

Ground testing exposed a second stiction case after the initial feedforward
boost had already disengaged. During deceleration, both wheels could stop near
`320..370 pps` while the speed loop still applied approximately `520
permille`; the Yaw test then timed out several degrees short. The speed loop
now re-enables its per-wheel startup assist only after the requested speed is
inside the configured feedforward ramp and measured speed falls below `100
pps`. This leaves the verified high-speed behavior unchanged while restoring
authority through the final low-speed approach.

The promoted defaults are:

```text
Kp / Ki / Kd:       45.0 / 0.8 / 7.0
maximum turn speed:  800 pps
minimum turn speed:  200 pps
feedforward boost:    40 permille
PWM limit:            750 permille
angle tolerance:        0.7 degrees
settle rate:             5.0 dps
settle time:            300 ms
timeout:               5000 ms
```

The post-fix matrix passed `+/-2`, `+/-5`, `+/-10`, `+/-30`, `+/-45`,
`+/-90`, and `+/-135` degrees. A further ten-run alternating `+45/-45`
ground stress test completed every commanded run with result zero and high
impedance. High-rate telemetry measured `1.5..1.8 s` to enter the `1 degree / 5
dps` settling region. Nine runs had no target crossing; one crossed by only
`0.008 degrees`. Successful terminal errors ranged from `0.23` to `0.94
degrees`. A deliberate host CSV contention failure stopped through the normal
supervisor path with high impedance and was excluded from the ten successful
runs.

After a wireless update and MCU restart, `yaw get` returned the promoted
defaults without a host-side parameter write. A bare `yaw run` then completed
the default `-45 degree` command at `-44.214 degrees`, result zero, and high
impedance. The GUI remains the sole owner of high-rate latest-wave files when
the CLI controls it through the TCP bridge, eliminating concurrent CSV writes.

### Physical Yaw buttons

The three ground-test buttons now issue relative Yaw commands instead of
position-count moves:

```text
PB21:    +45 degrees
SW1/PB5: +90 degrees
SW2/PB4: -60 degrees
```

`YawBringupTest_RequestTurn()` owns the one-shot target update and start
request. Pressing any button while speed, position, or Yaw testing is active
retains the existing immediate supervised-stop behavior. Position commands
remain available through their public API and Bluetooth console.

### Yaw scheduling jitter measurement

Runtime instrumentation now records the main-loop interval, ICM20948 sample
interval, Yaw-control interval, and synchronous LCD update duration. The Yaw
status frame, GUI JSON snapshot, and CLI capture CSV expose both the latest and
maximum values without changing the seven-channel VOFA+ waveform format.

An initial `+90 degree` run with the promoted Yaw defaults measured a `55 ms`
maximum LCD update, `56 ms` main-loop interval, `61 ms` IMU interval, and `62
ms` Yaw-control interval. This confirmed that whole-dashboard synchronous
redraws periodically reduced the nominal `100 Hz` control path to roughly `16
Hz`.

Static labels are now drawn once and the dynamic dashboard is refreshed in
four slices at `50 ms` intervals. The complete screen still updates in about
`200 ms`, while the matched `+90 degree` run reduced the maximum LCD update to
`17 ms`, main-loop interval to `19 ms`, and both IMU and Yaw-control intervals
to `21 ms`. The run completed at `89.009 degrees` with result zero and high
impedance. Ground testing confirmed a substantial improvement in smoothness,
so a full asynchronous LCD state machine is not currently justified.

## 2026-07-17: continuous Heading control

`WheelHeadingControl` adds a continuous outer loop for moving straight while
holding a Yaw target. It owns the speed loop through the dedicated `HEADING`
supervisor mode and mixes a base-speed command with equal-and-opposite Yaw
correction. A separate 100 ms command lease prevents a stalled application
from leaving the vehicle moving. The existing pivot Yaw controller and its
ground-tuned parameters are unchanged.

The first ground run used `Kp=15`, `Ki=0`, `Kd=1.5`, `1000 pps` base speed,
and a `300 pps` correction limit for `2.5 s`. It completed safely but was too
short for useful path assessment. Telemetry reached `-5.017 degrees`; the
correction direction was correct but peaked at only about `70 pps`, confirming
insufficient authority.

The accepted run used:

```text
Kp / Ki / Kd:        30.0 / 3.0 / 1.5
base speed:          1200 pps
maximum correction:  400 pps
deadband:               0.5 degrees
PWM limit:             650 permille
duration:             6000 ms
```

Across the longer run, heading reached a maximum of `+2.386 degrees`, crossed
the target once, reached `-1.098 degrees` on the opposite side, and finished
near `-0.87 degrees`. Correction remained between approximately `-60` and
`+22 pps`; timing maxima remained `20 ms` for the main loop, `26 ms` for IMU
and Heading updates, and `17 ms` for an LCD slice. The run ended with result
zero and high impedance. The operator confirmed the ground path was acceptable
and did not show an exaggerated S shape.

## 2026-07-17: eight-channel line-sensor bring-up

The board schematic maps the line-sensor connector to `PA17/SCL` and
`PA16/SDA`. A polling I2C1 MCU driver and a separate external-device driver
were added without changing the validated motor, speed, position, Yaw, or
Heading loops. The module responds at 7-bit address `0x12`; firmware writes
control register `0x01`, reads data register `0x30`, and samples every `20 ms`.
The eight active-low channel weights are:

```text
-35, -25, -15, -5, +5, +15, +25, +35
```

Physical static tests produced:

```text
center:  raw=0xE7 mask=0x18 count=2 error=  0 seen=1
left:    raw=0x3F mask=0xC0 count=2 error=-30 seen=1
right:   raw=0xFC mask=0x03 count=2 error=+30 seen=1
no line: raw=0xFF mask=0x00 count=0          seen=0
```

The center reading repeated identically across five queries. After more than
23,000 samples, both the device read-error count and I2C bus-recovery count
remained zero. All checks were performed with the motor outputs in high
impedance. The retained `line_error` during `seen=0` records the last observed
side and is not a valid current-line measurement; future control must always
gate it with `line_seen`.

## 2026-07-17: supervised line-tracking outer-loop bring-up

`WheelLineTrackingControl` now owns the validated wheel-speed loop through the
dedicated `LINE_TRACKING` supervisor mode. The line sensor and outer loop both
run at `100 Hz`. A `100 ms` command lease, a `60 ms` observation-age limit,
board-button stop requests, and every non-corner line loss retain the immediate
supervised high-impedance stop path. The reusable controller remains separate
from the temporary timed bring-up workflow and tuner protocol.

The accepted ground-test configuration was:

```text
Kp / Ki / Kd:       30.0 / 0.0 / 0.0
base speed:          300 pps
maximum correction:  900 pps
deadband:               2 line-error units
PWM limit:             750 permille
duration:            10000 ms
```

The taped route's acute corner produces a wide sensor pattern. When at least
four channels are active, the controller preserves the last unambiguous turn
direction, reduces base speed to `250 pps`, and applies at least `350 pps` of
differential correction. This prevents the earlier failure in which the car
continued almost straight across the corner. If the line temporarily leaves
the array during an already confirmed and direction-locked corner, the
controller removes the forward base-speed component and searches with a
`+600/-600 pps` in-place pivot for at most `1800 ms`. Reacquisition ends that
blind search immediately; expiration still stops in high impedance. This
exception does not apply to ordinary line loss or to a wide pattern with no
known turn direction. Corner lock exits after the line remains within
`count<=3` and `|error|<=5` for `600 ms`; these limits match the quantization
of the eight digital channels without overlapping the `count>=4` wide-line
entry condition.

The first locked-turn run stopped between the corner and its following straight
because immediate line-loss handling removed motor torque. A `700 ms / 350
pps` reacquisition attempt then turned farther but still stopped short. In the
next `+750/-250 pps`, `1200 ms` version passed once but failed a later stress
run at the same halfway location. Its nonzero forward component and weak
right-wheel reversal did not produce enough rotation consistently.

The promoted search instead uses the in-place pivot above. On its first ground
run the outgoing line was reacquired after approximately `350 ms` at the
opposite edge of the array. Error then converged through `+35, +30, +25, +20,
+15, +10, +5, 0`, the car continued along the following straight, and the
operator physically confirmed the completed corner transition. Five subsequent
`10 s` runs all ended with result zero and high impedance. Together they
covered five wide-line episodes and two complete line-loss/reacquisition
episodes. No I2C errors occurred; measured maxima were `27 ms` for line-control
updates, `19 ms` for the main loop, and `17 ms` for an LCD slice. This validates
the tested route at `300 pps`; higher speeds and additional line geometries
remain separate ground-test coverage.

### Five-corner route high-speed baseline

The fixed closed route contains one right-angle corner, three obtuse corners,
one acute corner, and approximately 30-45 cm straight sections. Higher-speed
testing retained the accepted `Kp=30`, `Ki=0`, `Kd=0`, `900 pps` correction
limit, and `750 permille` speed-output limit while raising the requested base
speed to `1200 pps`.

Corner handling now commits the selected turn direction before blind search,
ramps lost-line pivot correction from `350` to `600 pps` over `200 ms`, and
accelerates the recovered base-speed command at `1200 pps/s`. A centered
corner exit no longer waits at a weak `300/300 pps` command: it drives through
with a `500 pps` base and `150 pps` residual turn correction for up to `350
ms`, allowing the chassis to finish aligning with the outgoing straight. The
operator reported that the resulting motion was basically smooth.

Timed tests now treat their configured duration as a minimum. After that
deadline, tracking continues until `count<=2` and `|error|<=5` remain stable
for `250 ms`; a `3000 ms` grace deadline still forces high impedance. This
prevents a normal timed test from stopping in the middle of a corner. The
bring-up protocol accepts runs up to `60000 ms`.

Three consecutive `30 s` runs completed with result zero and high impedance.
A subsequent `60.27 s` continuous run covered 19 wide-line entries and five
complete lost-line recoveries. The longest recovery was `937 ms`, below the
`1800 ms` safety limit. Average effective base speed was `705 pps` in the
first half and `691 pps` in the second half, showing no meaningful late-run
degradation. I2C errors remained zero, the maximum line-control interval was
`27 ms`, and the run stopped on `count=2 / error=0` with motor outputs in high
impedance. This establishes `1200 pps` as the accepted route baseline; higher
requested speeds remain experimental.

### 1400 pps route baseline and corner-exit recovery

The same five-corner route was subsequently accepted at a requested `1400
pps` with the existing `Kp=30`, `Ki=0`, `Kd=0`, `900 pps` correction limit,
`750 permille` output limit, and `2`-unit deadband. Fixed increases in
differential correction were rejected because they caused repeated spinning
at the acute corner. The promoted controller instead uses IMU Yaw-rate
feedback only while the line remains visible: it maps the original correction
to a `0.075 dps/pps` target, adds at most `180 pps` when measured rotation is
too slow, and falls back to the validated line-only behavior whenever the IMU
is invalid or stale. The seven-channel VOFA+ line waveform remains unchanged;
the target rate, measured rate, boost, and IMU-valid state are exposed through
`LSTAT` and tuner JSON/CSV telemetry.

Visible wide-line corners now retain a `420 pps` forward base. A corner that
actually lost the line keeps the validated `600 pps` blind pivot, then exits
at `800 pps` base with `220 pps` residual correction for the existing `350 ms`
stability window. Corners without line loss use a `1050 pps` base, `240 pps`
residual correction, and a `200 ms` window. After direction lock releases, a
separate `9000 pps/s` recovery ramp returns to normal route speed in roughly
`70 ms`; ordinary startup retains the gentler `3000 pps/s` ramp.

Two consecutive `30 s` runs of the promoted version completed with result
zero and high impedance, including acute-corner line-loss recovery. I2C errors
remained zero; measured maxima were `28 ms` for the line-control interval, `20
ms` for the main loop, and `18 ms` for an LCD slice. The operator accepted the
result as usable but noted that overall corner-to-straight flow is not yet
fully smooth. This is therefore the accepted `1400 pps` route baseline, not a
claim that line-tracking dynamics are final.

### Cold-start promotion and strict finish result

The accepted route configuration is now compiled into both the reusable line
controller and the bring-up profile:

```text
Kp / Ki / Kd:        30.0 / 0.0 / 0.0
requested base:       1400 pps
maximum correction:    900 pps
PWM limit:              750 permille
deadband:                 2 line-error units
minimum duration:     30000 ms
```

After a wireless update and MCU restart, a bare `line get` returned these
values without a host parameter write. A subsequent `line run` issued no
`line set`, completed the full route, and stopped at `count=1 / error=-5` with
result zero, zero I2C errors, zeroed wheel/base targets, and high impedance.
Measured maxima were `26 ms` for line control and `20 ms` for the main loop.

The timed bring-up workflow now reports `DONE` only after a centered line is
stable for `250 ms`. Failure to reach that condition during the three-second
finish grace reports `ABORT / COMMAND_TIMEOUT / HIGH-Z` instead of a false
success. The finish policy has host regression coverage for stable completion,
center-loss reset, grace timeout, and 32-bit millisecond wraparound. The
operator judged the promoted cold-start behavior materially unchanged and
usable, while still somewhat slow, and chose to end line-speed tuning at this
baseline.

## 2026-07-17: hardware watchdog and wireless-update compatibility

WWDT0 now runs from LFCLK with divider 2, a 15-bit period, and no closed
window, producing a 2.00 second timeout. It continues through sleep, pauses
while the CPU is halted by a debugger, and is refreshed only at the end of a
complete main-loop scheduling pass immediately before `WFI`. The tuner exposes
read-only status as `wdt stat`; the deliberate `wdt test` fault injection is
rejected unless every motor output is already high impedance.

The supervised fault-injection test started from:

```text
WSTAT active=1 kicks=108969 hz=1
```

After `wdt test`, refreshes stopped and WWDT0 reset the MCU approximately two
seconds later. The application restarted with `LineStatus=LOCKED`, motor
outputs in high impedance, and a growing watchdog refresh count. This verifies
both watchdog reset behavior and the existing suspicious-reset motion lockout.

The first watchdog-enabled wireless update exposed an MSPM0 reset-domain
interaction: WWDT0 remained active across the software reset and reset the
resident Bootloader during Flash erase/program, which returned Bootloader
status 4 (`BOOT_STATUS_BAD_STATE`) and left the application region erased.
The resident Bootloader itself remained intact and forced the motor pins to
high impedance.

`FirmwareUpdate_Task` now waits for UART TX idle, then resets and powers down
WWDT0 before writing the update mailbox and requesting the software reset. A
direct Bootloader recovery successfully programmed the 110000-byte image with
CRC32 `0x21A9E0ED` in 28.9 seconds. A subsequent normal application-initiated
wireless update of the same image completed in 29.2 seconds with no Bootloader
error. After restart, the firmware reported `READY`, zero motor targets and
outputs, high impedance, and `WSTAT active=1`, with the refresh count increasing
normally. This validates both fault recovery and the normal JDY-31 update path
while the hardware watchdog feature is enabled.

## 2026-07-17: formal application interaction state machine

The physical-button policy previously lived as a priority chain inside
`main.c`. It is now isolated in hardware-independent `car_app.c` with explicit
`LOCKED`, `READY`, `SERVICE`, and `MOTION_ACTIVE` states. The state machine
selects one active workflow deterministically, blocks new button motion during
reset lockout or JDY-31/update service, and converts any button press during an
active workflow into `STOP_ACTIVE` before a different command can start. The
existing ready-state mapping remains PB21 `+45 degrees`, PB4 `-60 degrees`, and
PB5 `+90 degrees`; none of the validated control-loop implementations or
tunings changed.

Host coverage verifies all three Yaw mappings, deterministic active-workflow
priority, service and suspicious-reset lockout, stop priority during service,
and state-transition counting. Both MCU toolchains build successfully and the
two host tests pass. The 111072-byte GCC image with CRC32 `0xAAC271C9` was
wirelessly programmed in 31.0 seconds. After restart, the board reported:

```text
ASTAT state=READY workflow=0 action=0 yaw=0 transitions=0 hz=1
LSTAT state=READY ... tL=0 tR=0 vL=0 vR=0 outL=0 outR=0 hz=1
WSTAT active=1 kicks=24939 hz=1
```

The line sensor also restarted with zero read or bus-recovery errors. This
validates cold-start state reporting and motor-safe idle behavior.

The subsequent ground regression exercised the `+90`, `-60`, and `+45 degree`
button commands, followed by repeated PB21 `+45 degree` starts interrupted by
PB4 during motion. The final observed stop occurred near `+15.9 degrees` and
reported result 9 (`WHEEL_YAW_CONTROL_STOPPED`) with zero targets, zero PWM
outputs, and high impedance. No `-60 degree` target followed any PB4 stop, so
the second button press stopped the active workflow without queuing a new
turn. The operator confirmed that the physical stop was immediate and normal.

## 2026-07-18: formal continuous line-following mission

The accepted `1400 pps` line-tracking baseline is now exposed through a
separate product mission instead of reusing the timed bring-up workflow.
`mission start` restores the accepted `Kp=30`, `Ki=0`, `Kd=0`, `900 pps`
maximum correction, `750 permille` output limit, and `2`-unit deadband before
starting. The mission continuously refreshes the existing 100 ms line-command
lease and has no normal time limit. `mission stop`, any application-level
button stop, sensor or command expiry, and controller faults all return the
drive to high impedance. The original `line run`, `LSTAT`, and seven-channel
`linewave` bring-up interfaces remain separate and unchanged.

Both MCU toolchains built successfully and all four host tests passed,
including the new mission lifecycle and application button-stop routing
coverage. The 114072-byte GCC image with CRC32 `0xE032F4F9` was wirelessly
programmed in 21.4 seconds. Cold-start status was:

```text
ASTAT state=READY workflow=0 action=0 yaw=0 transitions=0 hz=1
MSTAT state=READY runs=0 base=1400 limit=750 elapsed=0 ... control=0 hz=1
LSTAT state=READY sensor=READY ... errors=0 busRec=0 hz=1
WSTAT active=1 ... hz=1
```

The formal mission then ran forward on the fixed five-corner closed route for
49.27 seconds and produced 982 telemetry samples. The operator confirmed the
ground behavior was normal. Telemetry contained 14 wide-line entries and nine
temporary line-loss events; all were reacquired without a mission fault, with
the longest continuous loss approximately 1.05 seconds. The maximum observed
line-control interval was 27 ms, the main-loop maximum was 21 ms, the LCD slice
maximum was 18 ms, and I2C read/recovery errors remained zero. The run ended
through explicit operator stop with `MSTAT state=STOP`, result 9, controller
inactive, and motor outputs in high impedance. This promotes `1400 pps` as the
formal mission baseline; higher requested speeds remain experimental.

## 2026-07-18: composite distance and Heading motion owner

The first shared upper-layer motion interface is now implemented in
`app/motion/motion_supervisor` with `control/wheel_odometry`. It owns the
dedicated `MOTION` speed mode and combines a bounded average encoder-distance
correction with a bounded Heading correction. The caller can provide an
absolute target Heading or request a hold of the Heading captured at motion
start. Existing Position, Heading, Yaw, and line owners are rejected while
this owner is active, preventing accidental parallel outer loops.

The GCC image was rebuilt at 110872 bytes with CRC32 `0x15638787` and updated
over JDY-31 in 24.4 seconds. After restart, `OSTAT state=READY` and all motor
outputs were high impedance. Host coverage reached five passing tests,
including odometry deltas and application button-stop routing.

The first visible-distance trial used `+530` counts, `700 pps`, and a `4000 ms`
timeout. It correctly returned `ABORT / TIMEOUT` at 26 counts of remaining
error, demonstrating the timeout guard rather than falsely reporting success.
The following supervised trials all completed with high impedance:

```text
delta  heading  max pps  timeout  elapsed  final error  yaw error  result
+530   hold     700      8000     6606 ms  +23          +0.084 deg  DONE
-530   hold     350      10000    5345 ms  -24          +0.262 deg  DONE
+1060  hold     500      10000    3835 ms  +23          -0.920 deg  DONE
+2120  hold     220      15000   11794 ms  +22          +0.316 deg  DONE
```

The operator confirmed the vehicle showed no obvious deflection and remained
straight during the long, low-speed run. This promotes the composite owner as
the reusable interface for future distance, route, and competition tasks;
those tasks still need their own behavior-level tests.

## 2026-07-18: production and bring-up build profiles

The firmware now has explicit debug and product profiles controlled by
`CAR_ENABLE_BRINGUP`. The debug profile retains the validated speed, position,
Heading, and timed line workflows. The product profile replaces those temporary
state machines with disabled entry points while retaining the physical-button
Yaw workflow, line-sensor service, formal line mission, Motion owner, safety,
diagnostics, and wireless update paths.

Both GCC and TIClang debug/product targets built successfully, and all five
host tests passed. The product GCC image is 99776 bytes compared with 110872
bytes for the debug image. The product image was wirelessly programmed with
CRC32 `0x7FF003B7`; `ASTAT READY`, `MSTAT READY`, and `OSTAT READY` all
reported `hz=1`. Product `pos run` was rejected without arming the motors.

This accepts Stage 1 of `OPTIMIZATION_PLAN.md`. Stage 2 now moves scheduler
ordering out of `app/main.c` without changing task order or control behavior.

## 2026-07-18: scheduler boundary reduction

The application scheduler was moved from `app/main.c` into
`app/runtime/car_runtime.c`. The startup wrapper now contains only runtime
initialization and the perpetual step call; task ordering, button action
routing, display slicing, watchdog service, and low-power sleep are unchanged.

Both GCC and TIClang debug/product targets and all five host tests passed. The
updated GCC image was wirelessly programmed through JDY-31 and restarted in
`READY / HIGH-Z`. The operator accepted a supervised `+530`-count,
Heading-hold motion at `700 pps`; it completed normally (`runs=3`, final
position error `23` counts, result `0`) and the final status reported
`hz=1`. Previously validated active-workflow button-stop coverage remains the
regression evidence for the unchanged stop path.

This accepts Stage 2 of `OPTIMIZATION_PLAN.md`. Stage 3 is the next bounded
change: isolate tuning and diagnostics without changing command text, VOFA+
channels, or validated control-loop behavior.

## 2026-07-18: tuning command routing isolation

The UART tuning console now contains only line reception, overflow handling,
the ready banner, and waveform scheduling. Existing command parsing and
workflow dispatch moved to `app/tuning/tuning_command_router.c` with a small
public entry point. No command grammar, parameter range, response prefix, or
VOFA+ channel changed.

GCC and TIClang debug/product targets and all five host tests passed. The
updated GCC image was wirelessly programmed through JDY-31 and restarted in
`READY / HIGH-Z`. Read-only regression through the TCP bridge returned normal
speed, position, IMU, Yaw, Heading, line, mission, motion, and watchdog
responses; the operator confirmed the tuner panel and waveform display had no
visible anomaly.

This sub-step is accepted. Stage 3 remains in progress; the next bounded
change is splitting status emission by domain while preserving response text.

## 2026-07-18: tuning status-domain isolation

The status implementation was split by ownership into
`tuning_control_status.c`, `tuning_sensor_status.c`, and
`tuning_mission_status.c`. The existing `tuning_status.h` declarations remain
the stable facade used by the command router, so status prefixes, fields, and
VOFA+ waveform channels are unchanged.

GCC and TIClang debug/product targets and all five host tests passed. The
updated GCC image was wirelessly programmed through JDY-31 and restarted in
`READY / HIGH-Z`. Sequential TCP-bridge reads of speed, position, IMU, Yaw,
Heading, line, mission, motion, and watchdog status all returned normally; the
operator confirmed no tuner-panel anomaly.

This sub-step is accepted. Stage 3 remains in progress for the final
product-profile dependency audit.

## 2026-07-18: tuning and diagnostics isolation complete

The product-profile dependency audit confirms that temporary speed, position,
Heading, and timed-line implementations are not linked into the product image;
`bringup_disabled.c` supplies only the required safe API stubs. Symbol-size
comparison shows the product task stubs are 16-22 bytes, while the debug
workflow tasks are 452-644 bytes. The product GCC image is approximately
104240 bytes versus 115544 bytes for the debug image.

This completes Stage 3 of `OPTIMIZATION_PLAN.md`. The next stage is deliberately
conditional: add only a concrete competition route or motion composition, and
keep the existing Line mission, Motion owner, stop paths, and standalone loops
as the exclusive owners of their behavior.

## 2026-07-18: chassis control baseline accepted

The operator completed acceptance of the reusable chassis layer. The validated
baseline includes the wheel speed loop, wheel position loop with encoder
synchronization, relative Yaw control, continuous Heading hold, the PA16/PA17
eight-channel line-sensor path, and the supervised five-corner line mission at
the accepted `1400 pps` command speed. Wireless tuning through JDY-31, command
leases, button-priority stops, automatic timeout handling, and motor `HIGH-Z`
return are part of the accepted interface.

The standalone control APIs and product Mission owners are now considered the
stable chassis boundary for competition work. Future competition firmware
should add only the problem-specific upper workflow and its tests; it should call
these owners rather than retune or duplicate their inner loops. Higher line
speeds remain experimental and are not part of this acceptance.

## 2026-07-19: Gate 0 UART2 migration and wireless update regression

JDY-31 ownership moved from UART3 PA25/PA26 to UART2 PA21/PA22 in both the
application and resident Bootloader. GCC and TIClang application and
Bootloader targets build with global `-Os`; the final GCC application image is
75,216 bytes and the Bootloader contains 4,688 bytes of code/data.

The one-time J-Link installation completed with program verification. UART2
testing exposed JDY-31 uplink synchronization loss on the first bytes of a TX
burst. The Bootloader now uses a guarded synchronization preamble, and the
application uses end-of-transmission-paced bytes with a short training line.
The host updater tolerates bounded preamble noise, while the tuner drops the
single-byte training line before TCP forwarding. Firmware-update pending state
also suppresses waveform scheduling so the TX queue can drain before reset.

The final image completed two consecutive COM6 wireless updates in 13.8 and
13.9 seconds with CRC32 `0x34210D15`. After each restart the tuner TCP bridge
reported `ASTAT state=READY` and `hz=1`; port 13470 delivered the expected
FireWater wave stream, and the bridge retained seven-channel Yaw mode. This
completes Gate 0. Gate 1 starts the read-only K230 UART3 parser and transport.
