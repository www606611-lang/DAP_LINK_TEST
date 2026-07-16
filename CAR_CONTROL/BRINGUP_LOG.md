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
