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

## Manual checks still required

1. Press and release PB21 repeatedly. Confirm the counter changes and uptime
   does not restart.
2. Complete the supervised dual-motor bench procedure above.
3. Confirm the encoder supply voltage and whether A/B outputs are open-drain or
   push-pull before any powered motor test.

Both motor-output peripherals remain disabled and high impedance until the
supervised PB21 test is started.
