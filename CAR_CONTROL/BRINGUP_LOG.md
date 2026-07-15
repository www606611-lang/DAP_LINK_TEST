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

## Manual checks still required

1. Press and release PB21 repeatedly. Confirm the counter changes and uptime
   does not restart.
2. Confirm the encoder supply voltage and whether A/B outputs are open-drain or
   push-pull before any powered motor test.

No PWM or motor-output peripheral is enabled at this milestone.
