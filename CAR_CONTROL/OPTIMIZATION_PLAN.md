# CAR_CONTROL Optimization Plan

This document is the execution checklist for turning the validated firmware
prototype into a competition-ready application. Work is done in order. One
stage is changed and tested at a time; the next stage does not start until the
operator accepts the current stage on the vehicle.

## Baseline

- Branch: `checkpoint/position-cascade-20260519`
- Baseline commit: `14a109b2a73a57114ff7fbcdfda60c2533c71ce9`
- GCC image last flashed: 110872 bytes, CRC32 `0x15638787`
- Wireless update: JDY-31 on `COM6`
- Tuner owns `COM6`; CLI and VOFA+ use the tuner TCP bridge
- Stable line mission: five-corner route at requested `1400 pps`
- Conservative line baseline: five-corner route at `1200 pps`
- Motor safety: every motion path has a lease, supervised stop, and `HIGH-Z`
  stop state

The validated speed, position, Yaw, Heading, and line inner/outer loops are
not retuned or rewritten as part of structural work. A line corner change is
experimental until the operator accepts a full five-corner run.

## Rules For Every Stage

1. Inspect the current tree and preserve unrelated dirty files.
2. Make one bounded change with an explicit non-goal.
3. Run the relevant host tests.
4. Build both firmware targets:

   ```powershell
   cmake --build build-gcc --target car_control -j
   cmake --build build-ticlang --target car_control -j
   ```

5. Flash the GCC image through the JDY-31 updater only after the tuner reports
   `HIGH-Z`.
6. Restart the tuner hidden and confirm `READY / HIGH-Z`.
7. State the physical test direction, speed, duration, stop behavior, and
   required free space before a new ground-motion profile.
8. Record telemetry and the operator's physical result.
9. If accepted, update `BRINGUP_LOG.md`, commit only the accepted CAR_CONTROL
   files, push, and verify local and remote hashes match.
10. If rejected or inconclusive, restore the previous baseline before the next
    experiment. Do not commit experimental firmware.

## Ordered Stages

### Stage 0 - Baseline Lock (current)

Status: complete.

Confirm the baseline image, branch hash, wireless updater, tuner bridge,
`HIGH-Z` state, five host tests, and both toolchain builds. This is the
rollback point for every later stage.

### Stage 1 - Unified Mission Controller

Status: next.

Add a product-level mission scheduler under `app/mission/`. It should sequence
safe actions such as:

- relative distance with Heading hold;
- relative Yaw pivot;
- line-follow mission;
- stop, timeout, and fault termination.

The scheduler submits commands to existing owners. It must never call motor
PWM or start two outer loops concurrently. `MotionSupervisor` remains the
distance plus Heading implementation; the new layer owns sequencing and
mission state only.

Acceptance:

- host tests cover idle, one action, sequential actions, conflict rejection,
  timeout, operator stop, and fault stop;
- the current standalone speed, position, Yaw, Heading, and line commands
  behave exactly as before;
- a short supervised vehicle test completes one distance-plus-Heading action,
  ends in `DONE / HIGH-Z`, and is stopped by a board button during a second
  action.

Non-goals: no PID changes, no line-speed changes, no new route tuning.

### Stage 2 - Production Versus Bring-Up Build Profiles

Status: pending Stage 1 acceptance.

Add a CMake option such as `CAR_ENABLE_BRINGUP` and separate the product
mission build from the temporary `app/bringup` workflows. Keep bring-up code
available in the debug/tuning profile, but keep the competition image focused
on product missions, safety, diagnostics, and the required command protocol.

Acceptance:

- both profiles build with GCC and TIClang;
- the product profile cannot start a bring-up test accidentally;
- the tuning profile retains all currently validated test commands;
- both profiles preserve startup `HIGH-Z` and suspicious-reset lockout.

Non-goals: do not delete `experiments/legacy_motor_bringup` or alter reference
projects.

### Stage 3 - Mission and Tuning Tool Closure

Status: pending Stage 2 acceptance.

Add a Motion/Mission view to the tuner for distance, Heading, Yaw, segment
selection, state, progress, timeout, result, and `HIGH-Z`. Keep the existing
seven-channel VOFA+ groups unchanged. The CLI remains the automation path and
the GUI remains the single `COM6` owner.

Acceptance:

- every mission command available through CLI is visible in the GUI or has a
  documented CLI-only reason;
- latest mission state and telemetry are written to JSON/CSV;
- no foreground mouse automation is required;
- the existing speed, position, Yaw, Heading, and line panels regress-free.

### Stage 4 - Safety and Fault Regression

Status: pending Stage 3 acceptance.

Expand host and supervised hardware coverage for sensor-stale, command-lease
expiry, output failure, ownership loss, watchdog reset, wireless-update entry,
and button stop. Every path must prove both target commands are zero and the
motor pins are high impedance.

Acceptance:

- all five host tests pass;
- each injected fault reports a stable result and cannot restart motion;
- watchdog test boots into suspicious-reset lockout;
- wireless update still completes from idle without a J-Link.

### Stage 5 - Product Configuration and Calibration

Status: pending Stage 4 acceptance.

Centralize board/product defaults in a configuration header or module: wheel
mapping, counts per revolution, speed limits, output limits, IMU scale and
bias policy, and mission defaults. Keep tuning experiments in RAM until the
operator accepts a result; only accepted values become compiled defaults.

Acceptance:

- one documented source controls each product default;
- cold-start status and sensor calibration remain unchanged;
- full-battery and normal-battery checks do not change sign conventions or
  safety behavior;
- the configuration change has no control-loop behavior change by itself.

### Stage 6 - Optional Line-Tracking Optimization

Status: optional and last among control changes.

The `1400 pps` route is the accepted baseline, but corner-to-straight flow is
not claimed final. Only the line outer loop may change. Test one parameter or
one small state-machine change at a time, always covering the right angle,
three obtuse corners, and the acute corner.

Acceptance:

- no acute-corner spin or straight-through failure;
- no unexpected stop or line-loss safety fault;
- operator reports a clear improvement in corner flow;
- telemetry is no worse than the baseline for line error, recovery, and speed
  tracking;
- otherwise revert immediately.

## Completion Definition

The project is considered competition-ready when Stages 1 through 5 are
accepted, the product profile is the image used for the competition, every
mission has a tested stop/fault path, and the final branch and remote hashes
match. Stage 6 is an optimization opportunity, not a reason to destabilize the
validated baseline.

## Current Action

Implement Stage 1 only. Do not change the validated inner loops or the current
`1400 pps` line parameters while building the mission scheduler.
