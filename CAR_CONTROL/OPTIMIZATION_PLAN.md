# CAR_CONTROL Optimization Plan

This document is the execution checklist for turning the validated firmware
prototype into a competition-ready application. Work is done in order. One
stage is changed and tested at a time; the next stage does not start until the
operator accepts the current stage on the vehicle.

## Baseline

- Branch: `checkpoint/position-cascade-20260519`
- Baseline commit: `14a109b2a73a57114ff7fbcdfda60c2533c71ce9`
- Validated rollback GCC image: 110872 bytes, CRC32 `0x15638787`
- Planning-document commit: `8b382a5a2b1691d384c612c029d1a1fd89a23f5d`
- Wireless update: JDY-31 on `COM6`
- Tuner owns `COM6`; CLI and VOFA+ use the tuner TCP bridge
- Stable line mission: five-corner route at requested `1400 pps`
- Conservative line baseline: five-corner route at `1200 pps`
- Motor safety: every motion path has a lease, supervised stop, and `HIGH-Z`
  stop state

The validated speed, position, Yaw, Heading, and line inner/outer loops are
not retuned or rewritten as part of structural work. A line corner change is
experimental until the operator accepts a full five-corner run.

The oversized mission-controller experiment was removed before commit. The
validated `MotionSupervisor` remains the only distance plus Heading composite
owner until a concrete product route justifies a smaller mission workflow.

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

### Stage 1 - Architecture Cleanup And Build Boundary

Status: accepted by operator.

First separate code by responsibility without changing motion behavior. The
production image must not compile every temporary `app/bringup` workflow by
default. Add a CMake option such as `CAR_ENABLE_BRINGUP`:

- product profile: product missions, safety, required diagnostics, and the
  wireless command service;
- bring-up profile: the existing speed, position, Yaw, Heading, line, and
  sensor test workflows.

The currently uncommitted `mission_controller` experiment is not a reason to
add more abstraction. Before this stage is accepted, either remove it or
reduce it to a small adapter around an actually required product task.

Acceptance:

- both profiles build with GCC and TIClang;
- the product profile cannot start a bring-up test accidentally;
- the bring-up profile retains all currently validated test commands;
- startup `HIGH-Z`, suspicious-reset lockout, and wireless update are unchanged;
- no validated control-loop source is moved or rewritten.

Non-goals: no new mission scheduler, no PID changes, no line-speed changes,
and no reference-project edits.

### Stage 2 - Scheduler Boundary Reduction

Status: accepted by operator.

Move the long initialization and periodic-call list out of `app/main.c` into a
small application runtime adapter. `main.c` should retain startup, the main
loop, watchdog handoff, and sleep; the runtime adapter owns task ordering.

Acceptance:

- task order and measured loop/display timing are unchanged;
- all six host tests and both firmware targets pass;
- button stop, command leases, and high impedance behavior are unchanged;
- `main.c` no longer includes every workflow header directly.

Acceptance evidence:

- `main.c` is now an eight-line startup/loop wrapper; initialization,
  scheduler ordering, button routing, display slicing, watchdog handoff, and
  sleep live in `app/runtime/car_runtime.c`.
- GCC and TIClang debug/product targets and all five host tests passed.
- The wireless GCC image was updated through JDY-31 and restarted in
  `READY / HIGH-Z`.
- The operator accepted the supervised `+530`-count, Heading-hold smoke run
  at `700 pps`; it completed normally and returned to `HIGH-Z`.
- Existing button-stop coverage remains valid: an active workflow is stopped
  before any new button action can be queued.

### Stage 3 - Tuning And Diagnostics Isolation

Status: in progress.

Progress: command routing and status-domain isolation accepted. The UART
console is now a thin transport/waveform adapter and
`app/tuning/tuning_command_router.c` owns the existing command grammar and
workflow dispatch. Status emission is split into control, sensor, and mission
modules while the public header and waveform format remain unchanged. The
remaining Stage 3 work is to verify product-profile dependency boundaries.

Split the large tuning command/status implementation by domain while keeping
the external command text and VOFA+ channel groups unchanged. Make detailed
debug mirrors optional in the bring-up profile. The GUI remains the sole
`COM6` owner and the TCP bridge remains the automation path.

Acceptance:

- speed, position, Yaw, Heading, line, IMU, watchdog, and update commands all
  regress-free;
- latest JSON/CSV files and the seven-channel VOFA+ format are unchanged;
- product builds do not depend on test-only workflow headers;
- no foreground mouse automation is introduced.

### Stage 4 - Minimal Mission Composition

Status: pending Stage 3 acceptance.

Only after a concrete competition route requires composition, add the smallest
possible mission API. It may sequence existing owners, but it must not duplicate
their validation, safety, or control state machines. Do not add a generic
multi-action framework in advance of a real route requirement.

Acceptance:

- one documented route or motion task has a clear owner and stop path;
- host tests cover conflict rejection, completion, timeout, and operator stop;
- the standalone loops remain unchanged;
- the implementation stays small enough to understand as one application
  workflow.

### Stage 5 - Safety And Fault Regression

Status: pending Stage 4 acceptance.

Expand host and supervised hardware coverage for sensor-stale, command-lease
expiry, output failure, ownership loss, watchdog reset, wireless-update entry,
and button stop. Every path must prove both target commands are zero and the
motor pins are high impedance.

Acceptance:

- all applicable host tests pass;
- each injected fault reports a stable result and cannot restart motion;
- watchdog test boots into suspicious-reset lockout;
- wireless update still completes from idle without a J-Link.

### Stage 6 - Product Configuration And Calibration

Status: pending Stage 5 acceptance.

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

### Stage 7 - Optional Line-Tracking Optimization

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

The project is considered competition-ready when Stages 1 through 6 are
accepted, the product profile is the image used for the competition, every
mission has a tested stop/fault path, and the final branch and remote hashes
match. Stage 7 is an optimization opportunity, not a reason to destabilize the
validated baseline.

## Current Action

Stage 2 is accepted. Implement Stage 3 only. Do not change the validated inner
loops or the current `1400 pps` line parameters while splitting tuning and
diagnostics ownership.
