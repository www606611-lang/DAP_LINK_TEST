---
name: mspm0-control-debug
description: Use for this MSPM0 project when working on motor control, encoder speed/position loops, PID tuning, VOFA waveform telemetry, ST7789 status display, ICM20948 IMU issues, power/reset symptoms, or when integrating closed-loop movement APIs without polluting main.
---

# MSPM0 Control Debug

## Start Here

Read `AGENT_HANDOFF_2026-05-06.md` first for current project state and user preferences.

Keep `DAP_LINK_TEST/cmsis_dsp_empty.c` clean. Put reusable control logic in `DAP_LINK_TEST/PID/`, hardware drivers in `DAP_LINK_TEST/modules/`, and temporary experiments in `DAP_LINK_TEST/app/` only when needed. Delete or consolidate test modules after validation.

## Debug Order

For motor issues, do not tune PID first. Use this order:

1. Verify open-loop PWM at a small value.
2. Verify each wheel independently.
3. Verify encoder count direction and speed sign.
4. Verify one-wheel closed-loop speed.
5. Verify two-wheel closed-loop speed.
6. Verify position cascade loop.
7. Only then tune KP/KI/KD.

If MCU resets, LCD blanks, or IMU disappears when motors run, consider power integrity, ground, wiring, motor noise, and brownout before changing drivers.

## Current Control Structure

Speed API:

- `EncoderSpeedControl_Init(now_ms)`
- `EncoderSpeedControl_Task(now_ms)`
- `EncoderSpeedControl_SetTargetPps(left, right)`
- `EncoderSpeedControl_Stop()`

Position API:

- `EncoderPositionControl_Init(now_ms)`
- `EncoderPositionControl_Task(now_ms)`
- `EncoderPositionControl_SetTargetCount(left, right)`
- `EncoderPositionControl_AddTargetCount(left_delta, right_delta)`
- `EncoderPositionControl_ZeroPosition(now_ms)`
- `EncoderPositionControl_Stop()`

Position control is cascade PID:

```text
target position -> position PID -> target speed -> speed PID -> PWM
```

Yaw angle control is also cascade-style:

```text
target yaw -> yaw PID -> opposite wheel speed targets -> speed PID -> PWM
```

Use `YawAngleControl_*` APIs for yaw posture tests. If the car rotates opposite to the command, adjust the yaw control sign first, then tune PID.

## PID Tuning Guidance

Do not add KD by reflex. Encoder speed is quantized and D can amplify noise.

Tune in this order:

1. Confirm sign and sampling.
2. Tune KP with KI/KD low or zero.
3. Add KI only for steady-state error.
4. Add KD only if measurement is clean and overshoot cannot be handled by KP/KI/deadband/output limits.

Align PID updates with `ENCODER_SAMPLE_INTERVAL_MS`. Updating PID faster than encoder speed refresh causes repeated control decisions on stale measurements.

## Telemetry Rules

Use VOFA only for the current question. Keep `d:` channels minimal and integer-only unless float printf support is explicitly enabled.

Examples:

- Speed loop: `d:targetL,targetR,actualL,actualR`
- Position loop: `d:targetL,targetR,countL,countR`

Remove temporary telemetry after the test or isolate it in a clearly named debug module.

## Current Hardware Sign Conventions

Current working direction settings:

- `Encoder_SetInverted(ENCODER_LEFT, true)`
- `Motor_SetRightInverted(true)`

If wiring changes, retest sign conventions before tuning.

## Build Check

After meaningful edits, run both:

```powershell
cmake --build build-ticlang --target dap_link_test
cmake --build build-gcc --target dap_link_test
```
