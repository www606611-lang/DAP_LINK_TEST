# Agent Handoff - 2026-05-06

## Scope
This handoff covers the work done in `C:\Users\ASUS\Desktop\mspm0_Project` on:

- K230 rectangle tracking script
- MSPM0 ST7789 status UI
- encoder speed test
- new PID-based encoder motor loop helper

This note is meant for the next agent to continue without re-discovering context.

## Important Working Rule
- The user explicitly wants `main`/`cmsis_dsp_empty.c` to stay clean.
- Do not stuff test parameters or control logic back into `cmsis_dsp_empty.c`.
- Keep test/business logic in separate modules under `app/` or `PID/`.

## Current Main Entry
Current entry file:
- [DAP_LINK_TEST/cmsis_dsp_empty.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/cmsis_dsp_empty.c)

It is intentionally clean and only keeps:
- `app_init()`
- `app_task()`
- init and task scheduling for timer / motor / encoder / imu / lcd / uart rx / speed test

## Added Modules
### 1. Encoder motor PID helper
Files:
- [DAP_LINK_TEST/PID/encoder_motor_pid.h](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/PID/encoder_motor_pid.h)
- [DAP_LINK_TEST/PID/encoder_motor_pid.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/PID/encoder_motor_pid.c)

What it does:
- wraps existing `pid.c/.h`
- supports:
  - speed mode
  - position mode
- structure:
  - position loop outputs speed target
  - speed loop outputs motor PWM

Important implementation detail:
- `EncoderMotorPID_Update()` was fixed to only update when
  `elapsed_ms >= ENCODER_SAMPLE_INTERVAL_MS`
- this was added because the loop was previously updating much faster than encoder speed refresh, causing unstable repeated PID updates on stale speed data

Current behavior:
- output limits apply to PWM command
- `GetSpeedTargetPps()` returns direct speed target in speed mode
- float target is internally tracked, but telemetry export was converted to integer before UART output

### 2. Encoder speed test module
Files:
- [DAP_LINK_TEST/app/encoder_speed_test.h](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/app/encoder_speed_test.h)
- [DAP_LINK_TEST/app/encoder_speed_test.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/app/encoder_speed_test.c)

Purpose:
- standalone speed-loop test without polluting `cmsis_dsp_empty.c`
- auto-start after short delay
- drive both motors with closed-loop speed control
- periodically change target speed for waveform observation
- export telemetry over UART0 for VOFA+

Current test parameters in `encoder_speed_test.c`:
- start delay: `100 ms`
- target step interval: `3000 ms`
- telemetry interval: `50 ms`
- target sequence:
  - `300 pps`
  - `500 pps`
  - `700 pps`
  - `500 pps`
- current speed-loop tunings:
  - left: `kp=0.08 ki=0 kd=0`
  - right: `kp=0.08 ki=0 kd=0`
- integral limit: `120`
- deadband: `24`
- pwm limit: `420`

Current direction handling in test module:
- left encoder inverted:
  - `Encoder_SetInverted(ENCODER_LEFT, true);`
- right motor output inverted:
  - `Motor_SetRightInverted(true);`

Reason:
- user reported left and right wheel behavior strongly asymmetric
- right side was suspected to have opposite control direction
- this was a quick practical correction, not a final verified motor/encoder polarity model

## VOFA+ Telemetry
Telemetry is currently sent from:
- [DAP_LINK_TEST/app/encoder_speed_test.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/app/encoder_speed_test.c)

Format currently sent:
```text
d:targetL,targetR,actualL,actualR,pwmL,pwmR
```

All 6 fields are now integers.

Example:
```text
d:300,300,0,0,120,118
```

Why integer only:
- previously float `snprintf("%.1f")` produced:
  - `d:,,0,0,,`
- root cause was embedded `printf/snprintf` float formatting support not enabled in current build/newlib-nano settings
- telemetry was changed to integer formatting to guarantee waveform output

VOFA+ connection assumptions:
- Serial
- `115200`
- `8N1`
- FireWater protocol

Expected channel order:
1. targetL
2. targetR
3. actualL
4. actualR
5. pwmL
6. pwmR

## ST7789 UI
File:
- [DAP_LINK_TEST/app/lcd_status.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/app/lcd_status.c)

User requirements that were applied:
- keep right-top timer
- remove large title
- remove blue small labels
- remove visible `RX:` label, but keep RX content
- keep content left-aligned and compact
- show only:
  - RX content
  - IMU R/P/Y
  - encoder speeds

Current screen behavior:
- top left: RX content only
- top right: timer `mm:ss`
- middle: `R / P / Y`
- bottom: encoder speed `L / R`

Important note:
- displayed encoder speed is scaled for readability
- `lcd_status.c` divides display value by `10`
- this is display-only
- underlying encoder module still computes raw `speed_pps`

## Encoder Speed Calculation
File:
- [DAP_LINK_TEST/modules/encoder.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/modules/encoder.c)

Current raw speed formula:
```c
encoder->speed_pps = (int32_t) (((int64_t) delta * 1000) / elapsed_ms);
```

Meaning:
- unit is pulses per second
- not RPM
- user was confused by large values
- display was scaled down for ST7789 readability only

## K230 Rectangle Tracking Script
File:
- [矩形识别+串口发.py](C:/Users/ASUS/Desktop/mspm0_Project/矩形识别+串口发.py)

Recent direction before switching focus to MSP side:
- blob-only path replaced earlier `find_rects()` dependency
- later attempts added:
  - merge blobs
  - bright-window based candidate path
- current script contains several experimental detection paths and handoff notes inside file comments

State from latest visible edits:
- build string currently around:
  - `bright-window-v5-tight-box`
- user feedback at various points:
  - perspective/distortion recognition still imperfect
  - some versions over-boxed the target
  - some versions tracked more accurately but box was too large

No further K230 work was done in the later MSP speed-loop session.

## Build Status
Latest build status during this session:
- `cmake --build . --target dap_link_test`
- build succeeded after each meaningful step

## Current Known Problems
### 1. Speed loop still not good
User’s latest real-world feedback:
- left wheel still around `30`
- right wheel powers on around `575`
- user suspects right side direction/sign issues

Interpretation:
- loop is still not tuned/stabilized
- there may still be:
  - right motor direction sign mismatch
  - right encoder sign mismatch
  - motor asymmetry
  - scaling mismatch between target and real encoder response

### 2. VOFA+ waveform had initial issue
Resolved root cause:
- float fields were empty because float formatting was unsupported

Still needs validation on hardware:
- whether complete integer `d:...` frames now show up correctly in VOFA+
- whether FireWater parser draws all 6 channels as expected

### 3. Control reference may still be too abstract
Current speed target uses `pps`.
- This is raw encoder speed, not RPM.
- A future improvement could convert to RPM if encoder counts-per-rev are known.

## Recommended Next Steps
Priority order recommended for next agent:

1. Verify raw UART output on hardware
- confirm lines now look like:
  - `d:300,300,...`
- if not, stop and inspect actual flashed image / UART route

2. Verify right-side sign conventions
- if right wheel still runs away:
  - test removing `Motor_SetRightInverted(true)`
  - instead try `Encoder_SetInverted(ENCODER_RIGHT, true)`
- do not blindly keep flipping both sides
- determine one consistent sign model

3. Only after sign is correct, tune speed loop
- start with single wheel if needed
- keep `ki = 0` initially
- tune `kp` only
- once basic following is stable, add small `ki`

4. Use VOFA+ waveforms as main tuning reference
- watch:
  - targetL vs actualL
  - targetR vs actualR
  - pwmL / pwmR saturation or oscillation

5. If user wants, later add position-loop test mode
- do not put it in `cmsis_dsp_empty.c`
- keep it inside test module or another app module

## Files Added/Changed In This Session
Added:
- [DAP_LINK_TEST/PID/encoder_motor_pid.h](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/PID/encoder_motor_pid.h)
- [DAP_LINK_TEST/PID/encoder_motor_pid.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/PID/encoder_motor_pid.c)
- [DAP_LINK_TEST/app/encoder_speed_test.h](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/app/encoder_speed_test.h)
- [DAP_LINK_TEST/app/encoder_speed_test.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/app/encoder_speed_test.c)

Modified:
- [DAP_LINK_TEST/cmsis_dsp_empty.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/cmsis_dsp_empty.c)
- [DAP_LINK_TEST/app/lcd_status.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/app/lcd_status.c)
- [DAP_LINK_TEST/CMakeLists.txt](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/CMakeLists.txt)
- [矩形识别+串口发.py](C:/Users/ASUS/Desktop/mspm0_Project/矩形识别+串口发.py)
- [DAP_LINK_TEST/app/track_control.c](C:/Users/ASUS/Desktop/mspm0_Project/DAP_LINK_TEST/app/track_control.c)

## User Preference Notes
- Keep `main` / entry file clean.
- Prefer concise, modular code.
- Do not casually modify unrelated functions/files.
- For ST7789:
  - minimalist UI
  - no oversized title clutter
  - keep timer
  - left-aligned compact text

