# MSPM0 Project Working Rules

This file is the current source of truth for Codex work in this repository.
The legacy `AGENT.md` and `AGENT_HANDOFF.md` files describe older projects,
contain stale state, and must not override these rules.

## Active Project

- The active firmware is `CAR_CONTROL`.
- `DAP_LINK_TEST` and `DIANSAI_MOTOR_DRIVER_BOARD` are reference projects.
- Do not edit reference projects unless the user explicitly requests it.
- Preserve all unrelated dirty working-tree changes.

## Build And Flash

- Build both supported firmware targets after C/C++ or CMake changes:

```powershell
cmake --build build-gcc --target car_control -j
cmake --build build-ticlang --target car_control -j
```

- The normal programming path is JDY-31 wireless update over `COM6`.
- The user normally has no J-Link connected. Never ask the user to connect,
  disconnect, plug, or unplug J-Link during the normal workflow.
- Mention J-Link only when the user explicitly requests wired debugging or
  wireless recovery has been proven unavailable.
- Automatically flash after firmware changes unless the user asks to defer it.
- Before every wireless flash:
  1. Query live firmware state through the tuner TCP bridge when available.
  2. Require `HIGH-Z` before taking ownership of `COM6`.
  3. Stop only the tuner process that owns TCP ports 13470/13471.
  4. Run `CAR_CONTROL/tools/FirmwareUpdater.ps1` with the GCC application bin.
  5. Restart the tuner hidden with `-AutoConnect -StartMinimized -StartYawMode`.
  6. Query the new firmware and confirm `HIGH-Z` before any motor command.

Typical update command:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File CAR_CONTROL/tools/FirmwareUpdater.ps1 `
  -Port COM6 `
  -Image build-gcc/CAR_CONTROL/car_control.bin
```

## Serial And Tuning Ownership

- The WinForms tuner is the sole owner of `COM6` during normal operation.
- CLI commands should use the tuner TCP bridge at `127.0.0.1:13471`.
- VOFA+ should use the tuner TCP stream at `127.0.0.1:13470`.
- Do not open a second direct serial owner while the tuner is connected.
- Do not use foreground mouse automation to control the tuner or VOFA+.
- Prefer protocol commands plus JSON/CSV files under
  `CAR_CONTROL/tools/speed_tuner/runtime`.
- Preserve the existing seven-channel VOFA+ Yaw waveform format unless the
  user explicitly approves a channel change.

## CanMV Workflow

- The long-lived CanMV integration is the global `canmv` MCP registered in
  `C:/Users/ASUS/.codex/config.toml`; its source is under
  `C:/Users/ASUS/.codex/mcp/canmv` and is reusable across projects.
- Use the CanMV MCP for IDE status, minimized launch, run, stop, console access,
  background capture, and K230 link telemetry. Do not use foreground mouse or
  keyboard automation for CanMV.
- Keep CanMV minimized and never activate its window merely to inspect state.
- When a required CanMV operation or API is missing, extend the global MCP and
  add a focused test before using that capability. Do not fall back to ad hoc
  foreground automation.
- Minimized `PrintWindow` capture currently returns a blank content area. Treat
  it as unavailable evidence and use structured link telemetry or ask for a
  physical-display observation until a direct framebuffer API is added.

## K230-Chassis Wireless Architecture

- The production inter-processor path is:
  `K230 WLAN STA <-> ESP32-C3 SoftAP <-> UART3 <-> Tianmengxing`.
- ESP32-C3 is the chassis-side wireless bridge. A PC may emulate K230 during
  bring-up, but is not part of the production control path.
- K230 owns vision, gimbal computation, MCP2515 CAN, and both ZDT gimbal motors.
  Tianmengxing owns every chassis control loop and both wheel motors.
- K230 and Tianmengxing exchange only framed high-level commands and status.
  They never exchange raw CAN frames, gimbal step pulses, or direct PWM values.
- ESP32-C3 uses GPIO21 TX and GPIO20 RX at 115200 8N1. Connect GPIO21 TX to
  Tianmengxing UART3 PA13 RX, GPIO20 RX to PA14 TX, and share ground.
- ESP32-C3 external 5 V and USB power are mutually exclusive. Disconnect the
  chassis 5 V lead before plugging its USB cable into the computer.
- JDY-31 remains the sole UART2 owner for tuning and wireless application
  updates. The ESP32-C3 bridge must not alter the Bootloader or COM6 workflow.
- The W3 wireless shadow transport is accepted, but received network commands
  still have zero motion ownership. Motor control requires the later W4
  supervisor owner, a command lease, button stop, and automatic HIGH-Z on link
  loss.

## Physical Test Safety

- Flashing firmware does not authorize motor motion by itself.
- Before a new ground-motion profile, state its direction, speed, duration,
  stopping behavior, and required free space.
- If the user has already confirmed the vehicle is positioned for testing,
  proceed without repeating hardware questions.
- Otherwise ask only for physical-space readiness. Do not ask about J-Link.
- Every motor test must retain supervised stop, command leases, and automatic
  return to high impedance.
- Any board button must stop an active bring-up motion before it can start a
  different motion.

## Line-Tracking Test Route

- The ground-test route is a five-corner closed loop made from black tape.
- Its fixed corner set is one right-angle corner, three obtuse corners, and
  one acute corner. The straight segments are approximately 30-45 cm long.
- A full-lap line-tracking claim must cover all five corners. A few wide-line
  events during a short run are not sufficient evidence of a completed lap,
  and the acute corner must be judged separately from the other four.

## Git Workflow

- Do not commit experimental firmware before physical acceptance.
- When the user confirms a physical test passed, commit the verified
  `CAR_CONTROL` changes and push the current branch.
- Stage only files that belong to the accepted work. Never include unrelated
  IDE settings, reference-project edits, or user handoff files.
- Report the commit hash and verify that the local and remote branch hashes
  match.

## Firmware Architecture

- Keep dependency direction consistent with the bare-metal layering:
  `app -> control/device/BSP -> MCU drivers -> platform`.
- Hardware register access belongs in MCU drivers, board wiring in BSP,
  reusable control algorithms in `control`, and test workflows/protocol flow
  in `app`.
- Keep temporary bring-up state machines separate from reusable public APIs.
- Do not modify a previously validated inner loop merely to compensate for a
  new outer loop. Add correction at the owning outer layer.

## Validated Baselines

- Wheel speed control is a validated reusable inner loop.
- Wheel position control and straight-line encoder synchronization are
  validated.
- Relative pivot Yaw control is validated with the promoted defaults recorded
  in `CAR_CONTROL/BRINGUP_LOG.md`.
- LCD dynamic updates are sliced to avoid the former 55-62 ms control stalls.
- Continuous Heading control is ground validated with the promoted defaults
  recorded in `CAR_CONTROL/BRINGUP_LOG.md`.
- The PA16/PA17 eight-channel line-sensor driver is bench validated for center,
  left, right, and no-line states with zero I2C errors.
- The former direct K230 read-only UART3 link is a validated historical
  baseline. UART3 PA13/PA14 is assigned to the ESP32-C3 bridge; the new binary
  parser, timeout, diagnostics, and zero-motion shadow ownership are host-tested
  and installed on Tianmengxing. The production-power end-to-end path and the
  independent K230, ESP32-C3, and Tianmengxing reset/recovery matrix are
  accepted. W4 supervised wireless motion ownership remains pending.
- The composite `MOTION` owner and wheel odometry are ground validated for
  relative forward/reverse distance with startup-heading hold. Preserve the
  exclusive-owner rule; do not start standalone Position, Heading, Yaw, or
  line owners concurrently with a motion command.
- JDY-31 tuning and wireless update use UART2 at 115200 baud with PB17 TX and
  PA22 RX in both the application and resident Bootloader. The eight-byte TX
  training preamble is required for reliable first-byte turnaround. Startup
  must keep all four motor inputs inactive and report `READY / HIGH-Z` before
  any command is accepted.
- The supervised line-tracking outer loop is ground validated on the fixed
  five-corner route at a commanded 1400 pps base speed. Preserve the accepted
  1400 pps behavior and the validated standalone speed, position, Yaw, and
  Heading loops. Treat speeds above 1400 pps as experimental and do not commit
  them until their ground test is accepted.
