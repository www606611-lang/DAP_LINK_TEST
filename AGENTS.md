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
- The next integration step is composing Heading correction with position and
  line-tracking base-speed commands. Preserve the validated standalone loops
  while introducing that coordination layer.
