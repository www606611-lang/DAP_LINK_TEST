# K230 Gimbal Firmware

Status: physically accepted and frozen on 2026-07-24.

Validated build: `k230-gimbal-k8-hud-r1`

This project owns K230 vision, the local MCP2515 CAN bus, and both ZDT gimbal
motors. It runs independently from the Tianmengxing chassis. Chassis wireless
commands remain disabled until the competition mission is defined and accepted
as a separate integration gate.

## Run And Deploy

There is one application entry point:

```text
K230_GIMBAL/main.py
```

The same file is installed as `/sdcard/main.py`, so normal power-on execution
does not require CanMV IDE. During development, run `K230_GIMBAL/main.py` or the
small local probe that imports `/sdcard/K230_GIMBAL/vision.py`; both enter the
same runtime.

Runtime files are installed under:

```text
/sdcard/K230_GIMBAL/
```

Use the global `canmv` MCP for minimized IDE control, synchronization, SHA-256
readback verification, console telemetry, and device status. Do not use
foreground mouse automation.

## Hardware Baseline

### MCP2515 module

The installed module uses an 8 MHz MCP2515 oscillator and an SN65HVD230 CAN
transceiver. CAN bitrate is 500 kbit/s.

| Signal | Lushan Pi pin | K230 function |
| --- | ---: | --- |
| VCC | 17 | 3V3 |
| GND | 20 | GND |
| SCK | 23 | GPIO15 / QSPI0_CLK |
| MOSI | 19 | GPIO16 / QSPI0_D0 |
| MISO | 21 | GPIO17 / QSPI0_D1 |
| CS | 24 | GPIO14 / QSPI0_CS0 |
| INT | 18 | GPIO19 |

Both CAN endpoints require termination. With power removed, CANH-to-CANL should
measure approximately 60 ohms when both 120-ohm terminators are enabled.

### Gimbal axes

| Axis | CAN address | ZDT firmware | Positive command |
| --- | ---: | --- | --- |
| Yaw | 1 | X | CW |
| Pitch | 2 | Emm | UP |

Both installed motors are 1.8-degree units. Yaw address 1 must read back
`MotType=1.8 deg` and option status `0x04` before motion. On the installed
FW_X build, commissioning command `D7 35 01 19 6B` selects the correct
1.8-degree mode even though the X42S V1.0.3 manual associates `0x19` with the
0.9-degree type. Do not write `MotType` during normal startup. After changing
it, run `Cal_MFL` once and require healthy phase R/L and clear motor flags
before enabling Yaw.

Both axes use continuous rotation because the assembled gimbal uses conductive
slip rings. Position-mode APIs retain bounded limits for commissioning, but the
validated visual tracker uses supervised velocity mode.

## Vision Baseline

```text
model: /data/best.kmodel
labels: /data/labels.txt
sensor id: 2
display: 800 x 480 ST7701
AI frame: 640 x 384
model input: 320 x 320
fixed focus: 210
acquire confidence: 0.45
hold confidence: 0.25
NMS threshold: 0.45
target coordinates: 400 x 240, center 200 x 120
```

The target selector holds the current object near image edges and does not jump
to a distant detection while locked. Loss and reacquisition preserve the same
tracking direction conventions.

## Accepted Tracking Parameters

```text
deadband X/Y: 12 / 10 px
hysteresis X/Y: 4 / 3 px
Yaw gain: -0.135 RPM/px
Pitch gain: -0.090 RPM/px
filter alpha: 0.68
tracking update: 40 ms
Yaw speed: 1.0 to 11.0 RPM
Pitch speed: 2.0 to 7.0 RPM
Yaw acceleration: 300 RPM/s
Pitch acceleration setting: 40
command refresh: 150 ms
command lease: 600 ms
```

These values are the accepted practical limit of the current vision and motor
pipeline. Further gain or speed increases are expected to trade smoothness for
overshoot rather than materially reduce latency.

## Runtime Safety

- Startup discovers both motors read-only before transferring CAN to the gimbal
  owner.
- Motion requires successful explicit arming and a bounded command lease.
- A missing target stops motion after 120 ms and becomes `TRACK LOST` after
  500 ms.
- A single timeout while staging a synchronized speed command is retried once.
  The command has not been triggered at that point, so the retry cannot stack
  motion. A repeated failure stops both axes and latches a fault.
- Feedback, voltage, motor flags, and command timing are polled with bounded
  timeouts.
- Chassis UDP receive is nonblocking. If ESP32 heartbeats remain absent for five
  seconds while CanMV still reports WiFi connected, the radio state machine
  forces one WLAN reconnect instead of remaining in a stale `RADIO WAIT` state.
- IDE stop, script cleanup, target loss, lease expiry, and faults all return the
  gimbal to a stopped state.
- `CHASSIS_RADIO_ENABLED` remains `False`; the gimbal does not depend on the
  chassis or ESP32-C3.

## Display HUD

The on-device display refreshes target boxes at the vision frame rate and text
at 10 Hz to avoid adding control latency.

| Area | Values |
| --- | --- |
| Top left | Tracking state, target coordinates, X/Y error |
| Top right | FPS, AI latency, fixed focus |
| Bottom left | Yaw/Pitch command RPM and measured angle in degrees |
| Bottom right | CAN errors, timeouts, retries, and both bus voltages |

State colors are green for normal tracking, yellow for search/lost/warning, and
red for a fault.

## Acceptance Evidence

- Physical two-axis tracking, center lock, edge tracking, target loss and
  reacquisition were accepted by the operator.
- Final HUD layout, readability, and live values were physically accepted.
- Final sustained frame rate is approximately 21.6 to 21.8 FPS.
- Normal vision time is 32 to 35 ms; periodic model frames remain approximately
  114 to 122 ms.
- Normal CAN command time is 8 to 11 ms. Typical Yaw response is 41 to 44 ms;
  Pitch response is normally slower and may exceed 100 ms.
- The final post-deployment observation completed hundreds of tracking commands
  with zero CAN controller errors and zero timeouts.
- Yaw was recommissioned on 2026-07-26 after correcting `MotType` from 0.9 to
  1.8 degrees. `Cal_MFL` produced 1539 mOhm / 3050 uH with flags `0x02`.
  Bounded 5 RPM tests moved +54.6 and -54.7 degrees in two seconds, held
  4.7-5.3 RPM in both directions, and ended disabled without stall or protect.
- All 56 host tests pass.
- Every device runtime file and `/sdcard/main.py` passed SHA-256 readback
  verification after installation.

Accepted Git baselines:

```text
d43183c  responsive tracking parameters
9f6b4f4  production HUD and bounded transient command retry
```

## Frozen Boundary

Treat vision selection, velocity tracking, ZDT protocol, CAN timing, and the HUD
as validated reusable infrastructure. Competition work belongs in a separate
mission layer. It may submit high-level gimbal or chassis goals, but it must not
rewrite the accepted tracker merely to compensate for mission behavior.

The K230-to-ESP32 WiFi/UDP heartbeat gate, the bidirectional
ESP32-to-Tianmengxing UART3 shadow gate, and the W3 independent endpoint-reset
matrix are accepted. The production-power end-to-end run stayed online for 30
seconds with `esp=1`, `chassis=1`, 289 received frames, 109 transmitted frames,
and zero protocol/socket errors. K230 power cycling, ESP32-C3 reset, and
Tianmengxing reset each returned automatically to end-to-end online state while
the chassis remained `READY / HIGH-Z`. The later supervised chassis owner
remains a separate gate and does not block the independent gimbal baseline.
