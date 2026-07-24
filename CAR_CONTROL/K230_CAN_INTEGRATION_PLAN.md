# K230 Gimbal And Chassis Integration Plan

Status: Gate 0/Gate 1, K0/K1, and wireless W0/W1/W2 are complete. The W3
end-to-end data path is validated; its independent endpoint-reset matrix is
still pending. CAN and all accessory-motor output remain disabled until the
MCP2515 hardware is installed and K2 begins.

## 1. Final Ownership

The system is split by physical responsibility:

```text
K230 camera board
  - YOLO target detection
  - gimbal Yaw/Pitch control
  - SPI MCP2515 CAN controller
  - two ZDT CAN motors and their feedback

Tianmengxing MSPM0 board
  - chassis speed/position/Yaw/Heading/line loops
  - chassis odometry and competition mission execution
  - high-level command/status exchange through the ESP32-C3 wireless bridge
```

The K230 vision and gimbal stack starts and runs without Tianmengxing, UART,
the chassis tuner, or any chassis control loop. When a competition mission
needs chassis motion, K230 exchanges framed high-level commands and status over
WiFi with the ESP32-C3 bridge on the chassis. It does not exchange individual
gimbal step pulses, raw CAN frames, direct PWM values, or continuous vision
coordinates. This keeps rotating gimbal wiring local to K230 and preserves the
validated chassis control timing.

The former, unimplemented plan to connect the two ZDT motors to Tianmengxing
CANFD0 PA26/PA27 is superseded by this document. PA26/PA27 remain unused by the
gimbal path. Gate 0 and Gate 1 history is retained because the accepted UART3
vision link is still part of the final system.

## 2. Confirmed Interfaces

### K230 to Tianmengxing wireless link

```text
K230 built-in WiFi / CanMV network.WLAN (STA)
    <-> WPA2 + UDP
ESP32-C3 SuperMini (SoftAP 192.168.4.1)
    <-> UART 115200 8N1
Tianmengxing UART3 PA13 RX / PA14 TX
```

Bottom-side wiring:

| ESP32-C3 | Tianmengxing | Function |
| --- | --- | --- |
| GPIO21 TX | U9 RX / PA13 | ESP32 to chassis |
| GPIO20 RX | U9 TX / PA14 | Chassis to ESP32 |
| GND | U9 GL / board GND | Common 3.3 V logic reference |
| 5V | Board 5 V rail | External module power; not U9 NC |

ESP32-C3 external 5 V and USB power are mutually exclusive. The chassis 5 V
lead is disconnected before USB programming. JDY-31 UART2 PB17/PA22 remains
unchanged for tuning and wireless application updates.

Both UDP and UART carry one bounded binary format:

```text
0xA5 0x5A | version | type | sequence_le16 | length | payload | crc16_le
version: 1
maximum payload: 64 bytes
CRC: CRC-16/CCITT-FALSE over version through payload
```

Initial shadow messages are `HELLO`, `HEARTBEAT`, and `STATUS`. Command and
emergency-stop message IDs are reserved but have no motion ownership until the
separate supervised-motion gate is physically accepted.

### K230 to MCP2515 module

The selected module is a complete 3.3 V MCP2515 plus SN65HVD230 CAN module
with an 8 MHz oscillator.

| Module signal | Lushan Pi K230 physical pin | K230 function |
| --- | ---: | --- |
| VCC | 17 | 3V3 |
| GND | 20 | GND |
| SCK | 23 | GPIO15 / QSPI0_CLK |
| SI / MOSI | 19 | GPIO16 / QSPI0_D0 |
| SO / MISO | 21 | GPIO17 / QSPI0_D1 |
| CS | 24 | GPIO14 / QSPI0_CS0 |
| INT | 18 | GPIO19 |

The complete accessory-control chain is:

```text
K230 control task
  -> SPI/QSPI0
  -> MCP2515 classic-CAN controller
  -> onboard SN65HVD230 transceiver
  -> CANH/CANL trunk
  -> ZDT motor address 1 and address 2
```

The module oscillator value is part of the CAN bit-timing calculation. An
8 MHz configuration must not be replaced with the common 16 MHz constants.

## 3. K230 Project Layout

The only script an operator runs is `K230_GIMBAL/main.py`. During IDE testing,
it adds `/sdcard/K230_GIMBAL` to `sys.path` and imports the synchronized modules.
When K1 is accepted, the same entry point may be installed as `/sdcard/main.py`
for power-on execution.

```text
K230_GIMBAL/
  main.py             only entry point and startup boundary
  config.py           validated pins, dimensions, model and feature gates
  vision.py           sensor, AI2D, KPU, postprocess, display and cleanup
  wire_protocol.py    shared bounded wireless framing and CRC
  chassis_radio.py    nonblocking WLAN STA and UDP state
  mcp2515.py          added in K2 with the tested controller driver
  zdt_motor.py        added in K3 after read-only protocol evidence
  gimbal_control.py   added in K4 after motor units and limits are known
```

Files are added at the gate that gives them real behavior and tests. Empty
placeholder drivers are deliberately excluded. The former root-level script
`矩形识别+串口发.py` was removed after K1 acceptance; its Git history and the
backed-up device entry remain available for rollback.

## 4. Runtime Safety Contract

- `config.CHASSIS_RADIO_ENABLED` defaults to `False` until the wireless shadow
  link is accepted. K230 vision and gimbal startup never require the chassis.
- `config.CAN_ENABLED` and `config.GIMBAL_MOTION_ENABLED` default to `False`.
- K0/K1 do not import or initialize SPI, MCP2515, or any motor object.
- Starting, stopping, or synchronizing a K230 script never creates motion.
- K2 first verifies MCP2515 reset, register access, oscillator timing, loopback,
  receive queues, transmit completion, and controller error state.
- K3 performs read-only ZDT discovery before any enable or motion command.
- K4 motion requires explicit arming, bounded travel, direction and limit data,
  a command lease, timeout stop, and a physical stop path.
- Loss of a vision target never invents a motor command. The gimbal owner makes
  the hold, search, or stop decision explicitly.
- The Tianmengxing chassis remains `READY / HIGH-Z` during K230 device bring-up.

## 5. Protocol Baselines

### Vision

K1 must preserve the accepted implementation without parameter drift:

```text
model: /data/best.kmodel
labels: /data/labels.txt
sensor id: 2
preview: 800 x 480 ST7701
AI frame: 640 x 384
model input: 320 x 320
fixed focus: 210
confidence/NMS: 0.45 / 0.45
measured baseline: approximately 19.4 FPS
```

### ZDT CAN

The reference-project assumptions are provisional until K3 reads the real
devices:

```text
classic CAN: 500 kbit/s
candidate motor addresses: 1 and 2
extended identifier: (address << 8) | packet_index
maximum payload: 8 bytes
protocol check byte: 0x6B
```

No K230 public call may wait in an unbounded polling loop. SPI/CAN operations
must return a bounded result or queue work for a later scheduler tick.

## 6. Acceptance Gates

### Gate 0: JDY-31 resource migration

Status: completed 2026-07-19.

JDY-31 application update and tuning moved to UART2 PB17/PA22. The application
and resident Bootloader were installed and wireless update, tuner, TCP bridge,
VOFA+, and startup HIGH-Z were physically accepted. Two consecutive 75,216-byte
wireless updates completed in 13.8 and 13.9 seconds.

### Gate 1: Tianmengxing K230 vision receiver

Status: completed 2026-07-20.

UART3 PA13/PA14 receives bounded `@valid,cx,cy#` frames. Fragmented,
concatenated, malformed, oversize, out-of-range, resynchronization, and timeout
tests pass. The live system reached the full 400 x 240 range with zero parser
errors and zero UART overflow while the chassis remained `READY / HIGH-Z`.

### K0: revised architecture freeze

Status: completed 2026-07-22.

- Assign both ZDT motors and their CAN transport to the K230.
- Keep Tianmengxing UART3 only as a disabled optional diagnostic/status link.
- Record the 8 MHz MCP2515 module and final SPI pin map.
- Mark the former Tianmengxing MCAN Gate 2-6 route as superseded.

Acceptance: this document, the hardware map, ownership, protocols, and gate
boundaries agree before any CAN driver is written.

### K1: single-entry K230 vision project

Status: completed and physically accepted 2026-07-22.

- Split the accepted 459-line script into `main`, `config`, and `vision`
  ownership while keeping the former UART publisher as a disabled adapter.
- Add host tests for coordinate mapping, packet bounds, last-valid coordinates,
  letterbox math, target selection, and NMS behavior.
- Synchronize the project under `/sdcard/K230_GIMBAL` without replacing the
  current `/sdcard/main.py`.
- Run `K230_GIMBAL/main.py` through the minimized CanMV IDE.

Acceptance: the K230 starts without Tianmengxing or UART; startup and cleanup
have no exception; focus remains 210; physical prediction and preview match the
accepted script; measured FPS remains close to 19.4. Tianmengxing telemetry is
not part of K1 acceptance.

Evidence: seven host tests cover the disabled hardware gates, coordinate and
packet bounds, last-valid coordinates, letterbox math, target selection, and
NMS. All four runtime files were synchronized to `/sdcard/K230_GIMBAL` and
verified by SHA-256 readback. The former 11,481-byte `/sdcard/main.py` was backed
up before installing the new 442-byte single entry point. The operator accepted
the live preview, detection box, and fixed-focus result. CanMV remained
connected, minimized, and running; UART, CAN, and motor output stayed disabled.

### K2: MCP2515 transport, no motor commands

- Implement QSPI0 SPI transfer, CS and INT handling.
- Verify reset and register read/write against the 8 MHz device.
- Configure classic CAN 500 kbit/s and validate MCP2515 loopback.
- Validate bounded TX/RX queues, timeouts and error counters.
- Enter listen-only mode on the external bus before normal mode is allowed.

Acceptance: repeated loopback and idle-bus reads have no unexplained errors;
the chassis and both ZDT motors remain stopped.

### K3: one-motor read-only discovery

- Connect/query one ZDT motor at a time.
- Read version, address, state, bus voltage, encoder/position and fault flags.
- Record the real response format before accepting addresses 1 and 2.

Acceptance: responses repeat reliably, address ownership is unambiguous, bus
errors remain zero, and neither motor is enabled.

### K4: supervised two-axis motion

- Add enable/disable, stop, speed, relative/absolute position and sync commands.
- Confirm units, direction, acceleration and mechanical travel limits per axis.
- Add explicit arming, command lease, timeout stop and fault latch.
- Validate each motor independently before synchronized commands.

Acceptance: each axis stops on command, lease expiry and fault; all motion is
bounded; no K230 or Tianmengxing reset occurs.

### K5: gimbal control

- Add target filtering, deadband, limits and Yaw/Pitch trajectory generation.
- Close the loop from vision coordinates through ZDT feedback.
- Keep raw CAN and motor protocol details out of the controller.

Acceptance: center hold, step response, target loss/reacquisition, edge targets
and long-duration thermal behavior pass without oscillation or limit impact.

### K6: competition integration

- Define the actual task state machine after the competition behavior is known.
- Exchange high-level chassis commands and status through the accepted
  K230/ESP32-C3 wireless API.
- Compose existing chassis Motion/Heading/Line APIs without changing validated
  inner loops.

Acceptance criteria are written from the real task before K6 begins.

## 7. K230-Chassis Wireless Gates

### W0: protocol and ESP32-C3 bridge build

Status: complete 2026-07-23; ESP32-C3 firmware installed.

- Add host-tested framing, CRC, sequence, length, and resynchronization logic.
- Build an ESP32-C3 SoftAP/UDP/UART bridge with PlatformIO.
- Keep all command messages in shadow mode.

Acceptance: both firmware projects build; malformed input is rejected; no
motor API is referenced by the wireless modules.

Evidence: K230 has 11 passing host tests, ESP32-C3 has five passing protocol
tests and a successful `-Os` PlatformIO build, and Tianmengxing has seven
passing host tests plus successful GCC/TIClang builds. ESP32-C3 uses 738,980
bytes Flash and 39,716 bytes RAM in the current Arduino build.

### W1: K230 to ESP32-C3 WiFi heartbeat

Status: accepted 2026-07-23.

- K230 joins the ESP32-C3 SoftAP through nonblocking `network.WLAN` state.
- Exchange HELLO and HEARTBEAT datagrams without affecting the accepted vision
  preview, focus, or sustained frame rate.

Acceptance: sequence advances, packet age remains bounded, reconnect succeeds,
and K230 vision remains physically normal.

Evidence: a 90-second run included an ESP32 restart. K230 returned to
`RADIO ONLINE` in about 8.5 seconds, received 318 frames, and sent 299 frames
with zero socket, CRC, duplicate, or ordering errors.

### W2: ESP32-C3 to Tianmengxing UART3 shadow link

Status: accepted on production chassis power 2026-07-24.

- Replace the retired direct K230 UART3 parser with interrupt-driven bidirectional
  wireless transport on PA13/PA14.
- Exchange HELLO, HEARTBEAT, and chassis STATUS only.
- Expose link age and parser counters while retaining `READY / HIGH-Z`.

Acceptance: fragmented, concatenated, malformed, overflow, CRC, duplicate,
out-of-order, and timeout tests pass; live traffic never arms a motor.

Evidence: the 78,776-byte Tianmengxing image was installed wirelessly in 15.4
seconds and retained `READY / HIGH-Z`. Live ESP32 traffic advanced both UART3
directions with zero parser, CRC, length, overflow, or transmit-drop errors.
The final K230 run reported `esp=1` and `chassis=1` throughout.

### W3: end-to-end shadow system

Status: sustained bidirectional path accepted 2026-07-24; independent K230,
ESP32-C3, and Tianmengxing reset/recovery matrix pending.

- Validate K230 -> WiFi -> ESP32 -> UART3 -> Tianmengxing and the reverse path.
- Test resets and reconnects independently at all three endpoints.

Acceptance: bidirectional counters advance for a sustained run, command frames
remain shadow-only, and chassis timing/watchdog metrics do not regress.

Evidence so far: under chassis 5 V power, K230 reached end-to-end online state
in about one second and stayed online for 30 seconds. It received 289 frames
and sent 109 frames with zero CRC, length, duplicate, out-of-order, or socket
errors. The SoftAP remained visible in all 11 scans at about 87% signal.

### W4: supervised wireless chassis owner

- Add one exclusive wireless outer-loop owner after the real command semantics
  are defined.
- Require explicit arm, monotonically advancing command sequence, a bounded
  command lease, board-button stop, and automatic stop on either link timeout.
- Submit targets through the validated speed/motion APIs; never call PWM or the
  motor device driver directly.

Acceptance: low-speed suspended-wheel and ground tests stop on command, timeout,
WiFi loss, ESP32 reset, K230 reset, and every board button.

## 8. Hardware Checks Before K3/K4

- Confirm K230, MCP2515 module and both ZDT devices share the required reference.
- With power removed, measure CANH-to-CANL termination; expect about 60 ohms when
  two endpoint 120-ohm terminators are installed.
- Confirm the exact ZDT model and firmware protocol from read-only responses.
- Confirm addresses one at a time before both devices share the bus.
- Record axis direction, reduction, units, soft limits and physical hard stops.
