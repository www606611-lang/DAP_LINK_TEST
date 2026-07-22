# K230 Gimbal And Chassis Integration Plan

Status: Gate 0 and Gate 1 of the chassis vision link and K0/K1 of the independent
K230 gimbal project are complete. CAN and all accessory-motor output remain
disabled until the MCP2515 hardware is installed and K2 begins.

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
  - optional high-level command/status exchange when a mission requires it
```

The K230 vision and gimbal stack starts and runs without Tianmengxing, UART,
the chassis tuner, or any chassis control loop. A future competition mission
may exchange high-level commands and status through UART, but it does not
exchange individual gimbal step pulses, raw CAN frames, or continuous vision
coordinates by default. This keeps rotating gimbal wiring local to the K230
and preserves the validated chassis control timing.

The former, unimplemented plan to connect the two ZDT motors to Tianmengxing
CANFD0 PA26/PA27 is superseded by this document. PA26/PA27 remain unused by the
gimbal path. Gate 0 and Gate 1 history is retained because the accepted UART3
vision link is still part of the final system.

## 2. Confirmed Interfaces

### Optional K230 to Tianmengxing link

| Function | K230 | Tianmengxing | Settings |
| --- | --- | --- | --- |
| Vision/status TX | UART2 GPIO11 TX | U9 RX / UART3 PA13 | 115200 8N1 |
| Reserved command RX | UART2 GPIO12 RX | U9 TX / UART3 PA14 | 115200 8N1 |
| Reference | Common GND | U9 GL | 3.3 V logic |

This interface is disabled in the independent K230 build. The previously
validated compatibility frame remains available only as a legacy diagnostic
adapter:

```text
@valid,cx,cy#
coordinates: 400 x 240
center: 200,120
example: @1,203,117#
lost target while online: @0,last_cx,last_cy#
Tianmengxing offline timeout: 150 ms
```

`valid=0` is an online no-target frame. It is not an offline indication.

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
  chassis_link.py     disabled legacy UART diagnostic adapter
  mcp2515.py          added in K2 with the tested controller driver
  zdt_motor.py        added in K3 after read-only protocol evidence
  gimbal_control.py   added in K4 after motor units and limits are known
```

Files are added at the gate that gives them real behavior and tests. Empty
placeholder drivers are deliberately excluded. The former root-level script
`矩形识别+串口发.py` was removed after K1 acceptance; its Git history and the
backed-up device entry remain available for rollback.

## 4. Runtime Safety Contract

- `config.CHASSIS_LINK_ENABLED` defaults to `False`; K230 startup does not
  initialize UART2 or require the Tianmengxing board.
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
- Enable the optional Tianmengxing UART only if that task needs high-level
  chassis/gimbal command or status exchange.
- Compose existing chassis Motion/Heading/Line APIs without changing validated
  inner loops.

Acceptance criteria are written from the real task before K6 begins.

## 7. Hardware Checks Before K3/K4

- Confirm K230, MCP2515 module and both ZDT devices share the required reference.
- With power removed, measure CANH-to-CANL termination; expect about 60 ohms when
  two endpoint 120-ohm terminators are installed.
- Confirm the exact ZDT model and firmware protocol from read-only responses.
- Confirm addresses one at a time before both devices share the bus.
- Record axis direction, reduction, units, soft limits and physical hard stops.
