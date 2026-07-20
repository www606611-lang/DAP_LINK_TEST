# K230 Vision And CAN Stepper Integration Plan

Status: Gate 0 communication migration complete. K230 and CAN feature code,
SysConfig resources, and EDA files remain unchanged; Gate 1 is next.

## 1. Scope

Add these independent capabilities to `CAR_CONTROL` without changing the
validated chassis loops:

- receive K230 vision targets over a dedicated UART;
- control two ZDT CAN closed-loop motors on one classic CAN bus;
- keep JDY-31 tuning and wireless application updates available;
- expose device state to the LCD and tuner status without changing the
  accepted seven-channel VOFA+ Yaw stream;
- leave competition behavior in `app/mission` until the actual task is known.

The existing speed, position, Yaw, Heading, line-tracking, motion, and wheel
odometry implementations remain unchanged during device bring-up.

## 2. Confirmed Hardware Map

Sources inspected:

- main board backup:
  `C:/Users/ASUS/Documents/LCEDA-Pro/projects/电赛电机驱动板_backup/电赛电机驱动板_2026-07-14-16-44.epro2`;
- CAN distribution board project:
  `C:/Users/ASUS/Documents/LCEDA-Pro/projects/电赛电机驱动板_before_step_can_plan_20260710_1645.eprj2`;
- MSPM0G3507 SysConfig device pin-mux database;
- `DAP_LINK_TEST` K230 and ZDT reference code.

| Function | Final MCU resource | Board evidence | Decision |
| --- | --- | --- | --- |
| JDY-31 tuning/update | UART2: PB17 TX, PA22 RX | Free Tianmengxing header pins; valid UART2 mux | Move from UART3, then isolate TX from the PA21 route |
| K230 vision | UART3: PA13 RX, PA14 TX | Main-board U9 routes RX to A13 and TX to A14 | Use the intended U9 connector |
| ZDT CAN bus | CANFD0: PA26 CANTX, PA27 CANRX | Main-board CAN module nets map A26/A27 to MCAN0_TX/RX | Use classic CAN at 500 kbit/s |
| LCD DMA | DMA channel 2 | Existing validated ST7789 configuration | Preserve unchanged |

Connector details:

- U9 has `NC/RX/TX/GL`, with no K230 power output. Connect K230 TX to U9 RX
  (PA13), K230 RX to U9 TX (PA14), and connect the grounds.
- The main-board SN65HVD230 module exposes `CANH_STEP` and `CANL_STEP` to CN2.
- The stepper distribution board routes the incoming CAN bus to CN1 and CN2
  for the two motors. Follow the `CANH`, `CANL`, and `GL` net names rather than
  assuming that the connector pin numbers have the same physical orientation.
- The distribution-board R2 position is a 120 ohm termination option and is
  initially DNP. With power removed, the complete bus should measure about
  60 ohms between CANH and CANL when two endpoint terminators are present.

## 3. Resource Conflict Resolution

The former JDY-31 assignment was UART3 on PA25/PA26. It conflicted twice with
the final board interfaces:

- K230 U9 also requires the UART3 peripheral instance on PA13/PA14;
- CANFD0 requires PA26 as CANTX.

The durable allocation is therefore:

```text
UART2 PB17/PA22 -> JDY-31
UART3 PA13/PA14 -> K230 U9
CANFD0 PA26/PA27 -> onboard SN65HVD230
```

Both the application and resident Bootloader now use UART2. The one-time wired
installation of the migrated Bootloader and application is complete. Normal
builds and updates continue over JDY-31/COM6 and the tuner TCP bridge.

Migration is complete only after all of these checks pass:

1. tuner auto-connects on COM6 through UART2 at 115200 8N1;
2. the live application reports HIGH-Z;
3. an application image is updated through the resident Bootloader;
4. the new application starts and again reports HIGH-Z;
5. the tuner TCP status and VOFA+ TCP stream still work.

## 4. Protocol Baselines

### K230

The existing K230 script is the compatibility baseline:

```text
115200 8N1
frame: @valid,cx,cy#
example: @1,203,117#
coordinates: 400 x 240
center: 200,120
offline timeout: 150 ms
```

The first implementation remains receive-only and accepts bounded decimal
fields. A complete valid `valid=0` frame means that the link is online but no
target is present. Only a complete, correctly parsed frame refreshes the link
timestamp; random bytes and partial frames do not keep the device online.

The old reference implementation is not copied directly because it mixes UART
register access, parsing, timeout state, and LCD updates in one application
file. It also lacks coordinate range validation and useful parser statistics.

### ZDT CAN motors

The existing reference establishes this baseline:

```text
classic CAN: 500 kbit/s
motor addresses: 1 and 2
extended identifier: (address << 8) | packet_index
maximum payload: 8 bytes
protocol check byte: 0x6B
```

The reusable device layer will cover enable/disable, velocity, relative and
absolute position, stop, synchronized start, clear-stall, zero/origin, and
read-only status requests. The old million-iteration register polling loops
and direct MCAN ownership inside the ZDT protocol file are not reused.

## 5. Planned Firmware Layers

```text
app/bringup/k230_vision_bringup.*
app/bringup/zdt_stepper_bringup.*
app/actuators/zdt_stepper_supervisor.*
app/mission/<future competition workflow>.*
        |
drivers/device/k230/k230_vision_link.*
drivers/device/zdt/zdt_stepper_can.*
        |
drivers/mcu/vision_uart.*
drivers/mcu/mcan0.*
        |
SysConfig / TI DriverLib / platform
```

Responsibilities:

- `vision_uart`: UART3 FIFO/interrupt byte transport and bounded RX ring;
- `k230_vision_link`: framing, parsing, range checks, timeout, counters, and a
  copyable snapshot; no LCD or motor calls;
- `mcan0`: nonblocking classic-CAN transmit queue, RX FIFO draining, error
  counters, bus state, and controlled bus-off recovery;
- `zdt_stepper_can`: ZDT command encoding, response decoding, per-address state,
  and explicit result codes; no direct MCAN register access;
- bring-up modules: supervised hardware test sequences only;
- `zdt_stepper_supervisor`: command lease, explicit arming, timeout stop, fault
  latch, and board-button stop for accessory motion;
- mission modules: the only place that may combine K230 targets, chassis motion,
  and CAN motor actions for a competition task.

The existing chassis `ControlSupervisor` remains the exclusive owner of wheel
motion. Accessory motor ownership is separate, while an application-level stop
request stops both supervisors.

## 6. Public API Shape

The exact names may be adjusted to existing style, but the behavior boundary is
fixed before implementation.

```c
typedef struct {
    bool online;
    bool target_valid;
    uint16_t cx;
    uint16_t cy;
    int16_t error_x;
    int16_t error_y;
    uint32_t frame_sequence;
    uint32_t last_frame_ms;
    uint32_t parse_error_count;
    uint32_t overflow_count;
    uint32_t timeout_count;
} k230_vision_snapshot_t;

void K230VisionLink_Init(uint32_t now_ms);
void K230VisionLink_Task(uint32_t now_ms);
bool K230VisionLink_GetSnapshot(k230_vision_snapshot_t *snapshot);
```

```c
typedef enum {
    ZDT_RESULT_OK = 0,
    ZDT_RESULT_BUSY,
    ZDT_RESULT_TIMEOUT,
    ZDT_RESULT_BUS_OFF,
    ZDT_RESULT_BAD_ARGUMENT,
    ZDT_RESULT_NOT_ARMED
} zdt_result_t;

zdt_result_t ZdtStepper_Enable(uint8_t address, bool enable);
zdt_result_t ZdtStepper_SetSpeed(uint8_t address, int16_t rpm, uint8_t accel);
zdt_result_t ZdtStepper_MoveRelative(
    uint8_t address, int32_t pulses, uint16_t rpm, uint8_t accel, bool sync);
zdt_result_t ZdtStepper_Stop(uint8_t address, bool sync);
zdt_result_t ZdtStepper_StartSync(void);
bool ZdtStepper_GetSnapshot(uint8_t address, zdt_stepper_snapshot_t *snapshot);
```

No public call waits in a long polling loop. Commands either enter a bounded
queue or return a clear busy/error result.

## 7. Safety And Runtime Rules

- Startup leaves both ZDT motors disabled or explicitly stopped.
- Flashing firmware never starts either chassis or accessory motion.
- A CAN motor command requires an active accessory lease that is refreshed by
  its owning bring-up or mission workflow.
- Lease expiry, any board-button stop, watchdog/reset lockout, CAN bus-off, or
  an unrecoverable device fault sends stop/disable and latches the workflow.
- K230 target loss does not invent a movement command. The owning mission
  chooses stop, hold, or recovery behavior explicitly.
- LCD rendering reads snapshots only and remains sliced so it cannot block the
  100 Hz chassis control work.
- New K230/CAN state is exposed through tuner status/JSON first. The accepted
  seven-channel VOFA+ Yaw packet remains byte-for-byte unchanged.

## 8. Implementation And Acceptance Gates

### Gate 0: communication resource migration

Status: completed 2026-07-19.

- Move application JDY-31 SysConfig to UART2 PB17/PA22.
- Move Bootloader UART to UART2 PB17/PA22.
- Build GCC and TIClang application and Bootloader targets.
- Perform the one-time Bootloader/application installation and complete the
  five migration checks in section 3.

Acceptance: wireless update, tuner, TCP bridge, VOFA+, and HIGH-Z all match the
current validated behavior. The final 75,216-byte image completed two
consecutive wireless updates in 13.8 and 13.9 seconds; each restart reported
`ASTAT state=READY` with `hz=1`, and both TCP ports were revalidated.

### Gate 1: K230 parser and transport

- Add host tests for valid, lost-target, fragmented, concatenated, malformed,
  overflow, out-of-range, and timeout input.
- Add UART3 RX transport and read-only vision snapshot.
- Display ONLINE/LOST, coordinates, frame age, and error counters.

Acceptance: moving the target covers the full 400 x 240 range, center is near
200/120, `valid=0` is distinct from offline, and unplugging data becomes offline
within the expected timeout without affecting chassis timing.

### Gate 2: MCAN transport

- Configure CANFD0 PA26/PA27 for classic 500 kbit/s operation.
- Verify internal loopback before enabling external transmit.
- Verify RX queue, TX completion, error counters, and bus-off behavior.

Acceptance: no blocking-loop timing regression and no unexpected CAN errors in
loopback or on an idle, correctly terminated bus.

### Gate 3: one-motor read-only discovery

- Connect one motor only.
- Query version, state, bus voltage, encoder/position, and fault flags.
- Confirm its address and response framing without issuing enable or motion.

Acceptance: repeatable responses, stable address, zero unexplained bus errors,
and a stopped motor.

### Gate 4: supervised motion and second motor

- Test motor 1 at low speed with a short lease and automatic stop.
- Confirm direction, units, acceleration, stop, and fault handling.
- Repeat independently for motor 2, then test synchronized start/stop.
- Verify every board button stops active accessory motion.

Acceptance: both motors have confirmed addresses and direction signs; repeated
tests stop on command, timeout, and button input without reset or bus-off.

### Gate 5: diagnostics and API promotion

- Add K230/CAN snapshots to the general LCD dashboard and tuner status files.
- Promote only physically accepted device APIs out of bring-up ownership.
- Record results in `BRINGUP_LOG.md`, update `HARDWARE_MAP.md` and
  `ARCHITECTURE.md`, then commit and push only accepted files.

### Gate 6: competition mission integration

- Add the actual task state machine after the competition behavior is known.
- Consume K230 and ZDT snapshots through their public APIs.
- Compose existing chassis Motion/Heading/Line APIs rather than modifying the
  validated inner loops.

Acceptance criteria are written from the real task before this gate begins.

## 9. Open Hardware Checks Before Motion

These checks do not block parser/driver implementation, but must be closed
before Gate 4:

- confirm the exact ZDT motor model and firmware protocol revision from the
  read-only version response;
- confirm addresses 1 and 2 without two same-address devices replying together;
- measure CANH-to-CANL termination with power removed;
- record motor direction/inversion and mechanical travel limits;
- confirm K230 and main-board grounds are common and UART logic is 3.3 V.
