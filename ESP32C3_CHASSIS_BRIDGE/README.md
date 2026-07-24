# ESP32-C3 Chassis Bridge

This is the permanent K230-to-Tianmengxing transport:

```text
K230 WiFi STA <-> ESP32-C3 SoftAP/UDP <-> UART3 <-> Tianmengxing
```

The bridge validates and re-encodes bounded protocol frames. It has no motor,
PID, PWM, or chassis-control API. CONTROL and EMERGENCY_STOP frames remain
shadow traffic until the later supervised-owner gate.

## Runtime wiring

| ESP32-C3 SuperMini | Tianmengxing |
| --- | --- |
| GPIO21 TX | U9 RX / PA13 |
| GPIO20 RX | U9 TX / PA14 |
| GND | U9 GL / board GND |
| 5V | Stable board 5V rail |

UART is 115200 8N1. External 5V and USB power are mutually exclusive. Remove
the chassis 5V lead before connecting ESP32-C3 USB to the computer.

## Build and upload

```powershell
pio run -e esp32c3
pio run -e esp32c3 -t upload --upload-port COM_PORT
pio device monitor --port COM_PORT --baud 115200
```

The SoftAP is `CAR-K230` at `192.168.4.1`; UDP listens on port 4210 and replies
to the K230 socket on port 4211. USB diagnostics report station/peer state,
UDP/UART frame counts, CRC errors, and drops every two seconds.

The bridge rate-limits failed heartbeat sends and rebuilds SoftAP/UDP once when
a station disconnects. A three-second cooldown prevents the restart's internal
disconnect event from causing a second rebuild.

## Acceptance evidence

- ESP32-C3 revision 0.4, 4 MB flash, MAC `7C:4F:AD:4B:C6:A0`.
- Release build uses 12.1% RAM and 56.6% application flash.
- K230 obtained `192.168.4.2` in about 1.5 seconds and stayed associated for a
  120-second baseline with zero disconnects.
- A 90-second heartbeat run included a live ESP32 firmware update. K230
  detected the stale WiFi status, forced one reconnect after five seconds, and
  returned to `RADIO ONLINE` in about 8.5 seconds.
- The final K230 run received 318 frames and sent 299 frames with zero socket,
  CRC, duplicate, or ordering errors. The post-restart ESP32 received and
  parsed 148 frames with zero CRC errors and zero drops.
- Testing confirmed that USB and chassis power must remain mutually exclusive.
  Unplugging USB while chassis power was still connected did not reset uptime
  and produced misleading radio and temperature observations.

This accepts the K230-to-ESP32 WiFi/UDP heartbeat gate. UART3 remains shadow
traffic with zero motion ownership until end-to-end chassis acceptance.
