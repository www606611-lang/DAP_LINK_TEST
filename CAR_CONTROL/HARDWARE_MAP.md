# Hardware Map

This table separates confirmed board resources from assumptions inherited from
the older firmware. Unconfirmed pins are intentionally absent from SysConfig.

| Function | MCU pins | Status | Source / decision |
| --- | --- | --- | --- |
| PB21 button | PB21 | Confirmed | Clean board test; active-low, pull-up |
| ST7789 SPI | PB8 MOSI, PB9 SCLK | Confirmed | Clean board test |
| ST7789 control | PB10 RESET, PB11 DC, PB14 CS, PB26 BLK | Confirmed | Clean board test |
| Motor channel A | PA29 AIN1, PA30 AIN2 | Confirmed | Drives AOUT/U4; paired with encoder channel 0 |
| Motor channel B | PA23 BIN1, PA24 BIN2 | Confirmed | Drives BOUT/U3; paired with encoder channel 1 |
| Encoder channel 0 / motor A / legacy left | PB0 A, PB1 B; forward sign inverted | Confirmed | Physical board, legacy SysConfig, hand-turn test, and motor pairing confirmation |
| Encoder channel 1 / motor B / legacy right | PB2 A, PB3 B; forward sign native | Confirmed | Physical board, legacy SysConfig, hand-turn test, and motor pairing confirmation |
| Legacy direction key `up` | PB4 | Reference only | PB4 is not an encoder input |
| IMU I2C | PA0 SDA, PA1 SCL | Pending | Old firmware only |
| Line sensor I2C | PA16 SDA, PA17 SCL | Pending | Old firmware only |
| K230 UART | PA21 TX, PA22 RX in old firmware | Pending | Legacy firmware mapping; not enabled yet |
| Bluetooth UART | PA25/PA26 or PA13/PA14 | Conflict | Old projects disagree; do not configure |
| CAN | PA12/PA13 or PA26/PA27 | Conflict | Old projects disagree; do not configure |

## Motor naming rule

Motor A is paired with encoder channel 0, and motor B is paired with encoder
channel 1. Firmware will keep physical channel names `A` and `B` until chassis
left/right orientation is explicitly confirmed. The older code's left/right
names are retained only as historical hints.

The encoder pin pairs, motor pairing, and forward count signs are confirmed.

MCU internal pulls remain disabled to match both legacy SysConfig files. The
encoder supply, board pull-ups, and output type must still be checked before
connecting a 5 V push-pull encoder.

## Encoder calibration

The GPIO decoder counts every valid A/B state transition. A five-wheel-turn
test measured 5310 counts on channel 0 and 5289 counts on channel 1:

```text
channel 0: 5310 / 5 = 1062.0 counts/wheel revolution
channel 1: 5289 / 5 = 1057.8 counts/wheel revolution
mean:      1059.9 counts/wheel revolution
```

Firmware uses the common nominal value `1060` counts per wheel revolution.
The 0.40% inter-channel spread is within the uncertainty of manual start/stop
alignment; per-wheel correction is not justified by this test.

## Power and reset risks

- Previous firmware linked a PB21 press directly to motor output changes.
- Historical reset causes include `BOR_SUPPLY_FAILURE` and `SYS_FLASH_ECC`.
- The hardware audit identified a possible half-powered/backfeed path when the
  MCU is powered from USB/J-Link while the Buck or AT8236 VM rail is off.
- Motor outputs must remain high impedance until the AT8236, Buck, VM, 5 V,
  3.3 V, and common-ground power-up sequence is valid.
- Future PWM bring-up starts with one physical channel, limited duration, and a
  slew limiter. It must not be tied directly to the PB21 event handler.
