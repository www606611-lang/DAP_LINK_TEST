# Hardware Map

This table separates confirmed board resources from assumptions inherited from
the older firmware. Unconfirmed pins are intentionally absent from SysConfig.

| Function | MCU pins | Status | Source / decision |
| --- | --- | --- | --- |
| Board button | PB21 | Confirmed | Clean board test; active-low, MCU pull-up |
| SW2 position button | PB4 | Schematic confirmed | Active-low; external 200 kOhm pull-up and 100 nF to GND; MCU pull-up enabled |
| SW1 position button | PB5 | Schematic confirmed | Active-low; external 200 kOhm pull-up and 100 nF to GND; MCU pull-up enabled |
| ST7789 SPI | PB8 MOSI, PB9 SCLK | Confirmed | Clean board test |
| ST7789 control | PB10 RESET, PB11 DC, PB14 CS, PB26 BLK | Confirmed | Clean board test |
| Motor channel A / left wheel | PA29 TIMG6_CCP0/AIN1, PA30 TIMG6_CCP1/AIN2 | Bench confirmed | 20 kHz PWM; drives AOUT/U4; paired with encoder channel 0 |
| Motor channel B / right wheel | PA23 TIMG7_CCP0/BIN1, PA24 TIMG7_CCP1/BIN2; forward command inverted | Bench confirmed | 20 kHz PWM; drives BOUT/U3; paired with encoder channel 1 |
| Encoder channel 0 / motor A / left wheel | PB0 A, PB1 B; forward sign inverted | Bench confirmed | Physical board, hand-turn calibration, and powered left-wheel test |
| Encoder channel 1 / motor B / right wheel | PB2 A, PB3 B; forward sign native | Bench confirmed | Hand-turn and corrected powered-motor tests both produce positive forward feedback |
| IMU I2C | PA0 SDA, PA1 SCL | Pending | Old firmware only |
| Line sensor I2C | PA16 SDA, PA17 SCL | Pending | Old firmware only |
| K230 UART | PA21 TX, PA22 RX in old firmware | Pending | Legacy firmware mapping; not enabled yet |
| Bluetooth UART3 | PA26 TX, PA25 RX | Configured; bench pending | Reuses the `DAP_LINK_TEST` mapping and generated 9600 baud setting; all Tianmengxing pins are available on the headers |
| CAN | PA12/PA13 or PA26/PA27 | Conflict | PA26 is now reserved for Bluetooth UART3 TX; CAN remains disabled |

## Motor naming rule

Motor A is paired with encoder channel 0 and is confirmed as the left wheel.
Motor B is paired with encoder channel 1 and is confirmed as the right wheel.
Its physical drive polarity is inverted in firmware so logical forward is
positive on both the motor command and E1 feedback.

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
- The guarded speed-loop test enables both channels with a 100 ms target lease
  inside a separate 200 ms hardware lease. The target ramps to 3500 pps, PWM is
  capped at 650 permille, a second PB21 press stops both immediately, and a
  5 second automatic stop returns both channels to high impedance.
