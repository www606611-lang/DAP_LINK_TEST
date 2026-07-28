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
| ICM20948 IMU I2C0 | PA0 SDA, PA1 SCL | Bench confirmed | Module responds at 7-bit address `0x69`; `WHO_AM_I=0xEA`; 100 Hz reads verified with zero I2C errors |
| Eight-channel line sensor I2C1 | PA16 SDA, PA17 SCL | Bench confirmed | Board CN1 pins 3/2; 7-bit address `0x12`; active-low channels; centered, left, right, and no-line states verified with zero I2C errors |
| ESP32-C3 radio UART3 | PA13 RX, PA14 TX | Firmware installed; end-to-end pending | Main-board U9 RX/TX routes to A13/A14 at 115200 8N1; ESP32 GPIO21 TX connects PA13 RX and GPIO20 RX connects PA14 TX; binary shadow link has zero motion ownership |
| Bluetooth UART2 | PB17 TX, PA22 RX | System confirmed | JDY-31A remains at 115200 baud; application, resident Bootloader, tuner bridge, complete command replies, and startup `HIGH-Z` were verified after moving TX from PA21 |
| CANFD0 | PA26 TX, PA27 RX | Schematic confirmed; firmware pending | Main-board CAN module maps `MCAN0_TX/MCAN0_RX` to A26/A27; CAN remains disabled until its own bring-up gate |
| Electromagnet MOS switch IO | PA2 / TIMG8_CCP1 | Full-power GPIO attraction bench confirmed; reduced-power hold pending | Active-high output; default grip remains at full GPIO power because reduced-duty tests did not retain the steel ball; startup/Bootloader default low; module and MCU share GND |

## Electromagnet MOS module wiring

The 5 V electromagnet is powered through the MOS switch module; its current
must not flow through an MCU pin:

```text
Tianmengxing PA2  -> MOS module IO
Tianmengxing GND  -> MOS module control GND
regulated 5 V +   -> MOS module power input +
regulated 5 V GND -> MOS module power input -
electromagnet +   -> MOS module load output +
electromagnet -   -> MOS module load output -
```

The module in the merchant diagram is treated as active-high. The two repeated
IO/GND pad rows are parallel connection points, not separate power inputs.
Before powering it, measure the electromagnet resistance and estimate its
steady current with `I = 5 V / R`; confirm the 5 V regulator, wiring, and MOS
module current ratings all exceed that value. Keep the electromagnet away from
the IMU and loose steel parts during the first pulse test.

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
- The resident Bootloader explicitly configures PA29, PA30, PA23, and PA24 as
  digital inputs before accepting a wireless update. Its Flash commands cannot
  address the protected `0x0000..0x2FFF` Bootloader partition.
- The guarded speed-loop test enables both channels with a 100 ms target lease
  inside a separate 200 ms hardware lease. The target ramps to 3500 pps, PWM is
  capped at 650 permille, a second PB21 press stops both immediately, and a
  5 second automatic stop returns both channels to high impedance.
