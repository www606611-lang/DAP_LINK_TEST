# DIANSAI_MOTOR_DRIVER_BOARD

Firmware target for the JLCEDA project `电赛电机驱动板`, based on the existing
MSPM0G3507 Tian Meng Star application.

The project keeps the original application modules and updates the board-level
SysConfig pinmux for the motor-driver PCB.

## Board Pin Map

- Motor left, board A path: `PA23` / `PA24` through `TIMG7`
- Motor right, board B path: `PA29` / `PA30` through `TIMG6`
- Encoder left: `PB0` / `PB1`
- Encoder right: `PB2` / `PB3`
- I2C_0 for IMU/OLED firmware path: `PA0` SDA / `PA1` SCL
- I2C_1 expansion/line sensor path: `PA16` SDA / `PA17` SCL
- UART_2: `PA21` TX / `PA22` RX
- UART_3: `PA14` TX / `PA13` RX
- MCAN0 moved off the UART_3 pins: `PA26` TX / `PA27` RX

## Hardware Notes

- The PCB netlist has separate `A0/A1` and `A00/A01` nets. Tian Meng Star U2 is
  on `A00/A01`, while H4/H9 are on `A0/A1`. If those headers are intended for
  I2C0, fix the net names in JLCEDA before fabricating.
- The schematic text marks A as the left motor and B as the right motor. This
  target maps `motorL` to A (`PA23/PA24`) and `motorR` to B (`PA29/PA30`).

## Build

From the repository root:

```powershell
cmake --preset ticlang-debug
cmake --build --preset ticlang-debug --target diansai_motor_driver_board
```

Output:

```text
build-ticlang/DIANSAI_MOTOR_DRIVER_BOARD/diansai_motor_driver_board.out
```
