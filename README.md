# MSPM0 ICM-20948 Attitude Demo

MSPM0G3507 project for ICM-20948 attitude display, SSD1306 OLED output, and
UART0 DMA receive display. Yaw is gyro-only with stationary bias tracking.

## Layout

- `DAP_LINK_TEST/app`: application tasks and display logic
- `DAP_LINK_TEST/modules`: device drivers
- `DAP_LINK_TEST/common`: delay and timer helpers
- `DAP_LINK_TEST/openocd`: DAPLink OpenOCD config

## Build

```powershell
cmake --build --preset gcc-debug
```

The `ticlang-debug` preset is kept only as an optional TI Arm Clang fallback.
