# DAP_LINK_TEST

MSPM0G3507 firmware project for the current attitude display demo.

## Runtime Modules

- ICM-20948 accelerometer/gyroscope attitude driver with gyro-only yaw and
  stationary bias tracking
- SSD1306 OLED status and attitude display
- UART0 DMA receive line display
- 1 ms timer and delay helpers

## Build

From the repository root:

```powershell
cmake --build --preset gcc-debug
```

The `ticlang-debug` preset is kept only as an optional TI Arm Clang fallback.
