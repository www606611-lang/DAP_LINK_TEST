# PB21 SPI LCD Test

Standalone CCS/SysConfig test project for the MSPM0G3507 board.

## Test behavior

- PB21 is configured as an active-low input with an internal pull-up.
- The button is polled and software-debounced for 20 ms.
- The ST7789 displays the stable state, press/release counts, last transition time,
  and uptime.
- The same values are exposed to the CCS Expressions view as
  `g_pb21_pressed`, `g_pb21_press_count`, `g_pb21_release_count`, and
  `g_pb21_last_change_ms`.

## Pin configuration

| Function | Pin |
| --- | --- |
| PB21 button | PB21 |
| LCD SPI MOSI | PB8 |
| LCD SPI SCLK | PB9 |
| LCD reset | PB10 |
| LCD data/command | PB11 |
| LCD chip select | PB14 |
| LCD backlight | PB26 |

All peripheral configuration is owned by `pb21_spi_lcd_test.syscfg`. Do not edit
generated `ti_msp_dl_config.*` files.

## Reusing the screen module

Copy `drivers/device/st7789`, `drivers/utility/delay.c`, and
`drivers/utility/delay.h` together. The destination SysConfig project must keep
the instance names `SPI_LCD`, `LCD_CTRL`, and DMA channel `DMA_CH2`, or update
the corresponding macros at the top of `st7789.c`.

## Build

From the repository root:

```powershell
cmake --build build-gcc --target pb21_spi_lcd_test
```

The CCS project can be imported from this directory. Its target configuration is
`targetConfigs/MSPM0G3507.ccxml`.

## VS Code workflow

Run `Terminal -> Run Task -> Build + Flash (J-Link)` to configure CMake,
build only this target with GCC, program the resulting ELF, verify it, reset the
MCU, and start execution. The separate `Build Target (GCC)` and
`Flash (J-Link)` tasks are also available when only one step is needed.

The VS Code task programs this exact output:

```text
build-gcc/PB21_SPI_LCD_TEST/pb21_spi_lcd_test.elf
```

To program through CCS Debug Server and read the test globals after two seconds:

```powershell
& 'D:\TI\ccs\ccs\ccs_base\scripting\bin\dss.bat' `
  PB21_SPI_LCD_TEST\tools\ccs_flash_verify.js `
  PB21_SPI_LCD_TEST\targetConfigs\MSPM0G3507.ccxml `
  PB21_SPI_LCD_TEST\Debug\PB21_SPI_LCD_TEST.out
```

If the CCS flash loader rejects a write, the same CCS-built output can be erased
and programmed with a current SEGGER J-Link installation:

```powershell
& "$env:USERPROFILE\SEGGER\JLink\JLink.exe" `
  -NoGui 1 `
  -CommandFile PB21_SPI_LCD_TEST\tools\flash.jlink
```
