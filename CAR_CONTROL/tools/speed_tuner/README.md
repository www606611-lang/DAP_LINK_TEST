# CAR Speed Loop Tuner

This tool changes the symmetric left/right speed-loop configuration over the
Bluetooth UART without a J-Link debug session.

## Wiring

| MSPM0G3507 | Bluetooth module |
| --- | --- |
| PA26 / UART3 TX | RX |
| PA25 / UART3 RX | TX |
| GND | GND |

The serial format is `9600 8N1`. The Bluetooth module's UART side must use a
3.3 V compatible logic level.

## Run

Open `Launch-SpeedTuner.cmd`, select the Bluetooth serial COM port, and connect.
The GUI supports read, atomic apply, a five-second run, stop, and live status.

The values are stored in RAM. A reset restores the firmware defaults. Applying
a configuration never starts the motors, and applying while a test is running
returns `ERR busy`.

The GUI mirrors live data into `runtime/latest_status.json`. Each `Run 5 s`
creates a timestamped CSV file and refreshes `runtime/latest_run.csv`, so the
latest panel state and the full test trace can be inspected without screenshots.

## Direct serial control

`SpeedTunerCli.ps1` lets Codex or a terminal own the serial port directly. The
GUI and CLI are mutually exclusive because Windows serial ports have one owner.
Use `-Takeover` to close the GUI before opening the requested COM port.

```powershell
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Takeover -Action Get
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Status
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Set -Kp 0.25 -Ki 0.10 -Kd 0 -Target 3500 -Limit 700
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Run
```

## Protocol

```text
spd get
spd set KP KI KD TARGET_PPS OUTPUT_LIMIT_PERMILLE
spd run
spd stop
spd stat
```

Accepted ranges:

```text
Kp       0 .. 5
Ki       0 .. 20
Kd       0 .. 2
Target   100 .. 6000 pps
Limit    100 .. 1000 permille
```
