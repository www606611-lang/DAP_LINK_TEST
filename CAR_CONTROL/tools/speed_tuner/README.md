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
The GUI supports read, atomic apply, a five-second run, stop, live status, and
two loopback TCP bridges. The GUI is the only process that owns the physical
serial port.

| Consumer | Connection | Data |
| --- | --- | --- |
| VOFA+ | TCP client `127.0.0.1:13470` | FireWater wave channels |
| Codex / MCP / CLI | TCP client `127.0.0.1:13471` | Commands, responses, and raw `wave:` lines |

Configure VOFA+ with the FireWater engine and TCP client
`127.0.0.1:13470`. The bridge emits one six-value FireWater group named
`wave`. The channel order is:

```text
L_Target,L_Speed,R_Target,R_Speed,L_Output,R_Output
```

The values are stored in RAM. A reset restores the firmware defaults. Applying
a configuration never starts the motors, and applying while a test is running
returns `ERR busy`.

The GUI mirrors live data into `runtime/latest_status.json`. Each `Run 5 s`
creates a timestamped CSV file and refreshes `runtime/latest_run.csv`, so the
latest panel state and the full test trace can be inspected without screenshots.

## Direct serial control

`SpeedTunerCli.ps1` uses the GUI control bridge when it is available and falls
back to the serial port when the GUI is not running. Use `-DirectSerial` to
skip bridge discovery. Use `-Takeover` to close the GUI before opening the
requested COM port.

```powershell
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Takeover -Action Get
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Status
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Set -Kp 0.12 -Ki 0.05 -Kd 0 -Target 3500 -Limit 650
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Run
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Step
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Reverse
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Sweep -Target 6000 -Limit 750 -ApplyConfig
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action Lease
```

The CLI never starts a run unless `-Action Run` is explicitly selected.

## Protocol

```text
spd get
spd set KP KI KD TARGET_PPS OUTPUT_LIMIT_PERMILLE
spd run
spd run step
spd run reverse
spd run sweep
spd run lease
spd stop
spd stat
```

The normal ramp uses the configured target. `step` switches between 50%,
100%, and 60% target. `reverse` ramps through zero before applying -70% target.
`sweep` tests 40%, 60%, 80%, and 100% target. The wheel controller adds a
signed board-specific feedforward term before the PID correction so low-speed
starts do not wait for the integrator to overcome the motor dead zone.
`lease` starts the inner speed loop under the `YAW` owner, then deliberately
stops refreshing targets. A passing regression reaches `CMD_TIMEOUT` within
100 ms and returns both motor channels to high impedance.

The firmware also publishes a 100 ms telemetry frame:

```text
wave:LEFT_TARGET,LEFT_SPEED,RIGHT_TARGET,RIGHT_SPEED,LEFT_OUTPUT,RIGHT_OUTPUT
```

Accepted ranges:

```text
Kp       0 .. 5
Ki       0 .. 20
Kd       0 .. 2
Target   100 .. 6000 pps
Limit    100 .. 1000 permille
```
