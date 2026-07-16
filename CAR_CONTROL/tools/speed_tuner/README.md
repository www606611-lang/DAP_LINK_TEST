# CAR Control Tuner

This tool changes the speed-loop configuration and carries position-loop
commands and telemetry over one Bluetooth UART connection.

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
The GUI has separate speed-loop and position-loop tabs. The position tab edits
Kp, relative target counts, maximum speed, output limit, tolerance, straight-line
sync Kp, and maximum sync correction, and provides Run, Stress 24, and Stop
controls. Its live panel shows position target, actual count, error, current
step, completed moves, worst error, wheel speeds, recovery totals, invalid
transitions, result, and high-impedance state. The GUI is the only process that
owns the physical serial port.

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

During a position move the firmware emits a `position` group instead:

```text
Left_Position_Target_Count
Left_Position_Actual_Count
Right_Position_Target_Count
Right_Position_Actual_Count
Left_Cascade_Speed_Target_PPS
Right_Cascade_Speed_Target_PPS
```

The position samples are mirrored into
`runtime/latest_position_wave.json` and
`runtime/latest_position_telemetry.csv`.

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

pos get
pos set KP TARGET_COUNTS MAX_SPEED_PPS OUTPUT_LIMIT_PERMILLE TOLERANCE_COUNTS SYNC_KP SYNC_MAX_PPS
pos run
pos run stress
pos stop
pos stat
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
poswave:LEFT_TARGET_COUNT,LEFT_COUNT,RIGHT_TARGET_COUNT,RIGHT_COUNT,LEFT_SPEED_TARGET,RIGHT_SPEED_TARGET
```

`pos run stress` executes 24 supervised moves: three repetitions of `+1`, `-1`,
`+0.5`, `-0.5`, `+2`, `-2`, `+0.25`, and `-0.25` times the configured target.
Every segment must settle and return to high impedance before the next begins.
Any failed segment stops the full sequence.

The final two `pos set` fields are optional for compatibility with older tools.
When omitted, the current sync settings are preserved. A zero sync gain or zero
sync maximum disables cross-coupling. Sync is applied only when both relative
wheel targets are equal and nonzero.

Accepted ranges:

```text
Kp       0 .. 5
Ki       0 .. 20
Kd       0 .. 2
Target   100 .. 6000 pps
Limit    100 .. 1000 permille
Pos sync Kp   0 .. 20 pps/count
Pos sync max  0 .. 6000 pps
```
