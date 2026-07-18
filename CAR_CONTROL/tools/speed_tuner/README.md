# CAR Control Tuner

This tool changes speed-, position-, and Yaw-loop configuration and carries
their commands and telemetry over one Bluetooth UART connection.

## Wiring

| MSPM0G3507 | Bluetooth module |
| --- | --- |
| PA26 / UART3 TX | RX |
| PA25 / UART3 RX | TX |
| GND | GND |

The serial format is `115200 8N1`. The Bluetooth module's UART side must use a
3.3 V compatible logic level.

## Run

Open `Launch-SpeedTuner.cmd`, select the Bluetooth serial COM port, and connect.
The GUI has separate speed-loop, position-loop, and Yaw-loop tabs. The position tab edits
Kp, relative target counts, maximum speed, output limit, tolerance, straight-line
sync Kp, and maximum sync correction, and provides Run, Stress 24, and Stop
controls. Its live panel shows position target, actual count, error, current
step, completed moves, worst error, wheel speeds, recovery totals, invalid
transitions, result, and high-impedance state. The GUI is the only process that
owns the physical serial port.

The Yaw tab edits Kp/Ki/Kd, relative target angle, maximum and minimum turn
speed, PWM limit, angle tolerance, settle rate/time, run timeout, and a
pivot-load feedforward boost. The minimum speed compensates motor stiction only
when the vehicle is nearly stationary and still outside tolerance. The boost
adds startup torque while the speed loop is owned by Yaw mode. Its live panel shows
target/current/error angle, Yaw rate, turn target, wheel targets/speeds,
outputs, result, and `HIGH-Z/ARMED`. Start with `-StartYawMode` to select this
tab before auto-connect.

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

During a Yaw move the bridge emits a seven-value FireWater group named `yaw`:

```text
Yaw_Target_mdeg
Yaw_Current_mdeg
Yaw_Error_mdeg
Yaw_Rate_mdps
Turn_Target_pps
Left_Speed_pps
Right_Speed_pps
```

Yaw samples and status are mirrored into `runtime/latest_yaw_wave.json`,
`runtime/latest_yaw_telemetry.csv`, and `runtime/latest_yaw_status.json`.

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
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action YawGet
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action YawSet -YawKp 45 -YawKi 0.8 -YawKd 3 -YawTarget -45 -YawMaxSpeed 2000 -YawMinSpeed 300 -YawBoost 80 -ApplyConfig
powershell -NoProfile -File SpeedTunerCli.ps1 -Port COM6 -Action YawRun
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

imu stat
imu zero

yaw get
yaw set KP KI KD TARGET_DEG MAX_SPEED_PPS OUTPUT_LIMIT_PERMILLE TOLERANCE_DEG SETTLE_RATE_DPS SETTLE_MS TIMEOUT_MS [MIN_SPEED_PPS [FEEDFORWARD_BOOST_PERMILLE]]
yaw run
yaw stop
yaw stat

mission start
mission stop
mission stat

motion start DELTA_COUNTS HEADING_DEG MAX_SPEED_PPS TIMEOUT_MS
motion start DELTA_COUNTS hold MAX_SPEED_PPS TIMEOUT_MS
motion stop
motion stat
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

The final two `yaw set` fields are optional for compatibility with older tools.
When omitted, the current minimum turn speed and Yaw feedforward boost are
preserved. The boost range is `0..300 permille`.

Accepted ranges:

```text
Kp       0 .. 5
Ki       0 .. 20
Kd       0 .. 2
Target   100 .. 6000 pps
Limit    100 .. 1000 permille
Pos sync Kp   0 .. 20 pps/count
Pos sync max  0 .. 6000 pps
Line duration 500 .. 60000 ms
```
