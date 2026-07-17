[CmdletBinding()]
param(
    [string]$Port = "COM6",
    [ValidateSet(
        "Get", "Set", "Run", "Step", "Reverse", "Sweep", "Lease",
        "Stop", "Status", "PositionGet", "PositionSet", "PositionRun",
        "PositionStress", "PositionStop", "PositionStatus",
        "ImuStatus", "ImuZero", "YawGet", "YawSet", "YawRun",
        "YawStop", "YawStatus", "HeadingGet", "HeadingSet",
        "HeadingRun", "HeadingStop", "HeadingStatus", "LineGet",
        "LineSet", "LineRun", "LineStop", "LineStatus", "LineCal")]
    [string]$Action = "Status",
    [switch]$Takeover,
    [switch]$DirectSerial,
    [string]$BridgeHost = "127.0.0.1",
    [uint16]$BridgePort = 13471,
    [switch]$ApplyConfig,
    [single]$Kp = 0.12,
    [single]$Ki = 0.05,
    [single]$Kd = 0.0,
    [single]$Target = 3500.0,
    [uint16]$Limit = 650,
    [single]$PositionKp = 3.0,
    [int32]$PositionTarget = 1060,
    [single]$PositionMaxSpeed = 2000.0,
    [uint16]$PositionLimit = 650,
    [uint16]$PositionTolerance = 24,
    [single]$PositionSyncKp = 2.0,
    [single]$PositionSyncMax = 400.0,
    [single]$YawKp = 45.0,
    [single]$YawKi = 0.8,
    [single]$YawKd = 7.0,
    [single]$YawTarget = -45.0,
    [single]$YawMaxSpeed = 800.0,
    [single]$YawMinSpeed = 200.0,
    [uint16]$YawBoost = 40,
    [uint16]$YawLimit = 750,
    [single]$YawTolerance = 0.7,
    [single]$YawSettleRate = 5.0,
    [uint16]$YawSettleTime = 300,
    [uint16]$YawTimeout = 5000,
    [single]$HeadingKp = 30.0,
    [single]$HeadingKi = 3.0,
    [single]$HeadingKd = 1.5,
    [single]$HeadingBaseSpeed = 1200.0,
    [single]$HeadingMaxCorrection = 400.0,
    [uint16]$HeadingLimit = 650,
    [single]$HeadingDeadband = 0.5,
    [uint16]$HeadingDuration = 6000,
    [single]$LineKp = 12.0,
    [single]$LineKi = 0.0,
    [single]$LineKd = 0.0,
    [single]$LineBaseSpeed = 700.0,
    [single]$LineMaxCorrection = 400.0,
    [uint16]$LineLimit = 500,
    [single]$LineDeadband = 2.0,
    [uint16]$LineDuration = 4000,
    [uint32]$PollMs = 300,
    [uint32]$RunTimeoutMs = 15000
)

$ErrorActionPreference = "Stop"
$baudRate = 115200
$serial = $null
$bridgeClient = $null
$bridgeReader = $null
$bridgeWriter = $null
$transport = "direct_serial"
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$runtimeDir = Join-Path $PSScriptRoot "runtime"
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[void](New-Item -ItemType Directory -Force -Path $runtimeDir)

function Stop-GuiOwner {
    $owners = Get-Process powershell, pwsh -ErrorAction SilentlyContinue |
        Where-Object { $_.MainWindowTitle -eq "CAR Speed Loop Tuner" }
    foreach ($owner in $owners) {
        Write-Host "TAKEOVER closing GUI pid=$($owner.Id)"
        Stop-Process -Id $owner.Id
    }
    if ($owners) {
        Start-Sleep -Milliseconds 300
    }
}

function Connect-Bridge {
    try {
        $script:bridgeClient = New-Object System.Net.Sockets.TcpClient
        $async = $script:bridgeClient.BeginConnect(
            $BridgeHost, [int]$BridgePort, $null, $null)
        if (-not $async.AsyncWaitHandle.WaitOne(500)) {
            Close-Bridge
            return $false
        }
        $script:bridgeClient.EndConnect($async)
        $script:bridgeClient.NoDelay = $true
        $stream = $script:bridgeClient.GetStream()
        $stream.ReadTimeout = 250
        $stream.WriteTimeout = 1500
        $script:bridgeReader = New-Object System.IO.StreamReader(
            $stream, [System.Text.Encoding]::ASCII, $false, 1024, $true)
        $script:bridgeWriter = New-Object System.IO.StreamWriter(
            $stream, [System.Text.Encoding]::ASCII, 1024, $true)
        $script:bridgeWriter.NewLine = "`n"
        $script:bridgeWriter.AutoFlush = $true
        $script:transport = "tcp_bridge"
        return $true
    } catch {
        Close-Bridge
        return $false
    }
}

function Close-Bridge {
    if ($null -ne $script:bridgeReader) {
        try { $script:bridgeReader.Dispose() } catch {}
        $script:bridgeReader = $null
    }
    if ($null -ne $script:bridgeWriter) {
        try { $script:bridgeWriter.Dispose() } catch {}
        $script:bridgeWriter = $null
    }
    if ($null -ne $script:bridgeClient) {
        try { $script:bridgeClient.Close() } catch {}
        try { $script:bridgeClient.Dispose() } catch {}
        $script:bridgeClient = $null
    }
}

function Send-Line([string]$line) {
    if ($transport -eq "tcp_bridge") {
        $bridgeWriter.WriteLine($line)
    } else {
        $serial.Write($line + "`n")
    }
    Write-Host "TX $line"
}

function Read-Expected([string[]]$prefixes, [uint32]$timeoutMs = 2500) {
    $deadline = [DateTime]::Now.AddMilliseconds($timeoutMs)
    while ([DateTime]::Now -lt $deadline) {
        try {
            if ($transport -eq "tcp_bridge") {
                $line = $bridgeReader.ReadLine()
                if ($null -eq $line) {
                    throw "Bridge connection closed."
                }
                $line = $line.Trim()
            } else {
                $line = $serial.ReadLine().Trim()
            }
            if ($line.Length -eq 0) { continue }
            if ($line.StartsWith("wave:") -or
                $line.StartsWith("poswave:") -or
                $line.StartsWith("yawwave:")) { continue }
            Write-Host "RX $line"
            foreach ($prefix in $prefixes) {
                if ($line.StartsWith($prefix)) {
                    return $line
                }
            }
        } catch [System.TimeoutException] {
        } catch [System.IO.IOException] {
            if ($transport -ne "tcp_bridge") { throw }
        }
    }
    throw "Timed out waiting for: $($prefixes -join ', ')"
}

function Invoke-Protocol([string]$command, [string[]]$prefixes) {
    Send-Line $command
    $response = Read-Expected $prefixes
    if ($response.StartsWith("ERR command")) {
        Start-Sleep -Milliseconds 120
        Send-Line $command
        $response = Read-Expected $prefixes
    }
    if ($response.StartsWith("ERR ")) {
        throw "Command '$command' was rejected: $response"
    }
    return $response
}

function Assert-ConfigRange {
    if (($Kp -lt 0) -or ($Kp -gt 5) -or
        ($Ki -lt 0) -or ($Ki -gt 20) -or
        ($Kd -lt 0) -or ($Kd -gt 2) -or
        ($Target -lt 100) -or ($Target -gt 6000) -or
        ($Limit -lt 100) -or ($Limit -gt 1000)) {
        throw "Configuration is outside firmware range."
    }
}

function Get-SetCommand {
    Assert-ConfigRange
    return ('spd set {0} {1} {2} {3} {4}' -f
        $Kp.ToString('0.####', $culture),
        $Ki.ToString('0.####', $culture),
        $Kd.ToString('0.####', $culture),
        $Target.ToString('0.####', $culture),
        $Limit.ToString($culture))
}

function Get-PositionSetCommand {
    if (($PositionKp -le 0) -or ($PositionKp -gt 20) -or
        ($PositionTarget -eq 0) -or
        ($PositionTarget -lt -100000) -or
        ($PositionTarget -gt 100000) -or
        ($PositionMaxSpeed -lt 100) -or ($PositionMaxSpeed -gt 6000) -or
        ($PositionLimit -lt 100) -or ($PositionLimit -gt 1000) -or
        ($PositionTolerance -lt 1) -or ($PositionTolerance -gt 200) -or
        ($PositionSyncKp -lt 0) -or ($PositionSyncKp -gt 20) -or
        ($PositionSyncMax -lt 0) -or ($PositionSyncMax -gt 6000)) {
        throw "Position configuration is outside firmware range."
    }
    return ('pos set {0} {1} {2} {3} {4} {5} {6}' -f
        $PositionKp.ToString('0.####', $culture),
        $PositionTarget.ToString($culture),
        $PositionMaxSpeed.ToString('0.####', $culture),
        $PositionLimit.ToString($culture),
        $PositionTolerance.ToString($culture),
        $PositionSyncKp.ToString('0.####', $culture),
        $PositionSyncMax.ToString('0.####', $culture))
}

function Get-YawSetCommand {
    if (($YawKp -le 0) -or ($YawKp -gt 50) -or
        ($YawKi -lt 0) -or ($YawKi -gt 20) -or
        ($YawKd -lt 0) -or ($YawKd -gt 20) -or
        ($YawTarget -eq 0) -or
        ($YawTarget -lt -180) -or ($YawTarget -gt 180) -or
        ($YawMaxSpeed -lt 100) -or ($YawMaxSpeed -gt 6000) -or
        ($YawMinSpeed -lt 0) -or ($YawMinSpeed -gt $YawMaxSpeed) -or
        ($YawBoost -gt 300) -or
        ($YawLimit -lt 100) -or ($YawLimit -gt 1000) -or
        ($YawTolerance -lt 0.1) -or ($YawTolerance -gt 15) -or
        ($YawSettleRate -lt 0.1) -or ($YawSettleRate -gt 50) -or
        ($YawSettleTime -lt 50) -or ($YawSettleTime -gt 2000) -or
        ($YawTimeout -lt 500) -or ($YawTimeout -gt 15000)) {
        throw "Yaw configuration is outside firmware range."
    }
    return ('yaw set {0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11}' -f
        $YawKp.ToString('0.####', $culture),
        $YawKi.ToString('0.####', $culture),
        $YawKd.ToString('0.####', $culture),
        $YawTarget.ToString('0.####', $culture),
        $YawMaxSpeed.ToString('0.####', $culture),
        $YawLimit.ToString($culture),
        $YawTolerance.ToString('0.####', $culture),
        $YawSettleRate.ToString('0.####', $culture),
        $YawSettleTime.ToString($culture),
        $YawTimeout.ToString($culture),
        $YawMinSpeed.ToString('0.####', $culture),
        $YawBoost.ToString($culture))
}

function Get-HeadingSetCommand {
    if (($HeadingKp -le 0) -or ($HeadingKp -gt 50) -or
        ($HeadingKi -lt 0) -or ($HeadingKi -gt 20) -or
        ($HeadingKd -lt 0) -or ($HeadingKd -gt 20) -or
        ([Math]::Abs($HeadingBaseSpeed) -lt 100) -or
        (([Math]::Abs($HeadingBaseSpeed) + $HeadingMaxCorrection) -gt 6000) -or
        ($HeadingMaxCorrection -lt 10) -or
        ($HeadingLimit -lt 100) -or ($HeadingLimit -gt 1000) -or
        ($HeadingDeadband -lt 0) -or ($HeadingDeadband -gt 15) -or
        ($HeadingDuration -lt 500) -or ($HeadingDuration -gt 10000)) {
        throw "Heading configuration is outside firmware range."
    }
    return ('heading set {0} {1} {2} {3} {4} {5} {6} {7}' -f
        $HeadingKp.ToString('0.####', $culture),
        $HeadingKi.ToString('0.####', $culture),
        $HeadingKd.ToString('0.####', $culture),
        $HeadingBaseSpeed.ToString('0.####', $culture),
        $HeadingMaxCorrection.ToString('0.####', $culture),
        $HeadingLimit.ToString($culture),
        $HeadingDeadband.ToString('0.####', $culture),
        $HeadingDuration.ToString($culture))
}

function Get-LineSetCommand {
    if (($LineKp -le 0) -or ($LineKp -gt 100) -or
        ($LineKi -lt 0) -or ($LineKi -gt 100) -or
        ($LineKd -lt 0) -or ($LineKd -gt 20) -or
        ($LineBaseSpeed -lt 100) -or
        (($LineBaseSpeed + $LineMaxCorrection) -gt 6000) -or
        ($LineMaxCorrection -le 0) -or
        ($LineLimit -lt 100) -or ($LineLimit -gt 1000) -or
        ($LineDeadband -lt 0) -or ($LineDeadband -gt 20) -or
        ($LineDuration -lt 500) -or ($LineDuration -gt 60000)) {
        throw "Line-tracking configuration is outside firmware range."
    }
    return ('line set {0} {1} {2} {3} {4} {5} {6} {7}' -f
        $LineKp.ToString('0.####', $culture),
        $LineKi.ToString('0.####', $culture),
        $LineKd.ToString('0.####', $culture),
        $LineBaseSpeed.ToString('0.####', $culture),
        $LineMaxCorrection.ToString('0.####', $culture),
        $LineLimit.ToString($culture),
        $LineDeadband.ToString('0.####', $culture),
        $LineDuration.ToString($culture))
}

function Parse-Status([string]$line) {
    $values = @{}
    foreach ($match in [System.Text.RegularExpressions.Regex]::Matches(
        $line, '(?<key>[A-Za-z][A-Za-z0-9]*)=(?<value>[^\s]+)')) {
        $values[$match.Groups['key'].Value] = $match.Groups['value'].Value
    }
    return $values
}

function Save-DirectStatus([hashtable]$values, [string]$csvPath) {
    $timestamp = [DateTime]::Now.ToString("o")
    $status = [ordered]@{
        timestamp = $timestamp
        source = $transport
        port = if ($transport -eq "tcp_bridge") {
            "$BridgeHost`:$BridgePort"
        } else { $Port }
        baud = $baudRate
        state = $values['state']
        left_speed_pps = [int]$values['left']
        right_speed_pps = [int]$values['right']
        left_output_permille = [int]$values['outL']
        right_output_permille = [int]$values['outR']
        invalid_left = [uint32]$values['invL']
        invalid_right = [uint32]$values['invR']
        result = [uint32]$values['res']
        high_impedance = ($values['hz'] -eq '1')
    }
    $jsonPath = Join-Path $runtimeDir "latest_direct_status.json"
    [System.IO.File]::WriteAllText(
        $jsonPath, ($status | ConvertTo-Json), $utf8NoBom)

    if ($csvPath -ne "") {
        $row = @(
            $timestamp, $values['state'], $values['left'], $values['right'],
            $values['outL'], $values['outR'], $values['invL'], $values['invR'],
            $values['res'], $values['hz']) -join ','
        [System.IO.File]::AppendAllText(
            $csvPath, "$row`r`n", $utf8NoBom)
        [System.IO.File]::AppendAllText(
            (Join-Path $runtimeDir "latest_direct_run.csv"),
            "$row`r`n", $utf8NoBom)
    }
}

function Save-YawStatus([hashtable]$values, [string]$csvPath) {
    $timestamp = [DateTime]::Now.ToString("o")
    $status = [ordered]@{
        timestamp = $timestamp
        source = $transport
        port = if ($transport -eq "tcp_bridge") {
            "$BridgeHost`:$BridgePort"
        } else { $Port }
        state = $values['state']
        target_mdeg = [int]$values['target']
        current_mdeg = [int]$values['current']
        error_mdeg = [int]$values['error']
        yaw_rate_mdps = [int]$values['rate']
        turn_target_pps = [int]$values['turn']
        left_target_pps = [int]$values['tL']
        right_target_pps = [int]$values['tR']
        left_speed_pps = [int]$values['vL']
        right_speed_pps = [int]$values['vR']
        left_output_permille = [int]$values['outL']
        right_output_permille = [int]$values['outR']
        result = [uint32]$values['res']
        high_impedance = ($values['hz'] -eq '1')
        loop_interval_ms = [uint32]$values['loop']
        loop_max_interval_ms = [uint32]$values['loopMax']
        imu_interval_ms = [uint32]$values['imuDt']
        imu_max_interval_ms = [uint32]$values['imuMax']
        yaw_interval_ms = [uint32]$values['yawDt']
        yaw_max_interval_ms = [uint32]$values['yawMax']
        lcd_duration_ms = [uint32]$values['lcd']
        lcd_max_duration_ms = [uint32]$values['lcdMax']
    }
    [System.IO.File]::WriteAllText(
        (Join-Path $runtimeDir "latest_yaw_status.json"),
        ($status | ConvertTo-Json), $utf8NoBom)

    if ($csvPath -ne "") {
        $row = @(
            $timestamp, $values['state'], $values['target'],
            $values['current'], $values['error'], $values['rate'],
            $values['turn'], $values['tL'], $values['tR'],
            $values['vL'], $values['vR'], $values['outL'],
            $values['outR'], $values['res'], $values['hz'],
            $values['loop'], $values['loopMax'], $values['imuDt'],
            $values['imuMax'], $values['yawDt'], $values['yawMax'],
            $values['lcd'], $values['lcdMax']) -join ','
        [System.IO.File]::AppendAllText(
            $csvPath, "$row`r`n", $utf8NoBom)
        if ($transport -ne "tcp_bridge") {
            [System.IO.File]::AppendAllText(
                (Join-Path $runtimeDir "latest_yaw_telemetry.csv"),
                "$row`r`n", $utf8NoBom)
        }
    }
}

function Save-HeadingStatus([hashtable]$values, [string]$csvPath) {
    $timestamp = [DateTime]::Now.ToString("o")
    $status = [ordered]@{
        timestamp = $timestamp
        source = $transport
        port = if ($transport -eq "tcp_bridge") {
            "$BridgeHost`:$BridgePort"
        } else { $Port }
        state = $values['state']
        target_mdeg = [int]$values['target']
        current_mdeg = [int]$values['current']
        error_mdeg = [int]$values['error']
        yaw_rate_mdps = [int]$values['rate']
        base_speed_pps = [int]$values['base']
        correction_pps = [int]$values['corr']
        left_target_pps = [int]$values['tL']
        right_target_pps = [int]$values['tR']
        left_speed_pps = [int]$values['vL']
        right_speed_pps = [int]$values['vR']
        left_output_permille = [int]$values['outL']
        right_output_permille = [int]$values['outR']
        result = [uint32]$values['res']
        high_impedance = ($values['hz'] -eq '1')
        loop_max_interval_ms = [uint32]$values['loopMax']
        imu_max_interval_ms = [uint32]$values['imuMax']
        heading_interval_ms = [uint32]$values['headDt']
        heading_max_interval_ms = [uint32]$values['headMax']
        lcd_max_duration_ms = [uint32]$values['lcdMax']
    }
    [System.IO.File]::WriteAllText(
        (Join-Path $runtimeDir "latest_heading_status.json"),
        ($status | ConvertTo-Json), $utf8NoBom)

    if ($csvPath -ne "") {
        $row = @(
            $timestamp, $values['state'], $values['target'],
            $values['current'], $values['error'], $values['rate'],
            $values['base'], $values['corr'], $values['tL'],
            $values['tR'], $values['vL'], $values['vR'],
            $values['outL'], $values['outR'], $values['res'],
            $values['hz'], $values['loopMax'], $values['imuMax'],
            $values['headDt'], $values['headMax'],
            $values['lcdMax']) -join ','
        [System.IO.File]::AppendAllText(
            $csvPath, "$row`r`n", $utf8NoBom)
        [System.IO.File]::AppendAllText(
            (Join-Path $runtimeDir "latest_heading_telemetry.csv"),
            "$row`r`n", $utf8NoBom)
    }
}

function Save-LineStatus([hashtable]$values, [string]$csvPath = "") {
    $timestamp = [DateTime]::Now.ToString("o")
    $status = [ordered]@{
        timestamp = $timestamp
        source = $transport
        port = if ($transport -eq "tcp_bridge") {
            "$BridgeHost`:$BridgePort"
        } else { $Port }
        state = $values['state']
        sensor_state = $values['sensor']
        raw = [uint32]$values['raw']
        active_mask = [uint32]$values['mask']
        active_count = [uint32]$values['count']
        line_error = [int]$values['error']
        line_seen = ($values['seen'] -eq '1')
        sample_count = [uint32]$values['samples']
        error_count = [uint32]$values['errors']
        calibration_count = [uint32]$values['cal']
        sample_age_ms = [uint32]$values['age']
        bus_transaction_count = [uint32]$values['busTx']
        bus_recovery_count = [uint32]$values['busRec']
        bus_result = [uint32]$values['busRes']
        base_speed_pps = [int]$values['base']
        correction_pps = [int]$values['corr']
        target_yaw_rate_mdps = [int]$values['yawT']
        measured_yaw_rate_mdps = [int]$values['yawR']
        yaw_rate_boost_pps = [int]$values['yawBoost']
        imu_feedback_valid = ($values['imu'] -eq '1')
        left_target_pps = [int]$values['tL']
        right_target_pps = [int]$values['tR']
        left_speed_pps = [int]$values['vL']
        right_speed_pps = [int]$values['vR']
        left_output_permille = [int]$values['outL']
        right_output_permille = [int]$values['outR']
        result = [uint32]$values['res']
        sensor_result = [uint32]$values['sensorRes']
        high_impedance = ($values['hz'] -eq '1')
        line_interval_ms = [uint32]$values['lineDt']
        line_max_interval_ms = [uint32]$values['lineMax']
        loop_max_interval_ms = [uint32]$values['loopMax']
        lcd_max_duration_ms = [uint32]$values['lcdMax']
    }
    [System.IO.File]::WriteAllText(
        (Join-Path $runtimeDir "latest_line_status.json"),
        ($status | ConvertTo-Json), $utf8NoBom)

    if ($csvPath -ne "") {
        $row = @(
            $timestamp, $values['state'], $values['raw'], $values['mask'],
             $values['count'], $values['error'], $values['seen'],
             $values['base'], $values['corr'], $values['yawT'],
             $values['yawR'], $values['yawBoost'], $values['imu'],
             $values['tL'],
            $values['tR'], $values['vL'], $values['vR'],
            $values['outL'], $values['outR'], $values['res'],
            $values['hz'], $values['age'], $values['lineDt'],
            $values['lineMax'], $values['loopMax'], $values['lcdMax']) -join ','
        [System.IO.File]::AppendAllText(
            $csvPath, "$row`r`n", $utf8NoBom)
        if ($transport -ne "tcp_bridge") {
            [System.IO.File]::AppendAllText(
                (Join-Path $runtimeDir "latest_line_telemetry.csv"),
                "$row`r`n", $utf8NoBom)
        }
    }
}

if ($Takeover) {
    Stop-GuiOwner
}

try {
    $runCommand = $null
    $yawRunRequested = $false
    $yawRunActive = $false
    $headingRunRequested = $false
    $headingRunActive = $false
    $lineRunRequested = $false
    $lineRunActive = $false
    $bridgeConnected = $false
    if ((-not $Takeover) -and (-not $DirectSerial)) {
        $bridgeConnected = Connect-Bridge
    }
    if ($bridgeConnected) {
        Write-Host "OPEN bridge=$BridgeHost`:$BridgePort"
    } else {
        $transport = "direct_serial"
        $serial = New-Object System.IO.Ports.SerialPort(
            $Port, $baudRate, [System.IO.Ports.Parity]::None, 8,
            [System.IO.Ports.StopBits]::One)
        $serial.Encoding = [System.Text.Encoding]::ASCII
        $serial.DtrEnable = $false
        $serial.RtsEnable = $false
        $serial.NewLine = "`n"
        $serial.ReadTimeout = 250
        $serial.WriteTimeout = 1500
        $serial.Open()
        Write-Host "OPEN port=$Port baud=$baudRate"
        [System.IO.File]::WriteAllText(
            (Join-Path $runtimeDir "last_port.txt"),
            $Port, [System.Text.Encoding]::ASCII)
        Start-Sleep -Milliseconds 800
        $serial.DiscardInBuffer()
        $serial.Write("`n")
        Start-Sleep -Milliseconds 180
        $serial.DiscardInBuffer()
        $serial.DiscardOutBuffer()
    }

    switch ($Action) {
        "Get" {
            [void](Invoke-Protocol "spd get" @("OK CFG ", "ERR "))
        }
        "Set" {
            [void](Invoke-Protocol (Get-SetCommand) @("OK CFG ", "ERR "))
        }
        "Stop" {
            [void](Invoke-Protocol "spd stop" @("OK STOP", "ERR "))
            Start-Sleep -Milliseconds 200
            $statusLine = Invoke-Protocol "spd stat" @("STAT ", "ERR ")
            if ($statusLine.StartsWith("STAT ")) {
                Save-DirectStatus (Parse-Status $statusLine) ""
            }
        }
        "Status" {
            $statusLine = Invoke-Protocol "spd stat" @("STAT ", "ERR ")
            if ($statusLine.StartsWith("STAT ")) {
                Save-DirectStatus (Parse-Status $statusLine) ""
            }
        }
        "PositionGet" {
            [void](Invoke-Protocol "pos get" @("OK PCFG ", "ERR "))
        }
        "PositionSet" {
            [void](Invoke-Protocol (Get-PositionSetCommand) @("OK PCFG ", "ERR "))
        }
        "PositionRun" {
            [void](Invoke-Protocol "pos run" @("OK POS RUN SINGLE", "ERR "))
        }
        "PositionStress" {
            [void](Invoke-Protocol "pos run stress" @("OK POS RUN STRESS", "ERR "))
        }
        "PositionStop" {
            [void](Invoke-Protocol "pos stop" @("OK POS STOP", "ERR "))
        }
        "PositionStatus" {
            [void](Invoke-Protocol "pos stat" @("PSTAT ", "ERR "))
        }
        "ImuStatus" {
            [void](Invoke-Protocol "imu stat" @("ISTAT ", "ERR "))
        }
        "ImuZero" {
            [void](Invoke-Protocol "imu zero" @("OK IMU ZERO", "ERR "))
        }
        "YawGet" {
            [void](Invoke-Protocol "yaw get" @("OK YCFG ", "ERR "))
        }
        "YawSet" {
            [void](Invoke-Protocol (Get-YawSetCommand) @("OK YCFG ", "ERR "))
        }
        "YawRun" {
            if ($ApplyConfig) {
                [void](Invoke-Protocol (Get-YawSetCommand) @("OK YCFG ", "ERR "))
            } else {
                [void](Invoke-Protocol "yaw get" @("OK YCFG ", "ERR "))
            }
            [void](Invoke-Protocol "yaw run" @("OK YAW RUN", "ERR "))
            $yawRunRequested = $true
            $yawRunActive = $true
        }
        "YawStop" {
            [void](Invoke-Protocol "yaw stop" @("OK YAW STOP", "ERR "))
        }
        "YawStatus" {
            [void](Invoke-Protocol "yaw stat" @("YSTAT ", "ERR "))
        }
        "HeadingGet" {
            [void](Invoke-Protocol "heading get" @("OK HCFG ", "ERR "))
        }
        "HeadingSet" {
            [void](Invoke-Protocol (Get-HeadingSetCommand) @("OK HCFG ", "ERR "))
        }
        "HeadingRun" {
            if ($ApplyConfig) {
                [void](Invoke-Protocol (Get-HeadingSetCommand) @("OK HCFG ", "ERR "))
            } else {
                [void](Invoke-Protocol "heading get" @("OK HCFG ", "ERR "))
            }
            [void](Invoke-Protocol "heading run" @("OK HEADING RUN", "ERR "))
            $headingRunRequested = $true
            $headingRunActive = $true
        }
        "HeadingStop" {
            [void](Invoke-Protocol "heading stop" @("OK HEADING STOP", "ERR "))
        }
        "HeadingStatus" {
            [void](Invoke-Protocol "heading stat" @("HSTAT ", "ERR "))
        }
        "LineGet" {
            [void](Invoke-Protocol "line get" @("OK LCFG ", "ERR "))
        }
        "LineSet" {
            [void](Invoke-Protocol (Get-LineSetCommand) @("OK LCFG ", "ERR "))
        }
        "LineRun" {
            if ($ApplyConfig) {
                [void](Invoke-Protocol (Get-LineSetCommand) @("OK LCFG ", "ERR "))
            } else {
                [void](Invoke-Protocol "line get" @("OK LCFG ", "ERR "))
            }
            [void](Invoke-Protocol "line run" @("OK LINE RUN", "ERR "))
            $lineRunRequested = $true
            $lineRunActive = $true
        }
        "LineStop" {
            [void](Invoke-Protocol "line stop" @("OK LINE STOP", "ERR "))
        }
        "LineStatus" {
            $response = Invoke-Protocol "line stat" @("LSTAT ", "ERR ")
            if ($response.StartsWith("LSTAT ")) {
                Save-LineStatus (Parse-Status $response)
            }
        }
        "LineCal" {
            [void](Invoke-Protocol "line cal" @("OK LINE CAL", "ERR "))
        }
        "Run" {
            $runCommand = "spd run"
        }
        "Step" {
            $runCommand = "spd run step"
        }
        "Reverse" {
            $runCommand = "spd run reverse"
        }
        "Sweep" {
            $runCommand = "spd run sweep"
        }
        "Lease" {
            $runCommand = "spd run lease"
        }
    }

    if ($null -ne $runCommand) {
            if ($ApplyConfig) {
                [void](Invoke-Protocol (Get-SetCommand) @("OK CFG ", "ERR "))
            } else {
                [void](Invoke-Protocol "spd get" @("OK CFG ", "ERR "))
            }
            [void](Invoke-Protocol $runCommand @("OK RUN", "ERR "))

            $stamp = [DateTime]::Now.ToString("yyyyMMdd_HHmmss")
            $profileName = $Action.ToLowerInvariant()
            $csvPath = Join-Path $runtimeDir (
                "direct_${profileName}_$stamp.csv")
            $latestCsvPath = Join-Path $runtimeDir "latest_direct_run.csv"
            $header = "timestamp,state,left_pps,right_pps,left_output,right_output,invalid_left,invalid_right,result,high_z`r`n"
            [System.IO.File]::WriteAllText($csvPath, $header, $utf8NoBom)
            [System.IO.File]::WriteAllText($latestCsvPath, $header, $utf8NoBom)

            $deadline = [DateTime]::Now.AddMilliseconds($RunTimeoutMs)
            $finished = $false
            while ([DateTime]::Now -lt $deadline) {
                Start-Sleep -Milliseconds $PollMs
                $statusLine = Invoke-Protocol "spd stat" @("STAT ", "ERR ")
                if (-not $statusLine.StartsWith("STAT ")) { continue }
                $values = Parse-Status $statusLine
                Save-DirectStatus $values $csvPath
                if ($values['state'] -in @('DONE', 'ABORT', 'LOCKED')) {
                    $finished = $true
                    break
                }
            }
            if (-not $finished) {
                [void](Invoke-Protocol "spd stop" @("OK STOP", "ERR "))
                throw "Run did not reach a terminal state before timeout."
            }
            Write-Host "CAPTURE $csvPath"
    }

    if ($yawRunRequested) {
        $stamp = [DateTime]::Now.ToString("yyyyMMdd_HHmmss")
        $csvPath = Join-Path $runtimeDir "yaw_run_$stamp.csv"
        $latestCsvPath = Join-Path $runtimeDir "latest_yaw_telemetry.csv"
        $header = "timestamp,state,target_mdeg,current_mdeg,error_mdeg,yaw_rate_mdps,turn_target_pps,left_target_pps,right_target_pps,left_speed_pps,right_speed_pps,left_output_permille,right_output_permille,result,high_z,loop_interval_ms,loop_max_interval_ms,imu_interval_ms,imu_max_interval_ms,yaw_interval_ms,yaw_max_interval_ms,lcd_duration_ms,lcd_max_duration_ms`r`n"
        [System.IO.File]::WriteAllText($csvPath, $header, $utf8NoBom)
        if ($transport -ne "tcp_bridge") {
            [System.IO.File]::WriteAllText(
                $latestCsvPath, $header, $utf8NoBom)
        }

        $deadline = [DateTime]::Now.AddMilliseconds($RunTimeoutMs)
        $acceptedAt = [DateTime]::Now
        $finished = $false
        $observedRunning = $false
        $terminalState = $null
        $terminalResult = $null
        $terminalHighZ = $null
        while ([DateTime]::Now -lt $deadline) {
            Start-Sleep -Milliseconds $PollMs
            $statusLine = Invoke-Protocol "yaw stat" @("YSTAT ", "ERR ")
            if (-not $statusLine.StartsWith("YSTAT ")) { continue }
            $values = Parse-Status $statusLine
            Save-YawStatus $values $csvPath
            if ($values['state'] -eq 'RUN') {
                $observedRunning = $true
                continue
            }
            if ($observedRunning -and
                ($values['state'] -in @('DONE', 'ABORT', 'LOCKED'))) {
                $finished = $true
                $terminalState = $values['state']
                $terminalResult = $values['res']
                $terminalHighZ = $values['hz']
                $yawRunActive = $false
                break
            }
            if ((-not $observedRunning) -and
                (([DateTime]::Now - $acceptedAt).TotalMilliseconds -ge 2000) -and
                ($values['state'] -in @('ABORT', 'LOCKED'))) {
                $finished = $true
                $terminalState = $values['state']
                $terminalResult = $values['res']
                $terminalHighZ = $values['hz']
                $yawRunActive = $false
                break
            }
        }
        if (-not $finished) {
            [void](Invoke-Protocol "yaw stop" @("OK YAW STOP", "ERR "))
            $yawRunActive = $false
            throw "Yaw run did not reach a terminal state before timeout."
        }
        Write-Host "CAPTURE $csvPath"
        if ($terminalState -ne 'DONE') {
            throw "Yaw run ended in state=$terminalState result=$terminalResult high_z=$terminalHighZ."
        }
    }

    if ($headingRunRequested) {
        $stamp = [DateTime]::Now.ToString("yyyyMMdd_HHmmss")
        $csvPath = Join-Path $runtimeDir "heading_run_$stamp.csv"
        $latestCsvPath = Join-Path $runtimeDir "latest_heading_telemetry.csv"
        $header = "timestamp,state,target_mdeg,current_mdeg,error_mdeg,yaw_rate_mdps,base_speed_pps,correction_pps,left_target_pps,right_target_pps,left_speed_pps,right_speed_pps,left_output_permille,right_output_permille,result,high_z,loop_max_interval_ms,imu_max_interval_ms,heading_interval_ms,heading_max_interval_ms,lcd_max_duration_ms`r`n"
        [System.IO.File]::WriteAllText($csvPath, $header, $utf8NoBom)
        [System.IO.File]::WriteAllText($latestCsvPath, $header, $utf8NoBom)

        $deadline = [DateTime]::Now.AddMilliseconds($RunTimeoutMs)
        $finished = $false
        $observedRunning = $false
        $terminalState = $null
        $terminalResult = $null
        $terminalHighZ = $null
        while ([DateTime]::Now -lt $deadline) {
            Start-Sleep -Milliseconds $PollMs
            $statusLine = Invoke-Protocol "heading stat" @("HSTAT ", "ERR ")
            if (-not $statusLine.StartsWith("HSTAT ")) { continue }
            $values = Parse-Status $statusLine
            Save-HeadingStatus $values $csvPath
            if ($values['state'] -eq 'RUN') {
                $observedRunning = $true
                continue
            }
            if ($observedRunning -and
                ($values['state'] -in @('DONE', 'ABORT', 'LOCKED'))) {
                $finished = $true
                $terminalState = $values['state']
                $terminalResult = $values['res']
                $terminalHighZ = $values['hz']
                $headingRunActive = $false
                break
            }
        }
        if (-not $finished) {
            [void](Invoke-Protocol "heading stop" @("OK HEADING STOP", "ERR "))
            $headingRunActive = $false
            throw "Heading run did not reach a terminal state before timeout."
        }
        Write-Host "CAPTURE $csvPath"
        if (($terminalState -ne 'DONE') -or ($terminalResult -ne '0') -or
            ($terminalHighZ -ne '1')) {
            throw "Heading run ended in state=$terminalState result=$terminalResult high_z=$terminalHighZ."
        }
    }

    if ($lineRunRequested) {
        $stamp = [DateTime]::Now.ToString("yyyyMMdd_HHmmss")
        $csvPath = Join-Path $runtimeDir "line_run_$stamp.csv"
        $latestCsvPath = Join-Path $runtimeDir "latest_line_telemetry.csv"
        $header = "timestamp,state,raw,mask,count,line_error,line_seen,base_speed_pps,correction_pps,target_yaw_rate_mdps,measured_yaw_rate_mdps,yaw_rate_boost_pps,imu_feedback_valid,left_target_pps,right_target_pps,left_speed_pps,right_speed_pps,left_output_permille,right_output_permille,result,high_z,sample_age_ms,line_interval_ms,line_max_interval_ms,loop_max_interval_ms,lcd_max_duration_ms`r`n"
        [System.IO.File]::WriteAllText($csvPath, $header, $utf8NoBom)
        if ($transport -ne "tcp_bridge") {
            [System.IO.File]::WriteAllText(
                $latestCsvPath, $header, $utf8NoBom)
        }

        $deadline = [DateTime]::Now.AddMilliseconds($RunTimeoutMs)
        $finished = $false
        $observedRunning = $false
        $terminalState = $null
        $terminalResult = $null
        $terminalHighZ = $null
        while ([DateTime]::Now -lt $deadline) {
            Start-Sleep -Milliseconds $PollMs
            $statusLine = Invoke-Protocol "line stat" @("LSTAT ", "ERR ")
            if (-not $statusLine.StartsWith("LSTAT ")) { continue }
            $values = Parse-Status $statusLine
            Save-LineStatus $values $csvPath
            if ($values['state'] -eq 'RUN') {
                $observedRunning = $true
                continue
            }
            if (($observedRunning -and
                    ($values['state'] -in @('DONE', 'ABORT', 'LOCKED'))) -or
                ((-not $observedRunning) -and
                    ($values['state'] -in @('ABORT', 'LOCKED')))) {
                $finished = $true
                $terminalState = $values['state']
                $terminalResult = $values['res']
                $terminalHighZ = $values['hz']
                $lineRunActive = $false
                break
            }
        }
        if (-not $finished) {
            [void](Invoke-Protocol "line stop" @("OK LINE STOP", "ERR "))
            $lineRunActive = $false
            throw "Line-tracking run did not reach a terminal state before timeout."
        }
        Write-Host "CAPTURE $csvPath"
        if (($terminalState -ne 'DONE') -or ($terminalResult -ne '0') -or
            ($terminalHighZ -ne '1')) {
            throw "Line run ended in state=$terminalState result=$terminalResult high_z=$terminalHighZ."
        }
    }
} finally {
    if ($yawRunActive) {
        try {
            [void](Invoke-Protocol "yaw stop" @("OK YAW STOP", "ERR "))
        } catch {
        }
    }
    if ($headingRunActive) {
        try {
            [void](Invoke-Protocol "heading stop" @("OK HEADING STOP", "ERR "))
        } catch {
        }
    }
    if ($lineRunActive) {
        try {
            [void](Invoke-Protocol "line stop" @("OK LINE STOP", "ERR "))
        } catch {
        }
    }
    Close-Bridge
    if ($null -ne $serial) {
        if ($serial.IsOpen) { $serial.Close() }
        $serial.Dispose()
    }
}
