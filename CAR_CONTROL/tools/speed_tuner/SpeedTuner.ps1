[CmdletBinding()]
param(
    [switch]$AutoConnect,
    [switch]$StartMinimized,
    [switch]$StartYawMode,
    [switch]$StartLineMode
)

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

[System.Windows.Forms.Application]::EnableVisualStyles()

$script:serial = $null
$script:rxBuffer = ""
$script:lastStatusPoll = [DateTime]::MinValue
$script:lastExternalCommandAt = [DateTime]::MinValue
$script:connectionReadyAt = [DateTime]::MinValue
$script:initialReadPending = $false
$script:primePending = $false
$script:activeMode = "speed"
$script:autoReconnect = $AutoConnect.IsPresent
$script:nextAutoConnectAt = [DateTime]::MinValue
$script:baudRate = 115200
$script:captureActive = $false
$script:runCsvPath = $null
$script:vofaPort = 13470
$script:controlPort = 13471
$script:vofaListener = $null
$script:controlListener = $null
$script:vofaClients = New-Object System.Collections.ArrayList
$script:controlClients = New-Object System.Collections.ArrayList
$script:utf8NoBom = New-Object System.Text.UTF8Encoding($false)
$script:runtimeDir = Join-Path $PSScriptRoot "runtime"
[void](New-Item -ItemType Directory -Force -Path $script:runtimeDir)
$script:latestStatusPath = Join-Path $script:runtimeDir "latest_status.json"
$script:latestRunPath = Join-Path $script:runtimeDir "latest_run.csv"
$script:latestWavePath = Join-Path $script:runtimeDir "latest_wave.json"
$script:latestTelemetryPath = Join-Path $script:runtimeDir "latest_telemetry.csv"
$script:latestPositionWavePath = Join-Path $script:runtimeDir "latest_position_wave.json"
$script:latestPositionStatusPath = Join-Path $script:runtimeDir "latest_position_status.json"
$script:latestPositionTelemetryPath = Join-Path $script:runtimeDir "latest_position_telemetry.csv"
$script:latestYawWavePath = Join-Path $script:runtimeDir "latest_yaw_wave.json"
$script:latestYawStatusPath = Join-Path $script:runtimeDir "latest_yaw_status.json"
$script:latestYawTelemetryPath = Join-Path $script:runtimeDir "latest_yaw_telemetry.csv"
$script:latestLineWavePath = Join-Path $script:runtimeDir "latest_line_wave.json"
$script:latestLineStatusPath = Join-Path $script:runtimeDir "latest_line_status.json"
$script:latestMissionStatusPath = Join-Path $script:runtimeDir "latest_mission_status.json"
$script:latestLineTelemetryPath = Join-Path $script:runtimeDir "latest_line_telemetry.csv"
$script:lineStatusSource = 'mission'
$script:lastPortPath = Join-Path $script:runtimeDir "last_port.txt"
$script:sessionLogPath = Join-Path $script:runtimeDir (
    "session_{0}.log" -f [DateTime]::Now.ToString("yyyyMMdd_HHmmss"))
$script:telemetryCsvPath = Join-Path $script:runtimeDir (
    "telemetry_{0}.csv" -f [DateTime]::Now.ToString("yyyyMMdd_HHmmss"))
$script:positionTelemetryCsvPath = Join-Path $script:runtimeDir (
    "position_telemetry_{0}.csv" -f [DateTime]::Now.ToString("yyyyMMdd_HHmmss"))
$script:yawTelemetryCsvPath = Join-Path $script:runtimeDir (
    "yaw_telemetry_{0}.csv" -f [DateTime]::Now.ToString("yyyyMMdd_HHmmss"))
$script:lineTelemetryCsvPath = Join-Path $script:runtimeDir (
    "line_telemetry_{0}.csv" -f [DateTime]::Now.ToString("yyyyMMdd_HHmmss"))
$telemetryHeader = "timestamp,left_target_pps,left_speed_pps,right_target_pps,right_speed_pps,left_output_permille,right_output_permille`r`n"
$positionTelemetryHeader = "timestamp,left_target_count,left_count,right_target_count,right_count,left_speed_target_pps,right_speed_target_pps`r`n"
$yawTelemetryHeader = "timestamp,target_mdeg,current_mdeg,error_mdeg,yaw_rate_mdps,turn_target_pps,left_speed_pps,right_speed_pps`r`n"
$lineTelemetryHeader = "timestamp,line_error,correction_pps,left_target_pps,left_speed_pps,right_target_pps,right_speed_pps,active_count`r`n"
[System.IO.File]::WriteAllText(
    $script:telemetryCsvPath, $telemetryHeader, $script:utf8NoBom)
[System.IO.File]::WriteAllText(
    $script:latestTelemetryPath, $telemetryHeader, $script:utf8NoBom)
[System.IO.File]::WriteAllText(
    $script:positionTelemetryCsvPath,
    $positionTelemetryHeader, $script:utf8NoBom)
[System.IO.File]::WriteAllText(
    $script:latestPositionTelemetryPath,
    $positionTelemetryHeader, $script:utf8NoBom)
[System.IO.File]::WriteAllText(
    $script:yawTelemetryCsvPath,
    $yawTelemetryHeader, $script:utf8NoBom)
[System.IO.File]::WriteAllText(
    $script:latestYawTelemetryPath,
    $yawTelemetryHeader, $script:utf8NoBom)
[System.IO.File]::WriteAllText(
    $script:lineTelemetryCsvPath,
    $lineTelemetryHeader, $script:utf8NoBom)
[System.IO.File]::WriteAllText(
    $script:latestLineTelemetryPath,
    $lineTelemetryHeader, $script:utf8NoBom)

$form = New-Object System.Windows.Forms.Form
$form.Text = "CAR Control Tuner | VOFA 13470 | MCP 13471"
$form.ClientSize = New-Object System.Drawing.Size(760, 632)
$form.StartPosition = "CenterScreen"
$form.MinimumSize = New-Object System.Drawing.Size(776, 671)
$form.Font = New-Object System.Drawing.Font("Segoe UI", 9)

function New-Label([string]$text, [int]$x, [int]$y, [int]$width = 100) {
    $label = New-Object System.Windows.Forms.Label
    $label.Text = $text
    $label.Location = New-Object System.Drawing.Point($x, $y)
    $label.Size = New-Object System.Drawing.Size($width, 23)
    $label.TextAlign = [System.Drawing.ContentAlignment]::MiddleLeft
    return $label
}

function New-TextBox([string]$text, [int]$x, [int]$y, [int]$width = 105) {
    $box = New-Object System.Windows.Forms.TextBox
    $box.Text = $text
    $box.Location = New-Object System.Drawing.Point($x, $y)
    $box.Size = New-Object System.Drawing.Size($width, 25)
    $box.TextAlign = [System.Windows.Forms.HorizontalAlignment]::Right
    $box.Font = New-Object System.Drawing.Font("Consolas", 10)
    return $box
}

function Add-Log([string]$text, [System.Drawing.Color]$color) {
    $entry = "$([DateTime]::Now.ToString('HH:mm:ss.fff'))  $text"
    $logBox.SelectionStart = $logBox.TextLength
    $logBox.SelectionLength = 0
    $logBox.SelectionColor = $color
    $logBox.AppendText("$entry`r`n")
    $logBox.SelectionColor = $logBox.ForeColor
    $logBox.ScrollToCaret()
    try {
        [System.IO.File]::AppendAllText(
            $script:sessionLogPath, "$entry`r`n", $script:utf8NoBom)
    } catch {}
}

function Send-TcpText(
    [System.Net.Sockets.TcpClient]$client, [string]$text) {
    try {
        if (($null -eq $client) -or (-not $client.Connected)) {
            return $false
        }
        $bytes = [System.Text.Encoding]::ASCII.GetBytes($text)
        $stream = $client.GetStream()
        $stream.Write($bytes, 0, $bytes.Length)
        return $true
    } catch {
        return $false
    }
}

function Close-TcpClient([System.Net.Sockets.TcpClient]$client) {
    if ($null -eq $client) { return }
    try { $client.Close() } catch {}
    try { $client.Dispose() } catch {}
}

function Broadcast-VofaLine([string]$line) {
    $alive = New-Object System.Collections.ArrayList
    foreach ($client in @($script:vofaClients)) {
        if (Send-TcpText $client ($line + "`n")) {
            [void]$alive.Add($client)
        } else {
            Close-TcpClient $client
        }
    }
    $script:vofaClients = $alive
}

function Broadcast-ControlLine([string]$line) {
    $alive = New-Object System.Collections.ArrayList
    foreach ($entry in @($script:controlClients)) {
        if (Send-TcpText $entry.Client ($line + "`n")) {
            [void]$alive.Add($entry)
        } else {
            Close-TcpClient $entry.Client
        }
    }
    $script:controlClients = $alive
}

function Test-ControlCommand([string]$line) {
    if ($line -match '^spd (get|stop|stat)$' -or
        $line -match '^spd run(?: (?:step|reverse|sweep|lease))?$' -or
        $line -match '^pos (get|stop|stat)$' -or
        $line -match '^pos run(?: stress)?$' -or
        $line -match '^heading (get|run|stop|stat)$' -or
        $line -match '^line (get|run|stop|stat|cal)$' -or
        $line -match '^mission (start|stop|stat)$' -or
        $line -match '^yaw (get|run|stop|stat)$' -or
        $line -match '^imu (stat|zero)$' -or
        $line -match '^app stat$' -or
        $line -match '^wdt (stat|test)$') {
        return $true
    }
    if ($line -match '^spd set [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+$') {
        return $true
    }
    if ($line -match '^pos set [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?\d+ [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+ \d+(?: [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+))?$') {
        return $true
    }
    if ($line -match '^heading set [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+ [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+$') {
        return $true
    }
    if ($line -match '^line set [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+ [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+$') {
        return $true
    }
    return $line -match '^yaw set [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+ [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+ \d+(?: [+-]?(?:\d+(?:\.\d*)?|\.\d+)(?: \d+)?)?$'
}

function Process-ControlLine($entry, [string]$line) {
    $line = $line.Trim()
    if ($line.Length -eq 0) { return }
    if ($line -eq 'bridge ping') {
        [void](Send-TcpText $entry.Client "BRIDGE PONG`n")
        return
    }
    if (-not (Test-ControlCommand $line)) {
        [void](Send-TcpText $entry.Client "ERR bridge_command`n")
        return
    }
    $script:lastExternalCommandAt = [DateTime]::Now
    if (-not (Send-Command $line $true)) {
        [void](Send-TcpText $entry.Client "ERR bridge_disconnected`n")
    }
}

function Test-TcpClientClosed([System.Net.Sockets.TcpClient]$client) {
    try {
        return $client.Client.Poll(
            0, [System.Net.Sockets.SelectMode]::SelectRead) -and
            ($client.Available -eq 0)
    } catch {
        return $true
    }
}

function Process-ControlClients {
    $alive = New-Object System.Collections.ArrayList
    foreach ($entry in @($script:controlClients)) {
        $client = $entry.Client
        try {
            if (Test-TcpClientClosed $client) {
                Close-TcpClient $client
                continue
            }
            while ($client.Available -gt 0) {
                $length = [Math]::Min($client.Available, 4096)
                $buffer = New-Object byte[] $length
                $read = $client.GetStream().Read($buffer, 0, $buffer.Length)
                if ($read -le 0) { break }
                $entry.Buffer += [System.Text.Encoding]::ASCII.GetString(
                    $buffer, 0, $read)
                while (($newline = $entry.Buffer.IndexOf("`n")) -ge 0) {
                    $line = $entry.Buffer.Substring(0, $newline).TrimEnd("`r")
                    $entry.Buffer = $entry.Buffer.Substring($newline + 1)
                    Process-ControlLine $entry $line
                }
            }
            [void]$alive.Add($entry)
        } catch {
            Close-TcpClient $client
        }
    }
    $script:controlClients = $alive
}

function Accept-TcpClients {
    try {
        while (($null -ne $script:vofaListener) -and
            $script:vofaListener.Pending()) {
            $client = $script:vofaListener.AcceptTcpClient()
            $client.NoDelay = $true
            [void]$script:vofaClients.Add($client)
            Add-Log "VOFA+ connected on 127.0.0.1:$script:vofaPort." ([System.Drawing.Color]::DarkCyan)
        }
        while (($null -ne $script:controlListener) -and
            $script:controlListener.Pending()) {
            $client = $script:controlListener.AcceptTcpClient()
            $client.NoDelay = $true
            $entry = [pscustomobject]@{ Client = $client; Buffer = "" }
            [void]$script:controlClients.Add($entry)
            [void](Send-TcpText $client (
                "BRIDGE READY control=$script:controlPort speed=6 position=6 yaw=7`n"))
        }
    } catch {
        Add-Log "Bridge accept error: $($_.Exception.Message)" ([System.Drawing.Color]::Firebrick)
    }
}

function Stop-TcpBridge {
    foreach ($client in @($script:vofaClients)) {
        Close-TcpClient $client
    }
    foreach ($entry in @($script:controlClients)) {
        Close-TcpClient $entry.Client
    }
    $script:vofaClients.Clear()
    $script:controlClients.Clear()
    if ($null -ne $script:vofaListener) {
        try { $script:vofaListener.Stop() } catch {}
        $script:vofaListener = $null
    }
    if ($null -ne $script:controlListener) {
        try { $script:controlListener.Stop() } catch {}
        $script:controlListener = $null
    }
}

function Start-TcpBridge {
    try {
        $script:vofaListener = New-Object System.Net.Sockets.TcpListener(
            [System.Net.IPAddress]::Loopback, $script:vofaPort)
        $script:controlListener = New-Object System.Net.Sockets.TcpListener(
            [System.Net.IPAddress]::Loopback, $script:controlPort)
        $script:vofaListener.Start()
        $script:controlListener.Start()
        Add-Log "Bridge ready: VOFA=$script:vofaPort MCP=$script:controlPort." ([System.Drawing.Color]::DarkCyan)
    } catch {
        Add-Log "Bridge start error: $($_.Exception.Message)" ([System.Drawing.Color]::Firebrick)
        Stop-TcpBridge
    }
}

function Start-RunCapture {
    $script:runCsvPath = Join-Path $script:runtimeDir (
        "run_{0}.csv" -f [DateTime]::Now.ToString("yyyyMMdd_HHmmss"))
    $header = "timestamp,state,left_pps,right_pps,left_output,right_output,invalid_left,invalid_right,result,high_z,kp,ki,kd,target_pps,limit_permille`r`n"
    [System.IO.File]::WriteAllText(
        $script:runCsvPath, $header, $script:utf8NoBom)
    [System.IO.File]::WriteAllText(
        $script:latestRunPath, $header, $script:utf8NoBom)
    $script:captureActive = $true
}

function Save-Status([hashtable]$values) {
    $timestamp = [DateTime]::Now.ToString("o")
    $status = [ordered]@{
        timestamp = $timestamp
        port = $portCombo.Text
        baud = $script:baudRate
        state = $values['state']
        left_speed_pps = [int]$values['left']
        right_speed_pps = [int]$values['right']
        left_output_permille = [int]$values['outL']
        right_output_permille = [int]$values['outR']
        invalid_left = [uint32]$values['invL']
        invalid_right = [uint32]$values['invR']
        result = [uint32]$values['res']
        high_impedance = ($values['hz'] -eq '1')
        kp = $kpBox.Text
        ki = $kiBox.Text
        kd = $kdBox.Text
        target_pps = $targetBox.Text
        output_limit_permille = $limitBox.Text
    }
    try {
        $json = $status | ConvertTo-Json
        [System.IO.File]::WriteAllText(
            $script:latestStatusPath, $json, $script:utf8NoBom)
    } catch {}

    if ($script:captureActive) {
        $row = @(
            $timestamp, $values['state'], $values['left'], $values['right'],
            $values['outL'], $values['outR'], $values['invL'], $values['invR'],
            $values['res'], $values['hz'], $kpBox.Text, $kiBox.Text,
            $kdBox.Text, $targetBox.Text, $limitBox.Text) -join ','
        try {
            [System.IO.File]::AppendAllText(
                $script:runCsvPath, "$row`r`n", $script:utf8NoBom)
            [System.IO.File]::AppendAllText(
                $script:latestRunPath, "$row`r`n", $script:utf8NoBom)
        } catch {}
        if ($values['state'] -in @('DONE', 'ABORT', 'LOCKED')) {
            $script:captureActive = $false
        }
    }
}

function Parse-WaveLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        '^wave:(?<lt>-?\d+),(?<ls>-?\d+),(?<rt>-?\d+),(?<rs>-?\d+),(?<lo>-?\d+),(?<ro>-?\d+)$')
    if (-not $match.Success) { return $null }
    return @{
        left_target_pps = [int]$match.Groups['lt'].Value
        left_speed_pps = [int]$match.Groups['ls'].Value
        right_target_pps = [int]$match.Groups['rt'].Value
        right_speed_pps = [int]$match.Groups['rs'].Value
        left_output_permille = [int]$match.Groups['lo'].Value
        right_output_permille = [int]$match.Groups['ro'].Value
    }
}

function Save-Wave([hashtable]$values) {
    $timestamp = [DateTime]::Now.ToString("o")
    $wave = [ordered]@{
        timestamp = $timestamp
        left_target_pps = $values['left_target_pps']
        left_speed_pps = $values['left_speed_pps']
        right_target_pps = $values['right_target_pps']
        right_speed_pps = $values['right_speed_pps']
        left_output_permille = $values['left_output_permille']
        right_output_permille = $values['right_output_permille']
    }
    $row = @(
        $timestamp,
        $values['left_target_pps'], $values['left_speed_pps'],
        $values['right_target_pps'], $values['right_speed_pps'],
        $values['left_output_permille'],
        $values['right_output_permille']) -join ','
    try {
        [System.IO.File]::WriteAllText(
            $script:latestWavePath,
            ($wave | ConvertTo-Json), $script:utf8NoBom)
        [System.IO.File]::AppendAllText(
            $script:telemetryCsvPath, "$row`r`n", $script:utf8NoBom)
        [System.IO.File]::AppendAllText(
            $script:latestTelemetryPath, "$row`r`n", $script:utf8NoBom)
    } catch {}
}

function Forward-Wave([hashtable]$values) {
    $fireWater = "wave: {0}, {1}, {2}, {3}, {4}, {5}" -f
        $values['left_target_pps'],
        $values['left_speed_pps'],
        $values['right_target_pps'],
        $values['right_speed_pps'],
        $values['left_output_permille'],
        $values['right_output_permille']
    Broadcast-VofaLine $fireWater
    Save-Wave $values
}

function Parse-PositionWaveLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        '^poswave:(?<lt>-?\d+),(?<lc>-?\d+),(?<rt>-?\d+),(?<rc>-?\d+),(?<lv>-?\d+),(?<rv>-?\d+)$')
    if (-not $match.Success) { return $null }
    return @{
        left_target_count = [int]$match.Groups['lt'].Value
        left_count = [int]$match.Groups['lc'].Value
        right_target_count = [int]$match.Groups['rt'].Value
        right_count = [int]$match.Groups['rc'].Value
        left_speed_target_pps = [int]$match.Groups['lv'].Value
        right_speed_target_pps = [int]$match.Groups['rv'].Value
    }
}

function Forward-PositionWave([hashtable]$values) {
    $timestamp = [DateTime]::Now.ToString("o")
    $fireWater = "position: {0}, {1}, {2}, {3}, {4}, {5}" -f
        $values['left_target_count'],
        $values['left_count'],
        $values['right_target_count'],
        $values['right_count'],
        $values['left_speed_target_pps'],
        $values['right_speed_target_pps']
    $positionWave = [ordered]@{
        timestamp = $timestamp
        left_target_count = $values['left_target_count']
        left_count = $values['left_count']
        right_target_count = $values['right_target_count']
        right_count = $values['right_count']
        left_speed_target_pps = $values['left_speed_target_pps']
        right_speed_target_pps = $values['right_speed_target_pps']
    }
    $row = @(
        $timestamp,
        $values['left_target_count'], $values['left_count'],
        $values['right_target_count'], $values['right_count'],
        $values['left_speed_target_pps'],
        $values['right_speed_target_pps']) -join ','
    Broadcast-VofaLine $fireWater
    try {
        [System.IO.File]::WriteAllText(
            $script:latestPositionWavePath,
            ($positionWave | ConvertTo-Json), $script:utf8NoBom)
        [System.IO.File]::AppendAllText(
            $script:positionTelemetryCsvPath,
            "$row`r`n", $script:utf8NoBom)
        [System.IO.File]::AppendAllText(
            $script:latestPositionTelemetryPath,
            "$row`r`n", $script:utf8NoBom)
    } catch {}
}

function Parse-YawWaveLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        '^yawwave:(?<target>-?\d+),(?<current>-?\d+),(?<error>-?\d+),(?<rate>-?\d+),(?<turn>-?\d+),(?<left>-?\d+),(?<right>-?\d+)$')
    if (-not $match.Success) { return $null }
    return @{
        target_mdeg = [int]$match.Groups['target'].Value
        current_mdeg = [int]$match.Groups['current'].Value
        error_mdeg = [int]$match.Groups['error'].Value
        yaw_rate_mdps = [int]$match.Groups['rate'].Value
        turn_target_pps = [int]$match.Groups['turn'].Value
        left_speed_pps = [int]$match.Groups['left'].Value
        right_speed_pps = [int]$match.Groups['right'].Value
    }
}

function Forward-YawWave([hashtable]$values) {
    $timestamp = [DateTime]::Now.ToString("o")
    $fireWater = "yaw: {0}, {1}, {2}, {3}, {4}, {5}, {6}" -f
        $values['target_mdeg'],
        $values['current_mdeg'],
        $values['error_mdeg'],
        $values['yaw_rate_mdps'],
        $values['turn_target_pps'],
        $values['left_speed_pps'],
        $values['right_speed_pps']
    $yawWave = [ordered]@{
        timestamp = $timestamp
        target_mdeg = $values['target_mdeg']
        current_mdeg = $values['current_mdeg']
        error_mdeg = $values['error_mdeg']
        yaw_rate_mdps = $values['yaw_rate_mdps']
        turn_target_pps = $values['turn_target_pps']
        left_speed_pps = $values['left_speed_pps']
        right_speed_pps = $values['right_speed_pps']
    }
    $row = @(
        $timestamp, $values['target_mdeg'], $values['current_mdeg'],
        $values['error_mdeg'], $values['yaw_rate_mdps'],
        $values['turn_target_pps'], $values['left_speed_pps'],
        $values['right_speed_pps']) -join ','
    Broadcast-VofaLine $fireWater
    try {
        [System.IO.File]::WriteAllText(
            $script:latestYawWavePath,
            ($yawWave | ConvertTo-Json), $script:utf8NoBom)
        [System.IO.File]::AppendAllText(
            $script:yawTelemetryCsvPath,
            "$row`r`n", $script:utf8NoBom)
        [System.IO.File]::AppendAllText(
            $script:latestYawTelemetryPath,
            "$row`r`n", $script:utf8NoBom)
    } catch {}
}

function Parse-LineWaveLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        '^linewave:(?<error>-?\d+),(?<corr>-?\d+),(?<lt>-?\d+),(?<ls>-?\d+),(?<rt>-?\d+),(?<rs>-?\d+),(?<count>\d+)$')
    if (-not $match.Success) { return $null }
    return @{
        line_error = [int]$match.Groups['error'].Value
        correction_pps = [int]$match.Groups['corr'].Value
        left_target_pps = [int]$match.Groups['lt'].Value
        left_speed_pps = [int]$match.Groups['ls'].Value
        right_target_pps = [int]$match.Groups['rt'].Value
        right_speed_pps = [int]$match.Groups['rs'].Value
        active_count = [int]$match.Groups['count'].Value
    }
}

function Forward-LineWave([hashtable]$values) {
    $timestamp = [DateTime]::Now.ToString("o")
    $fireWater = "line: {0}, {1}, {2}, {3}, {4}, {5}" -f
        $values['line_error'],
        $values['correction_pps'],
        $values['left_target_pps'],
        $values['left_speed_pps'],
        $values['right_target_pps'],
        $values['right_speed_pps']
    $lineWave = [ordered]@{
        timestamp = $timestamp
        line_error = $values['line_error']
        correction_pps = $values['correction_pps']
        left_target_pps = $values['left_target_pps']
        left_speed_pps = $values['left_speed_pps']
        right_target_pps = $values['right_target_pps']
        right_speed_pps = $values['right_speed_pps']
        active_count = $values['active_count']
    }
    $row = @(
        $timestamp, $values['line_error'], $values['correction_pps'],
        $values['left_target_pps'], $values['left_speed_pps'],
        $values['right_target_pps'], $values['right_speed_pps'],
        $values['active_count']) -join ','
    Broadcast-VofaLine $fireWater
    try {
        [System.IO.File]::WriteAllText(
            $script:latestLineWavePath,
            ($lineWave | ConvertTo-Json), $script:utf8NoBom)
        [System.IO.File]::AppendAllText(
            $script:lineTelemetryCsvPath,
            "$row`r`n", $script:utf8NoBom)
        [System.IO.File]::AppendAllText(
            $script:latestLineTelemetryPath,
            "$row`r`n", $script:utf8NoBom)
    } catch {}
}

function Refresh-Ports {
    $selected = $portCombo.Text
    if (($selected -eq "") -and (Test-Path $script:lastPortPath)) {
        $selected = [System.IO.File]::ReadAllText(
            $script:lastPortPath, [System.Text.Encoding]::ASCII).Trim()
    }
    $portCombo.Items.Clear()
    foreach ($port in ([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)) {
        [void]$portCombo.Items.Add($port)
    }
    if (($selected -ne "") -and $portCombo.Items.Contains($selected)) {
        $portCombo.SelectedItem = $selected
    } elseif ($portCombo.Items.Count -gt 0) {
        $portCombo.SelectedIndex = 0
    }
}

function Send-Command([string]$command, [bool]$quiet = $false) {
    if (($null -eq $script:serial) -or (-not $script:serial.IsOpen)) {
        Add-Log "Serial port is disconnected." ([System.Drawing.Color]::Firebrick)
        return $false
    }
    try {
        $script:serial.Write($command + "`n")
        if (-not $quiet) {
            Add-Log "TX  $command" ([System.Drawing.Color]::SteelBlue)
        }
        return $true
    } catch {
        Add-Log "TX error: $($_.Exception.Message)" ([System.Drawing.Color]::Firebrick)
        return $false
    }
}

function Disconnect-Serial {
    if ($null -ne $script:serial) {
        try {
            if ($script:serial.IsOpen) {
                $script:serial.Close()
            }
        } catch {}
        $script:serial.Dispose()
        $script:serial = $null
    }
    $connectButton.Text = "Connect"
    $connectionValue.Text = "DISCONNECTED"
    $connectionValue.ForeColor = [System.Drawing.Color]::Firebrick
    $script:initialReadPending = $false
    $script:primePending = $false
}

function Update-ConfigFromLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        'kp=(?<kp>[-0-9.]+)\s+ki=(?<ki>[-0-9.]+)\s+kd=(?<kd>[-0-9.]+)\s+target=(?<target>[-0-9.]+)\s+limit=(?<limit>[0-9]+)')
    if ($match.Success) {
        $kpBox.Text = $match.Groups['kp'].Value
        $kiBox.Text = $match.Groups['ki'].Value
        $kdBox.Text = $match.Groups['kd'].Value
        $targetBox.Text = $match.Groups['target'].Value
        $limitBox.Text = $match.Groups['limit'].Value
        foreach ($box in @($kpBox, $kiBox, $kdBox, $targetBox, $limitBox)) {
            $box.SelectionStart = 0
            $box.SelectionLength = 0
        }
    }
}

function Update-PositionConfigFromLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        'kp=(?<kp>[-0-9.]+)\s+target=(?<target>-?\d+)\s+max=(?<max>[-0-9.]+)\s+limit=(?<limit>\d+)\s+tol=(?<tol>\d+)(?:\s+syncKp=(?<syncKp>[-0-9.]+)\s+syncMax=(?<syncMax>[-0-9.]+))?')
    if ($match.Success) {
        $posKpBox.Text = $match.Groups['kp'].Value
        $posTargetBox.Text = $match.Groups['target'].Value
        $posMaxSpeedBox.Text = $match.Groups['max'].Value
        $posLimitBox.Text = $match.Groups['limit'].Value
        $posToleranceBox.Text = $match.Groups['tol'].Value
        if ($match.Groups['syncKp'].Success) {
            $posSyncKpBox.Text = $match.Groups['syncKp'].Value
            $posSyncMaxBox.Text = $match.Groups['syncMax'].Value
        }
        foreach ($box in @(
            $posKpBox, $posTargetBox, $posMaxSpeedBox,
            $posLimitBox, $posToleranceBox, $posSyncKpBox,
            $posSyncMaxBox)) {
            $box.SelectionStart = 0
            $box.SelectionLength = 0
        }
    }
}

function Update-YawConfigFromLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        'kp=(?<kp>[-0-9.]+)\s+ki=(?<ki>[-0-9.]+)\s+kd=(?<kd>[-0-9.]+)\s+target=(?<target>[-0-9.]+)\s+max=(?<max>[-0-9.]+)\s+limit=(?<limit>\d+)\s+tol=(?<tol>[-0-9.]+)\s+rate=(?<rate>[-0-9.]+)\s+stime=(?<stime>\d+)\s+timeout=(?<timeout>\d+)(?:\s+min=(?<min>[-0-9.]+))?(?:\s+boost=(?<boost>\d+))?')
    if (-not $match.Success) { return }

    $yawKpBox.Text = $match.Groups['kp'].Value
    $yawKiBox.Text = $match.Groups['ki'].Value
    $yawKdBox.Text = $match.Groups['kd'].Value
    $yawTargetBox.Text = $match.Groups['target'].Value
    $yawMaxSpeedBox.Text = $match.Groups['max'].Value
    $yawLimitBox.Text = $match.Groups['limit'].Value
    $yawToleranceBox.Text = $match.Groups['tol'].Value
    $yawSettleRateBox.Text = $match.Groups['rate'].Value
    $yawSettleTimeBox.Text = $match.Groups['stime'].Value
    $yawTimeoutBox.Text = $match.Groups['timeout'].Value
    if ($match.Groups['min'].Success) {
        $yawMinSpeedBox.Text = $match.Groups['min'].Value
    }
    if ($match.Groups['boost'].Success) {
        $yawBoostBox.Text = $match.Groups['boost'].Value
    }
    foreach ($box in @(
        $yawKpBox, $yawKiBox, $yawKdBox, $yawTargetBox,
        $yawMaxSpeedBox, $yawMinSpeedBox, $yawLimitBox, $yawBoostBox,
        $yawToleranceBox,
        $yawSettleRateBox, $yawSettleTimeBox, $yawTimeoutBox)) {
        $box.SelectionStart = 0
        $box.SelectionLength = 0
    }
}

function Update-LineConfigFromLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        'kp=(?<kp>[-0-9.]+)\s+ki=(?<ki>[-0-9.]+)\s+kd=(?<kd>[-0-9.]+)\s+base=(?<base>[-0-9.]+)\s+max=(?<max>[-0-9.]+)\s+limit=(?<limit>\d+)\s+dead=(?<dead>[-0-9.]+)\s+duration=(?<duration>\d+)')
    if (-not $match.Success) { return }

    $lineKpBox.Text = $match.Groups['kp'].Value
    $lineKiBox.Text = $match.Groups['ki'].Value
    $lineKdBox.Text = $match.Groups['kd'].Value
    $lineBaseBox.Text = $match.Groups['base'].Value
    $lineMaxBox.Text = $match.Groups['max'].Value
    $lineLimitBox.Text = $match.Groups['limit'].Value
    $lineDeadbandBox.Text = $match.Groups['dead'].Value
    $lineDurationBox.Text = $match.Groups['duration'].Value
    foreach ($box in @(
        $lineKpBox, $lineKiBox, $lineKdBox, $lineBaseBox,
        $lineMaxBox, $lineLimitBox, $lineDeadbandBox,
        $lineDurationBox)) {
        $box.SelectionStart = 0
        $box.SelectionLength = 0
    }
}

function Parse-KeyValueLine([string]$line) {
    $values = @{}
    foreach ($match in [System.Text.RegularExpressions.Regex]::Matches(
        $line, '(?<key>[A-Za-z][A-Za-z0-9]*)=(?<value>[^\s]+)')) {
        $values[$match.Groups['key'].Value] = $match.Groups['value'].Value
    }
    return $values
}

function Save-LineStatus([hashtable]$values) {
    $status = [ordered]@{
        timestamp = [DateTime]::Now.ToString("o")
        state = $values['state']
        sensor_state = $values['sensor']
        raw = [uint32]$values['raw']
        active_mask = [uint32]$values['mask']
        active_count = [uint32]$values['count']
        line_error = [int]$values['error']
        line_seen = ($values['seen'] -eq '1')
        sample_age_ms = [uint32]$values['age']
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
        high_impedance = ($values['hz'] -eq '1')
        line_interval_ms = [uint32]$values['lineDt']
        line_max_interval_ms = [uint32]$values['lineMax']
    }
    try {
        [System.IO.File]::WriteAllText(
            $script:latestLineStatusPath,
            ($status | ConvertTo-Json), $script:utf8NoBom)
    } catch {}
}

function Update-LineStatusFromLine([string]$line) {
    if (-not $line.StartsWith('LSTAT ')) { return $false }
    $values = Parse-KeyValueLine $line
    foreach ($key in @(
         'state', 'sensor', 'raw', 'mask', 'count', 'error', 'seen',
         'age', 'base', 'corr', 'yawT', 'yawR', 'yawBoost', 'imu',
         'tL', 'tR', 'vL', 'vR', 'outL',
        'outR', 'res', 'hz', 'lineDt', 'lineMax')) {
        if (-not $values.ContainsKey($key)) { return $false }
    }
    if ($values['state'] -eq 'RUN') {
        $script:lineStatusSource = 'test'
    }

    if ($script:lineStatusSource -eq 'test') {
        $lineStateValue.Text = $values['state']
        $lineResultValue.Text = $values['res']
    }
    $lineSensorStateValue.Text = $values['sensor']
    $lineRawValue.Text = '0x{0:X2}' -f [int]$values['raw']
    $lineBitsValue.Text = ([Convert]::ToString(
        [int]$values['mask'], 2)).PadLeft(8, '0') +
        ' (' + $values['count'] + ')'
    $lineErrorValue.Text = $values['error']
    $lineCorrectionValue.Text = $values['corr'] + ' pps'
    $lineTargetValue.Text = $values['tL'] + ' / ' +
        $values['tR'] + ' pps'
    $lineSpeedValue.Text = $values['vL'] + ' / ' +
        $values['vR'] + ' pps'
    $lineOutputValue.Text = $values['outL'] + ' / ' +
        $values['outR']
    $lineAgeValue.Text = $values['age'] + ' ms'
    if ($script:lineStatusSource -eq 'test') {
        if ($values['hz'] -eq '1') {
            $lineHighZValue.Text = 'HIGH-Z'
            $lineHighZValue.ForeColor = [System.Drawing.Color]::ForestGreen
        } else {
            $lineHighZValue.Text = 'ARMED'
            $lineHighZValue.ForeColor = [System.Drawing.Color]::DarkOrange
        }
    }
    Save-LineStatus $values
    return $true
}

function Update-MissionStatusFromLine([string]$line) {
    if (-not $line.StartsWith('MSTAT ')) { return $false }
    $values = Parse-KeyValueLine $line
    foreach ($key in @(
        'state', 'runs', 'base', 'limit', 'elapsed', 'error',
        'count', 'seen', 'age', 'res', 'control', 'hz')) {
        if (-not $values.ContainsKey($key)) { return $false }
    }
    if ($values['state'] -eq 'RUN') {
        $script:lineStatusSource = 'mission'
    }

    if ($script:lineStatusSource -eq 'mission') {
        $lineStateValue.Text = $values['state']
        $lineResultValue.Text = $values['res']
        if ($values['hz'] -eq '1') {
            $lineHighZValue.Text = 'HIGH-Z'
            $lineHighZValue.ForeColor = [System.Drawing.Color]::ForestGreen
        } else {
            $lineHighZValue.Text = 'ARMED'
            $lineHighZValue.ForeColor = [System.Drawing.Color]::DarkOrange
        }
    }

    $status = [ordered]@{
        timestamp = [DateTime]::Now.ToString('o')
        state = $values['state']
        run_count = [uint32]$values['runs']
        base_speed_pps = [int]$values['base']
        output_limit_permille = [uint16]$values['limit']
        elapsed_ms = [uint32]$values['elapsed']
        line_error = [int]$values['error']
        active_count = [uint16]$values['count']
        line_seen = ($values['seen'] -eq '1')
        sample_age_ms = [uint32]$values['age']
        result = [uint32]$values['res']
        controller_running = ($values['control'] -eq '1')
        high_impedance = ($values['hz'] -eq '1')
    }
    try {
        [System.IO.File]::WriteAllText(
            $script:latestMissionStatusPath,
            ($status | ConvertTo-Json), $script:utf8NoBom)
    } catch {}
    return $true
}

function Save-PositionStatus([hashtable]$values) {
    $status = [ordered]@{
        timestamp = [DateTime]::Now.ToString("o")
        state = $values['state']
        profile = $values['profile']
        step = [int]$values['step']
        step_count = [int]$values['steps']
        completed_moves = [int]$values['done']
        worst_final_error_count = [int]$values['worst']
        left_recovery_count = [int]$values['recL']
        right_recovery_count = [int]$values['recR']
        left_target_count = [int]$values['tL']
        left_count = [int]$values['cL']
        left_error_count = [int]$values['eL']
        right_target_count = [int]$values['tR']
        right_count = [int]$values['cR']
        right_error_count = [int]$values['eR']
        left_speed_pps = [int]$values['vL']
        right_speed_pps = [int]$values['vR']
        invalid_left = [int]$values['invL']
        invalid_right = [int]$values['invR']
        result = [int]$values['res']
        high_impedance = ($values['hz'] -eq '1')
    }
    try {
        [System.IO.File]::WriteAllText(
            $script:latestPositionStatusPath,
            ($status | ConvertTo-Json), $script:utf8NoBom)
    } catch {}
}

function Update-PositionStatusFromLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        '^PSTAT state=(?<state>[A-Z]+) profile=(?<profile>[A-Z]+) step=(?<step>\d+)/(?<steps>\d+) done=(?<done>\d+) worst=(?<worst>\d+) recL=(?<recL>\d+) recR=(?<recR>\d+) tL=(?<tL>-?\d+) cL=(?<cL>-?\d+) eL=(?<eL>-?\d+) tR=(?<tR>-?\d+) cR=(?<cR>-?\d+) eR=(?<eR>-?\d+) vL=(?<vL>-?\d+) vR=(?<vR>-?\d+) invL=(?<invL>\d+) invR=(?<invR>\d+) res=(?<res>\d+) hz=(?<hz>[01])$')
    if (-not $match.Success) { return $false }

    $values = @{}
    foreach ($key in @(
        'state', 'profile', 'step', 'steps', 'done', 'worst', 'recL', 'recR',
        'tL', 'cL', 'eL', 'tR', 'cR', 'eR', 'vL', 'vR',
        'invL', 'invR', 'res', 'hz')) {
        $values[$key] = $match.Groups[$key].Value
    }
    $posStateValue.Text = $values['state']
    $posStepValue.Text = "{0} {1}/{2} ({3} done)" -f
        $values['profile'], $values['step'], $values['steps'], $values['done']
    $posResultValue.Text = $values['res']
    $posLeftTargetValue.Text = $values['tL']
    $posLeftActualValue.Text = $values['cL']
    $posLeftErrorValue.Text = $values['eL']
    $posRightTargetValue.Text = $values['tR']
    $posRightActualValue.Text = $values['cR']
    $posRightErrorValue.Text = $values['eR']
    $posWorstValue.Text = $values['worst'] + " counts"
    $posSpeedValue.Text = $values['vL'] + " / " + $values['vR'] + " pps"
    $posInvalidValue.Text = $values['invL'] + " / " + $values['invR']
    $posRecoveryValue.Text = $values['recL'] + " / " + $values['recR']
    if ($values['hz'] -eq '1') {
        $posHighZValue.Text = "HIGH-Z"
        $posHighZValue.ForeColor = [System.Drawing.Color]::ForestGreen
    } else {
        $posHighZValue.Text = "ARMED"
        $posHighZValue.ForeColor = [System.Drawing.Color]::DarkOrange
    }
    Save-PositionStatus $values
    return $true
}

function Save-YawStatus([hashtable]$values) {
    $status = [ordered]@{
        timestamp = [DateTime]::Now.ToString("o")
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
        result = [int]$values['res']
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
    try {
        [System.IO.File]::WriteAllText(
            $script:latestYawStatusPath,
            ($status | ConvertTo-Json), $script:utf8NoBom)
    } catch {}
}

function Update-YawStatusFromLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        '^YSTAT state=(?<state>[A-Z]+) target=(?<target>-?\d+) current=(?<current>-?\d+) error=(?<error>-?\d+) rate=(?<rate>-?\d+) turn=(?<turn>-?\d+) tL=(?<tL>-?\d+) tR=(?<tR>-?\d+) vL=(?<vL>-?\d+) vR=(?<vR>-?\d+) outL=(?<outL>-?\d+) outR=(?<outR>-?\d+) res=(?<res>\d+) hz=(?<hz>[01]) loop=(?<loop>\d+) loopMax=(?<loopMax>\d+) imuDt=(?<imuDt>\d+) imuMax=(?<imuMax>\d+) yawDt=(?<yawDt>\d+) yawMax=(?<yawMax>\d+) lcd=(?<lcd>\d+) lcdMax=(?<lcdMax>\d+)$')
    if (-not $match.Success) { return $false }

    $values = @{}
    foreach ($key in @(
        'state', 'target', 'current', 'error', 'rate', 'turn',
        'tL', 'tR', 'vL', 'vR', 'outL', 'outR', 'res', 'hz',
        'loop', 'loopMax', 'imuDt', 'imuMax', 'yawDt', 'yawMax',
        'lcd', 'lcdMax')) {
        $values[$key] = $match.Groups[$key].Value
    }
    $culture = [System.Globalization.CultureInfo]::InvariantCulture
    $yawStateValue.Text = $values['state']
    $yawResultValue.Text = $values['res']
    $yawTargetValue.Text = (([int]$values['target']) / 1000.0).ToString(
        '0.000', $culture) + ' deg'
    $yawCurrentValue.Text = (([int]$values['current']) / 1000.0).ToString(
        '0.000', $culture) + ' deg'
    $yawErrorValue.Text = (([int]$values['error']) / 1000.0).ToString(
        '0.000', $culture) + ' deg'
    $yawRateValue.Text = (([int]$values['rate']) / 1000.0).ToString(
        '0.000', $culture) + ' dps'
    $yawTurnValue.Text = $values['turn'] + ' pps'
    $yawTargetSpeedValue.Text = $values['tL'] + ' / ' +
        $values['tR'] + ' pps'
    $yawSpeedValue.Text = $values['vL'] + ' / ' +
        $values['vR'] + ' pps'
    $yawOutputValue.Text = $values['outL'] + ' / ' +
        $values['outR'] + ' permille'
    if ($values['hz'] -eq '1') {
        $yawHighZValue.Text = 'HIGH-Z'
        $yawHighZValue.ForeColor = [System.Drawing.Color]::ForestGreen
    } else {
        $yawHighZValue.Text = 'ARMED'
        $yawHighZValue.ForeColor = [System.Drawing.Color]::DarkOrange
    }
    Save-YawStatus $values
    return $true
}

function Update-StatusFromLine([string]$line) {
    $match = [System.Text.RegularExpressions.Regex]::Match(
        $line,
        '^STAT state=(?<state>[A-Z]+) left=(?<left>-?\d+) right=(?<right>-?\d+) outL=(?<outL>-?\d+) outR=(?<outR>-?\d+) invL=(?<invL>\d+) invR=(?<invR>\d+) res=(?<res>\d+) hz=(?<hz>[01])$')
    if (-not $match.Success) {
        return $false
    }
    $values = @{}
    foreach ($key in @(
        'state', 'left', 'right', 'outL', 'outR',
        'invL', 'invR', 'res', 'hz')) {
        $values[$key] = $match.Groups[$key].Value
    }
    if ($values.ContainsKey('state')) { $stateValue.Text = $values['state'] }
    if ($values.ContainsKey('left')) { $leftSpeedValue.Text = $values['left'] + " pps" }
    if ($values.ContainsKey('right')) { $rightSpeedValue.Text = $values['right'] + " pps" }
    if ($values.ContainsKey('outL')) { $leftOutputValue.Text = $values['outL'] + " / 1000" }
    if ($values.ContainsKey('outR')) { $rightOutputValue.Text = $values['outR'] + " / 1000" }
    if ($values.ContainsKey('invL')) { $leftInvalidValue.Text = $values['invL'] }
    if ($values.ContainsKey('invR')) { $rightInvalidValue.Text = $values['invR'] }
    if ($values.ContainsKey('res')) { $resultValue.Text = $values['res'] }
    if ($values.ContainsKey('hz')) {
        if ($values['hz'] -eq '1') {
            $highZValue.Text = "HIGH-Z"
            $highZValue.ForeColor = [System.Drawing.Color]::ForestGreen
        } else {
            $highZValue.Text = "ARMED"
            $highZValue.ForeColor = [System.Drawing.Color]::DarkOrange
        }
    }
    Save-Status $values
    return $true
}

function Process-Line([string]$line) {
    $line = $line.Trim()
    if ($line.Length -eq 0) { return }
    if ($line.StartsWith('wave:')) {
        $wave = Parse-WaveLine $line
        if ($null -ne $wave) {
            Broadcast-ControlLine $line
            Forward-Wave $wave
        } else {
            Add-Log "RX dropped malformed wave frame." ([System.Drawing.Color]::DarkOrange)
        }
        return
    }
    if ($line.StartsWith('poswave:')) {
        $positionWave = Parse-PositionWaveLine $line
        if ($null -ne $positionWave) {
            Broadcast-ControlLine $line
            Forward-PositionWave $positionWave
        } else {
            Add-Log "RX dropped malformed position frame." ([System.Drawing.Color]::DarkOrange)
        }
        return
    }
    if ($line.StartsWith('yawwave:')) {
        $yawWave = Parse-YawWaveLine $line
        if ($null -ne $yawWave) {
            Broadcast-ControlLine $line
            Forward-YawWave $yawWave
        } else {
            Add-Log "RX dropped malformed Yaw frame." ([System.Drawing.Color]::DarkOrange)
        }
        return
    }
    if ($line.StartsWith('linewave:')) {
        $lineWave = Parse-LineWaveLine $line
        if ($null -ne $lineWave) {
            Broadcast-ControlLine $line
            Forward-LineWave $lineWave
        } else {
            Add-Log "RX dropped malformed line frame." ([System.Drawing.Color]::DarkOrange)
        }
        return
    }
    if ($line.StartsWith('STAT ')) {
        if (Update-StatusFromLine $line) {
            Broadcast-ControlLine $line
        } else {
            Add-Log "RX dropped malformed status frame." ([System.Drawing.Color]::DarkOrange)
        }
        return
    }
    if ($line.StartsWith('PSTAT ')) {
        if (Update-PositionStatusFromLine $line) {
            Broadcast-ControlLine $line
        } else {
            Add-Log "RX dropped malformed position status." ([System.Drawing.Color]::DarkOrange)
        }
        return
    }
    if ($line.StartsWith('YSTAT ')) {
        if (Update-YawStatusFromLine $line) {
            Broadcast-ControlLine $line
        } else {
            Add-Log "RX dropped malformed Yaw status." ([System.Drawing.Color]::DarkOrange)
        }
        return
    }
    if ($line.StartsWith('LSTAT ')) {
        if (Update-LineStatusFromLine $line) {
            Broadcast-ControlLine $line
        } else {
            Add-Log "RX dropped malformed line status." ([System.Drawing.Color]::DarkOrange)
        }
        return
    }
    if ($line.StartsWith('MSTAT ')) {
        if (Update-MissionStatusFromLine $line) {
            Broadcast-ControlLine $line
        } else {
            Add-Log "RX dropped malformed mission status." ([System.Drawing.Color]::DarkOrange)
        }
        return
    }
    Broadcast-ControlLine $line
    if ($line.StartsWith('OK CFG ')) {
        Update-ConfigFromLine $line
    } elseif ($line.StartsWith('OK PCFG ')) {
        Update-PositionConfigFromLine $line
    } elseif ($line.StartsWith('OK YCFG ')) {
        Update-YawConfigFromLine $line
    } elseif ($line.StartsWith('OK LCFG ')) {
        Update-LineConfigFromLine $line
    }
    $color = if ($line.StartsWith('ERR ')) {
        [System.Drawing.Color]::Firebrick
    } else {
        [System.Drawing.Color]::ForestGreen
    }
    Add-Log "RX  $line" $color
}

function Read-UiConfig {
    $culture = [System.Globalization.CultureInfo]::InvariantCulture
    $style = [System.Globalization.NumberStyles]::Float
    [single]$kp = 0
    [single]$ki = 0
    [single]$kd = 0
    [single]$target = 0
    [uint16]$limit = 0

    $valid = [single]::TryParse($kpBox.Text, $style, $culture, [ref]$kp) -and
        [single]::TryParse($kiBox.Text, $style, $culture, [ref]$ki) -and
        [single]::TryParse($kdBox.Text, $style, $culture, [ref]$kd) -and
        [single]::TryParse($targetBox.Text, $style, $culture, [ref]$target) -and
        [uint16]::TryParse($limitBox.Text, [ref]$limit)
    if (-not $valid) {
        Add-Log "Parameter format error." ([System.Drawing.Color]::Firebrick)
        return $null
    }
    if (($kp -lt 0) -or ($kp -gt 5) -or
        ($ki -lt 0) -or ($ki -gt 20) -or
        ($kd -lt 0) -or ($kd -gt 2) -or
        ($target -lt 100) -or ($target -gt 6000) -or
        ($limit -lt 100) -or ($limit -gt 1000)) {
        Add-Log "Parameter outside firmware range." ([System.Drawing.Color]::Firebrick)
        return $null
    }

    return ('spd set {0} {1} {2} {3} {4}' -f
        $kp.ToString('0.####', $culture),
        $ki.ToString('0.####', $culture),
        $kd.ToString('0.####', $culture),
        $target.ToString('0.####', $culture),
        $limit.ToString($culture))
}

function Read-PositionUiConfig {
    $culture = [System.Globalization.CultureInfo]::InvariantCulture
    $style = [System.Globalization.NumberStyles]::Float
    [single]$kp = 0
    [int32]$target = 0
    [single]$maxSpeed = 0
    [uint16]$limit = 0
    [uint16]$tolerance = 0
    [single]$syncKp = 0
    [single]$syncMax = 0

    $valid = [single]::TryParse(
            $posKpBox.Text, $style, $culture, [ref]$kp) -and
        [int32]::TryParse($posTargetBox.Text, [ref]$target) -and
        [single]::TryParse(
            $posMaxSpeedBox.Text, $style, $culture, [ref]$maxSpeed) -and
        [uint16]::TryParse($posLimitBox.Text, [ref]$limit) -and
        [uint16]::TryParse($posToleranceBox.Text, [ref]$tolerance) -and
        [single]::TryParse(
            $posSyncKpBox.Text, $style, $culture, [ref]$syncKp) -and
        [single]::TryParse(
            $posSyncMaxBox.Text, $style, $culture, [ref]$syncMax)
    if (-not $valid) {
        Add-Log "Position parameter format error." ([System.Drawing.Color]::Firebrick)
        return $null
    }
    if (($kp -le 0) -or ($kp -gt 20) -or
        ($target -eq 0) -or ($target -lt -100000) -or ($target -gt 100000) -or
        ($maxSpeed -lt 100) -or ($maxSpeed -gt 6000) -or
        ($limit -lt 100) -or ($limit -gt 1000) -or
        ($tolerance -lt 1) -or ($tolerance -gt 200) -or
        ($syncKp -lt 0) -or ($syncKp -gt 20) -or
        ($syncMax -lt 0) -or ($syncMax -gt 6000)) {
        Add-Log "Position parameter outside firmware range." ([System.Drawing.Color]::Firebrick)
        return $null
    }

    return ('pos set {0} {1} {2} {3} {4} {5} {6}' -f
        $kp.ToString('0.####', $culture),
        $target.ToString($culture),
        $maxSpeed.ToString('0.####', $culture),
        $limit.ToString($culture),
        $tolerance.ToString($culture),
        $syncKp.ToString('0.####', $culture),
        $syncMax.ToString('0.####', $culture))
}

function Read-YawUiConfig {
    $culture = [System.Globalization.CultureInfo]::InvariantCulture
    $style = [System.Globalization.NumberStyles]::Float
    [single]$kp = 0
    [single]$ki = 0
    [single]$kd = 0
    [single]$target = 0
    [single]$maxSpeed = 0
    [single]$minSpeed = 0
    [uint16]$boost = 0
    [uint16]$limit = 0
    [single]$tolerance = 0
    [single]$settleRate = 0
    [uint16]$settleTime = 0
    [uint16]$timeout = 0

    $valid = [single]::TryParse(
            $yawKpBox.Text, $style, $culture, [ref]$kp) -and
        [single]::TryParse(
            $yawKiBox.Text, $style, $culture, [ref]$ki) -and
        [single]::TryParse(
            $yawKdBox.Text, $style, $culture, [ref]$kd) -and
        [single]::TryParse(
            $yawTargetBox.Text, $style, $culture, [ref]$target) -and
        [single]::TryParse(
            $yawMaxSpeedBox.Text, $style, $culture, [ref]$maxSpeed) -and
        [single]::TryParse(
            $yawMinSpeedBox.Text, $style, $culture, [ref]$minSpeed) -and
        [uint16]::TryParse($yawBoostBox.Text, [ref]$boost) -and
        [uint16]::TryParse($yawLimitBox.Text, [ref]$limit) -and
        [single]::TryParse(
            $yawToleranceBox.Text, $style, $culture, [ref]$tolerance) -and
        [single]::TryParse(
            $yawSettleRateBox.Text, $style, $culture, [ref]$settleRate) -and
        [uint16]::TryParse($yawSettleTimeBox.Text, [ref]$settleTime) -and
        [uint16]::TryParse($yawTimeoutBox.Text, [ref]$timeout)
    if (-not $valid) {
        Add-Log "Yaw parameter format error." ([System.Drawing.Color]::Firebrick)
        return $null
    }
    if (($kp -le 0) -or ($kp -gt 50) -or
        ($ki -lt 0) -or ($ki -gt 20) -or
        ($kd -lt 0) -or ($kd -gt 20) -or
        ($target -eq 0) -or ($target -lt -180) -or ($target -gt 180) -or
        ($maxSpeed -lt 100) -or ($maxSpeed -gt 6000) -or
        ($minSpeed -lt 0) -or ($minSpeed -gt $maxSpeed) -or
        ($boost -gt 300) -or
        ($limit -lt 100) -or ($limit -gt 1000) -or
        ($tolerance -lt 0.1) -or ($tolerance -gt 15) -or
        ($settleRate -lt 0.1) -or ($settleRate -gt 50) -or
        ($settleTime -lt 50) -or ($settleTime -gt 2000) -or
        ($timeout -lt 500) -or ($timeout -gt 15000)) {
        Add-Log "Yaw parameter outside firmware range." ([System.Drawing.Color]::Firebrick)
        return $null
    }

    return ('yaw set {0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11}' -f
        $kp.ToString('0.####', $culture),
        $ki.ToString('0.####', $culture),
        $kd.ToString('0.####', $culture),
        $target.ToString('0.####', $culture),
        $maxSpeed.ToString('0.####', $culture),
        $limit.ToString($culture),
        $tolerance.ToString('0.####', $culture),
        $settleRate.ToString('0.####', $culture),
        $settleTime.ToString($culture),
        $timeout.ToString($culture),
        $minSpeed.ToString('0.####', $culture),
        $boost.ToString($culture))
}

function Read-LineUiConfig {
    $culture = [System.Globalization.CultureInfo]::InvariantCulture
    $style = [System.Globalization.NumberStyles]::Float
    [single]$kp = 0
    [single]$ki = 0
    [single]$kd = 0
    [single]$baseSpeed = 0
    [single]$maxCorrection = 0
    [uint16]$limit = 0
    [single]$deadband = 0
    [uint16]$duration = 0

    $valid = [single]::TryParse(
            $lineKpBox.Text, $style, $culture, [ref]$kp) -and
        [single]::TryParse(
            $lineKiBox.Text, $style, $culture, [ref]$ki) -and
        [single]::TryParse(
            $lineKdBox.Text, $style, $culture, [ref]$kd) -and
        [single]::TryParse(
            $lineBaseBox.Text, $style, $culture, [ref]$baseSpeed) -and
        [single]::TryParse(
            $lineMaxBox.Text, $style, $culture, [ref]$maxCorrection) -and
        [uint16]::TryParse($lineLimitBox.Text, [ref]$limit) -and
        [single]::TryParse(
            $lineDeadbandBox.Text, $style, $culture, [ref]$deadband) -and
        [uint16]::TryParse($lineDurationBox.Text, [ref]$duration)
    if (-not $valid) {
        Add-Log "Line parameter format error." ([System.Drawing.Color]::Firebrick)
        return $null
    }
    if (($kp -le 0) -or ($kp -gt 100) -or
        ($ki -lt 0) -or ($ki -gt 100) -or
        ($kd -lt 0) -or ($kd -gt 20) -or
        ($baseSpeed -lt 100) -or
        (($baseSpeed + $maxCorrection) -gt 6000) -or
        ($maxCorrection -le 0) -or
        ($limit -lt 100) -or ($limit -gt 1000) -or
        ($deadband -lt 0) -or ($deadband -gt 20) -or
        ($duration -lt 500) -or ($duration -gt 10000)) {
        Add-Log "Line parameter outside firmware range." ([System.Drawing.Color]::Firebrick)
        return $null
    }

    return ('line set {0} {1} {2} {3} {4} {5} {6} {7}' -f
        $kp.ToString('0.####', $culture),
        $ki.ToString('0.####', $culture),
        $kd.ToString('0.####', $culture),
        $baseSpeed.ToString('0.####', $culture),
        $maxCorrection.ToString('0.####', $culture),
        $limit.ToString($culture),
        $deadband.ToString('0.####', $culture),
        $duration.ToString($culture))
}

$connectionGroup = New-Object System.Windows.Forms.GroupBox
$connectionGroup.Text = "Connection"
$connectionGroup.Location = New-Object System.Drawing.Point(12, 10)
$connectionGroup.Size = New-Object System.Drawing.Size(736, 72)
$form.Controls.Add($connectionGroup)

$connectionGroup.Controls.Add((New-Label "COM port" 14 29 70))
$portCombo = New-Object System.Windows.Forms.ComboBox
$portCombo.DropDownStyle = [System.Windows.Forms.ComboBoxStyle]::DropDownList
$portCombo.Location = New-Object System.Drawing.Point(85, 27)
$portCombo.Size = New-Object System.Drawing.Size(110, 25)
$connectionGroup.Controls.Add($portCombo)

$refreshButton = New-Object System.Windows.Forms.Button
$refreshButton.Text = "Refresh"
$refreshButton.Location = New-Object System.Drawing.Point(205, 26)
$refreshButton.Size = New-Object System.Drawing.Size(78, 27)
$connectionGroup.Controls.Add($refreshButton)

$connectButton = New-Object System.Windows.Forms.Button
$connectButton.Text = "Connect"
$connectButton.Location = New-Object System.Drawing.Point(293, 26)
$connectButton.Size = New-Object System.Drawing.Size(86, 27)
$connectionGroup.Controls.Add($connectButton)

$connectionGroup.Controls.Add((New-Label "115200 8N1" 390 28 110))
$connectionValue = New-Label "DISCONNECTED" 535 28 180
$connectionValue.ForeColor = [System.Drawing.Color]::Firebrick
$connectionValue.Font = New-Object System.Drawing.Font("Segoe UI Semibold", 9)
$connectionGroup.Controls.Add($connectionValue)

$parameterTabs = New-Object System.Windows.Forms.TabControl
$parameterTabs.Location = New-Object System.Drawing.Point(12, 90)
$parameterTabs.Size = New-Object System.Drawing.Size(736, 154)
$form.Controls.Add($parameterTabs)

$speedTab = New-Object System.Windows.Forms.TabPage
$speedTab.Text = "Speed loop"
$positionTab = New-Object System.Windows.Forms.TabPage
$positionTab.Text = "Position loop"
$yawTab = New-Object System.Windows.Forms.TabPage
$yawTab.Text = "Yaw loop"
$lineTab = New-Object System.Windows.Forms.TabPage
$lineTab.Text = "Line tracking"
[void]$parameterTabs.TabPages.Add($speedTab)
[void]$parameterTabs.TabPages.Add($positionTab)
[void]$parameterTabs.TabPages.Add($yawTab)
[void]$parameterTabs.TabPages.Add($lineTab)
$parameterGroup = $speedTab

$parameterGroup.Controls.Add((New-Label "Kp" 15 23 105))
$kpBox = New-TextBox "0.1200" 15 46 105
$parameterGroup.Controls.Add($kpBox)
$parameterGroup.Controls.Add((New-Label "Ki" 135 23 105))
$kiBox = New-TextBox "0.0500" 135 46 105
$parameterGroup.Controls.Add($kiBox)
$parameterGroup.Controls.Add((New-Label "Kd" 255 23 105))
$kdBox = New-TextBox "0.0000" 255 46 105
$parameterGroup.Controls.Add($kdBox)
$parameterGroup.Controls.Add((New-Label "Target (pps)" 390 23 120))
$targetBox = New-TextBox "3500" 390 46 120
$parameterGroup.Controls.Add($targetBox)
$parameterGroup.Controls.Add((New-Label "Limit" 540 23 80))
$limitBox = New-TextBox "650" 540 46 80
$parameterGroup.Controls.Add($limitBox)

$readButton = New-Object System.Windows.Forms.Button
$readButton.Text = "Read"
$readButton.Location = New-Object System.Drawing.Point(15, 88)
$readButton.Size = New-Object System.Drawing.Size(92, 29)
$parameterGroup.Controls.Add($readButton)

$applyButton = New-Object System.Windows.Forms.Button
$applyButton.Text = "Apply"
$applyButton.Location = New-Object System.Drawing.Point(117, 88)
$applyButton.Size = New-Object System.Drawing.Size(92, 29)
$parameterGroup.Controls.Add($applyButton)

$runButton = New-Object System.Windows.Forms.Button
$runButton.Text = "Run 5 s"
$runButton.Location = New-Object System.Drawing.Point(515, 88)
$runButton.Size = New-Object System.Drawing.Size(98, 29)
$runButton.BackColor = [System.Drawing.Color]::Honeydew
$parameterGroup.Controls.Add($runButton)

$stopButton = New-Object System.Windows.Forms.Button
$stopButton.Text = "Stop"
$stopButton.Location = New-Object System.Drawing.Point(623, 88)
$stopButton.Size = New-Object System.Drawing.Size(98, 29)
$stopButton.BackColor = [System.Drawing.Color]::MistyRose
$parameterGroup.Controls.Add($stopButton)

$positionTab.Controls.Add((New-Label "Kp" 15 23 85))
$posKpBox = New-TextBox "3.0000" 15 46 85
$positionTab.Controls.Add($posKpBox)
$positionTab.Controls.Add((New-Label "Target" 110 23 85))
$posTargetBox = New-TextBox "1060" 110 46 85
$positionTab.Controls.Add($posTargetBox)
$positionTab.Controls.Add((New-Label "Max speed" 205 23 85))
$posMaxSpeedBox = New-TextBox "2000" 205 46 85
$positionTab.Controls.Add($posMaxSpeedBox)
$positionTab.Controls.Add((New-Label "Output" 300 23 85))
$posLimitBox = New-TextBox "650" 300 46 85
$positionTab.Controls.Add($posLimitBox)
$positionTab.Controls.Add((New-Label "Tolerance" 395 23 85))
$posToleranceBox = New-TextBox "24" 395 46 85
$positionTab.Controls.Add($posToleranceBox)
$positionTab.Controls.Add((New-Label "Sync Kp" 490 23 85))
$posSyncKpBox = New-TextBox "2.0000" 490 46 85
$positionTab.Controls.Add($posSyncKpBox)
$positionTab.Controls.Add((New-Label "Sync max" 585 23 105))
$posSyncMaxBox = New-TextBox "400" 585 46 105
$positionTab.Controls.Add($posSyncMaxBox)

$posReadButton = New-Object System.Windows.Forms.Button
$posReadButton.Text = "Read"
$posReadButton.Location = New-Object System.Drawing.Point(15, 88)
$posReadButton.Size = New-Object System.Drawing.Size(92, 29)
$positionTab.Controls.Add($posReadButton)

$posApplyButton = New-Object System.Windows.Forms.Button
$posApplyButton.Text = "Apply"
$posApplyButton.Location = New-Object System.Drawing.Point(117, 88)
$posApplyButton.Size = New-Object System.Drawing.Size(92, 29)
$positionTab.Controls.Add($posApplyButton)

$posRunButton = New-Object System.Windows.Forms.Button
$posRunButton.Text = "Run"
$posRunButton.Location = New-Object System.Drawing.Point(407, 88)
$posRunButton.Size = New-Object System.Drawing.Size(98, 29)
$posRunButton.BackColor = [System.Drawing.Color]::Honeydew
$positionTab.Controls.Add($posRunButton)

$posStressButton = New-Object System.Windows.Forms.Button
$posStressButton.Text = "Stress 24"
$posStressButton.Location = New-Object System.Drawing.Point(515, 88)
$posStressButton.Size = New-Object System.Drawing.Size(98, 29)
$posStressButton.BackColor = [System.Drawing.Color]::LightCyan
$positionTab.Controls.Add($posStressButton)

$posStopButton = New-Object System.Windows.Forms.Button
$posStopButton.Text = "Stop"
$posStopButton.Location = New-Object System.Drawing.Point(623, 88)
$posStopButton.Size = New-Object System.Drawing.Size(98, 29)
$posStopButton.BackColor = [System.Drawing.Color]::MistyRose
$positionTab.Controls.Add($posStopButton)

$yawColumns = @(12, 130, 248, 366, 484, 602)
$yawLabelsTop = @('Kp', 'Ki', 'Kd', 'Target (deg)', 'Max speed', 'Min speed')
$yawDefaultsTop = @('45.0000', '0.8000', '7.0000', '-45', '800', '200')
$yawTopBoxes = @()
for ($index = 0; $index -lt $yawColumns.Count; $index++) {
    $yawTab.Controls.Add((New-Label $yawLabelsTop[$index] $yawColumns[$index] 2 105))
    $box = New-TextBox $yawDefaultsTop[$index] $yawColumns[$index] 21 105
    $yawTab.Controls.Add($box)
    $yawTopBoxes += $box
}
$yawKpBox = $yawTopBoxes[0]
$yawKiBox = $yawTopBoxes[1]
$yawKdBox = $yawTopBoxes[2]
$yawTargetBox = $yawTopBoxes[3]
$yawMaxSpeedBox = $yawTopBoxes[4]
$yawMinSpeedBox = $yawTopBoxes[5]

$yawBottomColumns = $yawColumns
$yawLabelsBottom = @(
    'PWM limit', 'Tolerance (deg)', 'Settle rate', 'Settle (ms)',
    'Timeout (ms)', 'FF boost')
$yawDefaultsBottom = @('750', '0.7000', '5.0000', '300', '5000', '40')
$yawBottomBoxes = @()
for ($index = 0; $index -lt $yawBottomColumns.Count; $index++) {
    $yawTab.Controls.Add((New-Label $yawLabelsBottom[$index] $yawBottomColumns[$index] 47 105))
    $box = New-TextBox $yawDefaultsBottom[$index] $yawBottomColumns[$index] 66 105
    $yawTab.Controls.Add($box)
    $yawBottomBoxes += $box
}
$yawLimitBox = $yawBottomBoxes[0]
$yawToleranceBox = $yawBottomBoxes[1]
$yawSettleRateBox = $yawBottomBoxes[2]
$yawSettleTimeBox = $yawBottomBoxes[3]
$yawTimeoutBox = $yawBottomBoxes[4]
$yawBoostBox = $yawBottomBoxes[5]

$yawReadButton = New-Object System.Windows.Forms.Button
$yawReadButton.Text = 'Read'
$yawReadButton.Location = New-Object System.Drawing.Point(15, 96)
$yawReadButton.Size = New-Object System.Drawing.Size(92, 27)
$yawTab.Controls.Add($yawReadButton)

$yawApplyButton = New-Object System.Windows.Forms.Button
$yawApplyButton.Text = 'Apply'
$yawApplyButton.Location = New-Object System.Drawing.Point(117, 96)
$yawApplyButton.Size = New-Object System.Drawing.Size(92, 27)
$yawTab.Controls.Add($yawApplyButton)

$yawRunButton = New-Object System.Windows.Forms.Button
$yawRunButton.Text = 'Run'
$yawRunButton.Location = New-Object System.Drawing.Point(515, 96)
$yawRunButton.Size = New-Object System.Drawing.Size(98, 27)
$yawRunButton.BackColor = [System.Drawing.Color]::Honeydew
$yawTab.Controls.Add($yawRunButton)

$yawStopButton = New-Object System.Windows.Forms.Button
$yawStopButton.Text = 'Stop'
$yawStopButton.Location = New-Object System.Drawing.Point(623, 96)
$yawStopButton.Size = New-Object System.Drawing.Size(98, 27)
$yawStopButton.BackColor = [System.Drawing.Color]::MistyRose
$yawTab.Controls.Add($yawStopButton)

$lineColumns = @(12, 190, 368, 546)
$lineLabelsTop = @('Kp', 'Ki', 'Kd', 'Base speed (pps)')
$lineDefaultsTop = @('30.0000', '0.0000', '0.0000', '1400')
$lineTopBoxes = @()
for ($index = 0; $index -lt $lineColumns.Count; $index++) {
    $lineTab.Controls.Add((New-Label $lineLabelsTop[$index] $lineColumns[$index] 2 160))
    $box = New-TextBox $lineDefaultsTop[$index] $lineColumns[$index] 21 160
    $lineTab.Controls.Add($box)
    $lineTopBoxes += $box
}
$lineKpBox = $lineTopBoxes[0]
$lineKiBox = $lineTopBoxes[1]
$lineKdBox = $lineTopBoxes[2]
$lineBaseBox = $lineTopBoxes[3]

$lineLabelsBottom = @(
    'Max correction (pps)', 'PWM limit', 'Deadband', 'Duration (ms)')
$lineDefaultsBottom = @('900', '750', '2.0000', '30000')
$lineBottomBoxes = @()
for ($index = 0; $index -lt $lineColumns.Count; $index++) {
    $lineTab.Controls.Add((New-Label $lineLabelsBottom[$index] $lineColumns[$index] 47 160))
    $box = New-TextBox $lineDefaultsBottom[$index] $lineColumns[$index] 66 160
    $lineTab.Controls.Add($box)
    $lineBottomBoxes += $box
}
$lineMaxBox = $lineBottomBoxes[0]
$lineLimitBox = $lineBottomBoxes[1]
$lineDeadbandBox = $lineBottomBoxes[2]
$lineDurationBox = $lineBottomBoxes[3]

$lineReadButton = New-Object System.Windows.Forms.Button
$lineReadButton.Text = 'Read'
$lineReadButton.Location = New-Object System.Drawing.Point(8, 96)
$lineReadButton.Size = New-Object System.Drawing.Size(92, 27)
$lineTab.Controls.Add($lineReadButton)

$lineApplyButton = New-Object System.Windows.Forms.Button
$lineApplyButton.Text = 'Apply'
$lineApplyButton.Location = New-Object System.Drawing.Point(110, 96)
$lineApplyButton.Size = New-Object System.Drawing.Size(92, 27)
$lineTab.Controls.Add($lineApplyButton)

$lineCalButton = New-Object System.Windows.Forms.Button
$lineCalButton.Text = 'Calibrate'
$lineCalButton.Location = New-Object System.Drawing.Point(212, 96)
$lineCalButton.Size = New-Object System.Drawing.Size(92, 27)
$lineTab.Controls.Add($lineCalButton)

$lineRunButton = New-Object System.Windows.Forms.Button
$lineRunButton.Text = '30s Test'
$lineRunButton.Location = New-Object System.Drawing.Point(314, 96)
$lineRunButton.Size = New-Object System.Drawing.Size(92, 27)
$lineRunButton.BackColor = [System.Drawing.Color]::Honeydew
$lineTab.Controls.Add($lineRunButton)

$lineStopButton = New-Object System.Windows.Forms.Button
$lineStopButton.Text = 'Test Stop'
$lineStopButton.Location = New-Object System.Drawing.Point(416, 96)
$lineStopButton.Size = New-Object System.Drawing.Size(92, 27)
$lineStopButton.BackColor = [System.Drawing.Color]::MistyRose
$lineTab.Controls.Add($lineStopButton)

$missionStartButton = New-Object System.Windows.Forms.Button
$missionStartButton.Text = 'Mission'
$missionStartButton.Location = New-Object System.Drawing.Point(518, 96)
$missionStartButton.Size = New-Object System.Drawing.Size(92, 27)
$missionStartButton.BackColor = [System.Drawing.Color]::Honeydew
$lineTab.Controls.Add($missionStartButton)

$missionStopButton = New-Object System.Windows.Forms.Button
$missionStopButton.Text = 'Mission Stop'
$missionStopButton.Location = New-Object System.Drawing.Point(620, 96)
$missionStopButton.Size = New-Object System.Drawing.Size(98, 27)
$missionStopButton.BackColor = [System.Drawing.Color]::MistyRose
$lineTab.Controls.Add($missionStopButton)

$statusGroup = New-Object System.Windows.Forms.GroupBox
$statusGroup.Text = "Live status"
$statusGroup.Location = New-Object System.Drawing.Point(12, 252)
$statusGroup.Size = New-Object System.Drawing.Size(736, 142)
$form.Controls.Add($statusGroup)

$statusGroup.Controls.Add((New-Label "State" 15 25 70))
$stateValue = New-Label "--" 90 25 120
$stateValue.Font = New-Object System.Drawing.Font("Consolas", 10, [System.Drawing.FontStyle]::Bold)
$statusGroup.Controls.Add($stateValue)
$statusGroup.Controls.Add((New-Label "Motor" 245 25 70))
$highZValue = New-Label "--" 320 25 100
$highZValue.Font = New-Object System.Drawing.Font("Consolas", 10, [System.Drawing.FontStyle]::Bold)
$statusGroup.Controls.Add($highZValue)
$statusGroup.Controls.Add((New-Label "Result" 515 25 70))
$resultValue = New-Label "--" 590 25 100
$statusGroup.Controls.Add($resultValue)

$statusGroup.Controls.Add((New-Label "Left speed" 15 57 85))
$leftSpeedValue = New-Label "--" 105 57 115
$leftSpeedValue.Font = New-Object System.Drawing.Font("Consolas", 10)
$statusGroup.Controls.Add($leftSpeedValue)
$statusGroup.Controls.Add((New-Label "Right speed" 245 57 90))
$rightSpeedValue = New-Label "--" 340 57 115
$rightSpeedValue.Font = New-Object System.Drawing.Font("Consolas", 10)
$statusGroup.Controls.Add($rightSpeedValue)
$statusGroup.Controls.Add((New-Label "Invalid L/R" 515 57 85))
$leftInvalidValue = New-Label "--" 605 57 45
$rightInvalidValue = New-Label "--" 660 57 55
$statusGroup.Controls.Add($leftInvalidValue)
$statusGroup.Controls.Add($rightInvalidValue)

$statusGroup.Controls.Add((New-Label "Left output" 15 89 85))
$leftOutputValue = New-Label "--" 105 89 115
$leftOutputValue.Font = New-Object System.Drawing.Font("Consolas", 10)
$statusGroup.Controls.Add($leftOutputValue)
$statusGroup.Controls.Add((New-Label "Right output" 245 89 90))
$rightOutputValue = New-Label "--" 340 89 115
$rightOutputValue.Font = New-Object System.Drawing.Font("Consolas", 10)
$statusGroup.Controls.Add($rightOutputValue)

$positionStatusGroup = New-Object System.Windows.Forms.GroupBox
$positionStatusGroup.Text = "Position live status"
$positionStatusGroup.Location = New-Object System.Drawing.Point(12, 252)
$positionStatusGroup.Size = New-Object System.Drawing.Size(736, 142)
$positionStatusGroup.Visible = $false
$form.Controls.Add($positionStatusGroup)

$positionStatusGroup.Controls.Add((New-Label "State" 15 20 55))
$posStateValue = New-Label "--" 70 20 85
$posStateValue.Font = New-Object System.Drawing.Font("Consolas", 10, [System.Drawing.FontStyle]::Bold)
$positionStatusGroup.Controls.Add($posStateValue)
$positionStatusGroup.Controls.Add((New-Label "Progress" 165 20 65))
$posStepValue = New-Label "--" 230 20 205
$posStepValue.Font = New-Object System.Drawing.Font("Consolas", 9)
$positionStatusGroup.Controls.Add($posStepValue)
$positionStatusGroup.Controls.Add((New-Label "Motor" 445 20 55))
$posHighZValue = New-Label "--" 500 20 85
$posHighZValue.Font = New-Object System.Drawing.Font("Consolas", 10, [System.Drawing.FontStyle]::Bold)
$positionStatusGroup.Controls.Add($posHighZValue)
$positionStatusGroup.Controls.Add((New-Label "Result" 590 20 55))
$posResultValue = New-Label "--" 645 20 65
$positionStatusGroup.Controls.Add($posResultValue)

$positionStatusGroup.Controls.Add((New-Label "Left  target" 15 49 85))
$posLeftTargetValue = New-Label "--" 100 49 85
$positionStatusGroup.Controls.Add($posLeftTargetValue)
$positionStatusGroup.Controls.Add((New-Label "actual" 190 49 50))
$posLeftActualValue = New-Label "--" 240 49 85
$positionStatusGroup.Controls.Add($posLeftActualValue)
$positionStatusGroup.Controls.Add((New-Label "error" 335 49 50))
$posLeftErrorValue = New-Label "--" 385 49 75
$positionStatusGroup.Controls.Add($posLeftErrorValue)
$positionStatusGroup.Controls.Add((New-Label "Worst" 475 49 50))
$posWorstValue = New-Label "--" 525 49 150
$positionStatusGroup.Controls.Add($posWorstValue)

$positionStatusGroup.Controls.Add((New-Label "Right target" 15 76 85))
$posRightTargetValue = New-Label "--" 100 76 85
$positionStatusGroup.Controls.Add($posRightTargetValue)
$positionStatusGroup.Controls.Add((New-Label "actual" 190 76 50))
$posRightActualValue = New-Label "--" 240 76 85
$positionStatusGroup.Controls.Add($posRightActualValue)
$positionStatusGroup.Controls.Add((New-Label "error" 335 76 50))
$posRightErrorValue = New-Label "--" 385 76 75
$positionStatusGroup.Controls.Add($posRightErrorValue)
$positionStatusGroup.Controls.Add((New-Label "Speed L/R" 475 76 75))
$posSpeedValue = New-Label "--" 550 76 150
$positionStatusGroup.Controls.Add($posSpeedValue)

$positionStatusGroup.Controls.Add((New-Label "Invalid L/R" 15 105 85))
$posInvalidValue = New-Label "--" 100 105 120
$positionStatusGroup.Controls.Add($posInvalidValue)
$positionStatusGroup.Controls.Add((New-Label "Recovery L/R" 250 105 95))
$posRecoveryValue = New-Label "--" 345 105 120
$positionStatusGroup.Controls.Add($posRecoveryValue)

$yawStatusGroup = New-Object System.Windows.Forms.GroupBox
$yawStatusGroup.Text = 'Yaw live status'
$yawStatusGroup.Location = New-Object System.Drawing.Point(12, 252)
$yawStatusGroup.Size = New-Object System.Drawing.Size(736, 142)
$yawStatusGroup.Visible = $false
$form.Controls.Add($yawStatusGroup)

$yawStatusGroup.Controls.Add((New-Label 'State' 15 20 50))
$yawStateValue = New-Label '--' 65 20 100
$yawStateValue.Font = New-Object System.Drawing.Font(
    'Consolas', 10, [System.Drawing.FontStyle]::Bold)
$yawStatusGroup.Controls.Add($yawStateValue)
$yawStatusGroup.Controls.Add((New-Label 'Motor' 260 20 55))
$yawHighZValue = New-Label '--' 315 20 90
$yawHighZValue.Font = New-Object System.Drawing.Font(
    'Consolas', 10, [System.Drawing.FontStyle]::Bold)
$yawStatusGroup.Controls.Add($yawHighZValue)
$yawStatusGroup.Controls.Add((New-Label 'Result' 520 20 55))
$yawResultValue = New-Label '--' 580 20 80
$yawStatusGroup.Controls.Add($yawResultValue)

$yawStatusGroup.Controls.Add((New-Label 'Target' 15 51 50))
$yawTargetValue = New-Label '--' 65 51 105
$yawStatusGroup.Controls.Add($yawTargetValue)
$yawStatusGroup.Controls.Add((New-Label 'Current' 180 51 60))
$yawCurrentValue = New-Label '--' 240 51 110
$yawStatusGroup.Controls.Add($yawCurrentValue)
$yawStatusGroup.Controls.Add((New-Label 'Error' 360 51 50))
$yawErrorValue = New-Label '--' 410 51 105
$yawStatusGroup.Controls.Add($yawErrorValue)
$yawStatusGroup.Controls.Add((New-Label 'Rate' 525 51 45))
$yawRateValue = New-Label '--' 570 51 130
$yawStatusGroup.Controls.Add($yawRateValue)

$yawStatusGroup.Controls.Add((New-Label 'Turn' 15 84 45))
$yawTurnValue = New-Label '--' 60 84 105
$yawStatusGroup.Controls.Add($yawTurnValue)
$yawStatusGroup.Controls.Add((New-Label 'Target L/R' 175 84 70))
$yawTargetSpeedValue = New-Label '--' 245 84 130
$yawStatusGroup.Controls.Add($yawTargetSpeedValue)
$yawStatusGroup.Controls.Add((New-Label 'Speed L/R' 385 84 65))
$yawSpeedValue = New-Label '--' 450 84 120
$yawStatusGroup.Controls.Add($yawSpeedValue)
$yawStatusGroup.Controls.Add((New-Label 'Output L/R' 575 84 70))
$yawOutputValue = New-Label '--' 645 84 80
$yawStatusGroup.Controls.Add($yawOutputValue)

$lineStatusGroup = New-Object System.Windows.Forms.GroupBox
$lineStatusGroup.Text = 'Line tracking live status'
$lineStatusGroup.Location = New-Object System.Drawing.Point(12, 252)
$lineStatusGroup.Size = New-Object System.Drawing.Size(736, 142)
$lineStatusGroup.Visible = $false
$form.Controls.Add($lineStatusGroup)

$lineStatusGroup.Controls.Add((New-Label 'State' 15 20 45))
$lineStateValue = New-Label '--' 60 20 75
$lineStateValue.Font = New-Object System.Drawing.Font(
    'Consolas', 10, [System.Drawing.FontStyle]::Bold)
$lineStatusGroup.Controls.Add($lineStateValue)
$lineStatusGroup.Controls.Add((New-Label 'Sensor' 145 20 50))
$lineSensorStateValue = New-Label '--' 195 20 80
$lineStatusGroup.Controls.Add($lineSensorStateValue)
$lineStatusGroup.Controls.Add((New-Label 'Motor' 360 20 50))
$lineHighZValue = New-Label '--' 410 20 85
$lineHighZValue.Font = New-Object System.Drawing.Font(
    'Consolas', 10, [System.Drawing.FontStyle]::Bold)
$lineStatusGroup.Controls.Add($lineHighZValue)
$lineStatusGroup.Controls.Add((New-Label 'Result' 575 20 50))
$lineResultValue = New-Label '--' 625 20 70
$lineStatusGroup.Controls.Add($lineResultValue)

$lineStatusGroup.Controls.Add((New-Label 'Raw' 15 51 35))
$lineRawValue = New-Label '--' 50 51 65
$lineStatusGroup.Controls.Add($lineRawValue)
$lineStatusGroup.Controls.Add((New-Label 'Mask/count' 125 51 75))
$lineBitsValue = New-Label '--' 200 51 120
$lineBitsValue.Font = New-Object System.Drawing.Font('Consolas', 9)
$lineStatusGroup.Controls.Add($lineBitsValue)
$lineStatusGroup.Controls.Add((New-Label 'Error' 335 51 40))
$lineErrorValue = New-Label '--' 375 51 55
$lineStatusGroup.Controls.Add($lineErrorValue)
$lineStatusGroup.Controls.Add((New-Label 'Correction' 455 51 70))
$lineCorrectionValue = New-Label '--' 525 51 120
$lineStatusGroup.Controls.Add($lineCorrectionValue)

$lineStatusGroup.Controls.Add((New-Label 'Target L/R' 15 84 70))
$lineTargetValue = New-Label '--' 85 84 135
$lineStatusGroup.Controls.Add($lineTargetValue)
$lineStatusGroup.Controls.Add((New-Label 'Speed L/R' 235 84 65))
$lineSpeedValue = New-Label '--' 300 84 135
$lineStatusGroup.Controls.Add($lineSpeedValue)
$lineStatusGroup.Controls.Add((New-Label 'Output L/R' 450 84 70))
$lineOutputValue = New-Label '--' 520 84 100
$lineStatusGroup.Controls.Add($lineOutputValue)
$lineStatusGroup.Controls.Add((New-Label 'Age' 630 84 30))
$lineAgeValue = New-Label '--' 660 84 60
$lineStatusGroup.Controls.Add($lineAgeValue)

$logGroup = New-Object System.Windows.Forms.GroupBox
$logGroup.Text = "Command log"
$logGroup.Location = New-Object System.Drawing.Point(12, 402)
$logGroup.Size = New-Object System.Drawing.Size(736, 218)
$logGroup.Anchor = [System.Windows.Forms.AnchorStyles]::Top -bor
    [System.Windows.Forms.AnchorStyles]::Bottom -bor
    [System.Windows.Forms.AnchorStyles]::Left -bor
    [System.Windows.Forms.AnchorStyles]::Right
$form.Controls.Add($logGroup)

$logBox = New-Object System.Windows.Forms.RichTextBox
$logBox.Location = New-Object System.Drawing.Point(10, 23)
$logBox.Size = New-Object System.Drawing.Size(716, 184)
$logBox.Anchor = [System.Windows.Forms.AnchorStyles]::Top -bor
    [System.Windows.Forms.AnchorStyles]::Bottom -bor
    [System.Windows.Forms.AnchorStyles]::Left -bor
    [System.Windows.Forms.AnchorStyles]::Right
$logBox.ReadOnly = $true
$logBox.Font = New-Object System.Drawing.Font("Consolas", 9)
$logBox.BackColor = [System.Drawing.Color]::White
$logGroup.Controls.Add($logBox)

$refreshButton.Add_Click({ Refresh-Ports })
$connectButton.Add_Click({
    if (($null -ne $script:serial) -and $script:serial.IsOpen) {
        $script:autoReconnect = $false
        Disconnect-Serial
        return
    }
    if ($portCombo.Text -eq "") {
        Add-Log "Select a COM port." ([System.Drawing.Color]::Firebrick)
        return
    }
    try {
        $script:serial = New-Object System.IO.Ports.SerialPort(
            $portCombo.Text, $script:baudRate,
            [System.IO.Ports.Parity]::None, 8,
            [System.IO.Ports.StopBits]::One)
        $script:serial.Encoding = [System.Text.Encoding]::ASCII
        $script:serial.DtrEnable = $false
        $script:serial.RtsEnable = $false
        $script:serial.ReadTimeout = 100
        $script:serial.WriteTimeout = 1000
        $script:serial.Open()
        $script:serial.DiscardInBuffer()
        $script:serial.DiscardOutBuffer()
        $script:rxBuffer = ""
        $connectButton.Text = "Disconnect"
        $connectionValue.Text = $portCombo.Text
        $connectionValue.ForeColor = [System.Drawing.Color]::ForestGreen
        [System.IO.File]::WriteAllText(
            $script:lastPortPath, $portCombo.Text, [System.Text.Encoding]::ASCII)
        Add-Log "Connected to $($portCombo.Text) at $script:baudRate." ([System.Drawing.Color]::ForestGreen)
        $script:connectionReadyAt = [DateTime]::Now.AddMilliseconds(750)
        $script:lastStatusPoll = [DateTime]::Now
        $script:initialReadPending = $true
        $script:primePending = $true
    } catch {
        Add-Log "Connect error: $($_.Exception.Message)" ([System.Drawing.Color]::Firebrick)
        Disconnect-Serial
    }
})

$readButton.Add_Click({ [void](Send-Command "spd get") })
$applyButton.Add_Click({
    $command = Read-UiConfig
    if ($null -ne $command) {
        [void](Send-Command $command)
    }
})
$runButton.Add_Click({
    if (Send-Command "spd run") {
        Start-RunCapture
    }
})
$stopButton.Add_Click({ [void](Send-Command "spd stop") })

$posReadButton.Add_Click({ [void](Send-Command "pos get") })
$posApplyButton.Add_Click({
    $command = Read-PositionUiConfig
    if ($null -ne $command) {
        [void](Send-Command $command)
    }
})
$posRunButton.Add_Click({ [void](Send-Command "pos run") })
$posStressButton.Add_Click({ [void](Send-Command "pos run stress") })
$posStopButton.Add_Click({ [void](Send-Command "pos stop") })

$yawReadButton.Add_Click({ [void](Send-Command 'yaw get') })
$yawApplyButton.Add_Click({
    $command = Read-YawUiConfig
    if ($null -ne $command) {
        [void](Send-Command $command)
    }
})
$yawRunButton.Add_Click({ [void](Send-Command 'yaw run') })
$yawStopButton.Add_Click({ [void](Send-Command 'yaw stop') })

$lineReadButton.Add_Click({ [void](Send-Command 'line get') })
$lineApplyButton.Add_Click({
    $command = Read-LineUiConfig
    if ($null -ne $command) {
        [void](Send-Command $command)
    }
})
$lineCalButton.Add_Click({ [void](Send-Command 'line cal') })
$lineRunButton.Add_Click({
    $script:lineStatusSource = 'test'
    [void](Send-Command 'line run')
})
$lineStopButton.Add_Click({
    $script:lineStatusSource = 'test'
    [void](Send-Command 'line stop')
})
$missionStartButton.Add_Click({
    $script:lineStatusSource = 'mission'
    [void](Send-Command 'mission start')
})
$missionStopButton.Add_Click({
    $script:lineStatusSource = 'mission'
    [void](Send-Command 'mission stop')
})

$parameterTabs.Add_SelectedIndexChanged({
    $isPosition = ($parameterTabs.SelectedTab -eq $positionTab)
    $isYaw = ($parameterTabs.SelectedTab -eq $yawTab)
    $isLine = ($parameterTabs.SelectedTab -eq $lineTab)
    $script:activeMode = if ($isPosition) {
        'position'
    } elseif ($isYaw) {
        'yaw'
    } elseif ($isLine) {
        'line'
    } else {
        'speed'
    }
    $statusGroup.Visible = (-not $isPosition) -and
        (-not $isYaw) -and (-not $isLine)
    $positionStatusGroup.Visible = $isPosition
    $yawStatusGroup.Visible = $isYaw
    $lineStatusGroup.Visible = $isLine
    $script:lastStatusPoll = [DateTime]::Now
    if (($null -ne $script:serial) -and $script:serial.IsOpen) {
        if ($isPosition) {
            [void](Send-Command "pos get")
            [void](Send-Command "pos stat" $true)
        } elseif ($isYaw) {
            [void](Send-Command 'yaw get')
            [void](Send-Command 'yaw stat' $true)
        } elseif ($isLine) {
            [void](Send-Command 'line get')
            [void](Send-Command 'line stat' $true)
            [void](Send-Command 'mission stat' $true)
        } else {
            [void](Send-Command "spd get")
            [void](Send-Command "spd stat" $true)
        }
    }
})

$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 80
$timer.Add_Tick({
    Accept-TcpClients
    Process-ControlClients
    if (($null -eq $script:serial) -or (-not $script:serial.IsOpen)) {
        $now = [DateTime]::Now
        if ($script:autoReconnect -and
            ($now -ge $script:nextAutoConnectAt)) {
            $script:nextAutoConnectAt = $now.AddSeconds(3)
            $connectButton.PerformClick()
        }
        return
    }
    try {
        if ($script:serial.BytesToRead -gt 0) {
            $script:rxBuffer += $script:serial.ReadExisting()
            while (($newline = $script:rxBuffer.IndexOf("`n")) -ge 0) {
                $line = $script:rxBuffer.Substring(0, $newline).TrimEnd("`r")
                $script:rxBuffer = $script:rxBuffer.Substring($newline + 1)
                Process-Line $line
            }
        }
        $now = [DateTime]::Now
        if ($now -lt $script:connectionReadyAt) { return }
        if ($script:primePending) {
            $script:serial.Write("`n")
            $script:primePending = $false
            $script:connectionReadyAt = $now.AddMilliseconds(200)
            return
        }
        if ($script:initialReadPending) {
            $script:serial.DiscardInBuffer()
            if ($script:activeMode -eq "position") {
                [void](Send-Command "pos get")
            } elseif ($script:activeMode -eq 'yaw') {
                [void](Send-Command 'yaw get')
            } elseif ($script:activeMode -eq 'line') {
                [void](Send-Command 'line get')
            } else {
                [void](Send-Command "spd get")
            }
            $script:initialReadPending = $false
        }
        $externalControlActive =
            (($now - $script:lastExternalCommandAt).TotalMilliseconds -lt 900)
        $statusPollMs = if ($script:activeMode -in @('position', 'yaw', 'line')) {
            600
        } else {
            400
        }
        if ((-not $externalControlActive) -and
            (($now - $script:lastStatusPoll).TotalMilliseconds -ge $statusPollMs)) {
            if ($script:activeMode -eq "position") {
                [void](Send-Command "pos stat" $true)
            } elseif ($script:activeMode -eq 'yaw') {
                [void](Send-Command 'yaw stat' $true)
            } elseif ($script:activeMode -eq 'line') {
                [void](Send-Command 'line stat' $true)
                [void](Send-Command 'mission stat' $true)
            } else {
                [void](Send-Command "spd stat" $true)
            }
            $script:lastStatusPoll = $now
        }
    } catch {
        Add-Log "Serial error: $($_.Exception.Message)" ([System.Drawing.Color]::Firebrick)
        Disconnect-Serial
    }
})
Start-TcpBridge
$timer.Start()

$form.Add_FormClosing({
    $timer.Stop()
    Disconnect-Serial
    Stop-TcpBridge
})

Refresh-Ports
if ($StartYawMode) {
    $parameterTabs.SelectedTab = $yawTab
    $script:activeMode = 'yaw'
    $statusGroup.Visible = $false
    $positionStatusGroup.Visible = $false
    $yawStatusGroup.Visible = $true
    $lineStatusGroup.Visible = $false
}
if ($StartLineMode) {
    $parameterTabs.SelectedTab = $lineTab
    $script:activeMode = 'line'
    $statusGroup.Visible = $false
    $positionStatusGroup.Visible = $false
    $yawStatusGroup.Visible = $false
    $lineStatusGroup.Visible = $true
}
if ($StartMinimized) {
    $form.WindowState = [System.Windows.Forms.FormWindowState]::Minimized
}
if ($AutoConnect) {
    $form.Add_Shown({ $connectButton.PerformClick() })
}
[void]$form.ShowDialog()
