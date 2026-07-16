[CmdletBinding()]
param(
    [switch]$AutoConnect,
    [switch]$StartMinimized
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
$script:baudRate = 9600
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
$script:lastPortPath = Join-Path $script:runtimeDir "last_port.txt"
$script:sessionLogPath = Join-Path $script:runtimeDir (
    "session_{0}.log" -f [DateTime]::Now.ToString("yyyyMMdd_HHmmss"))
$script:telemetryCsvPath = Join-Path $script:runtimeDir (
    "telemetry_{0}.csv" -f [DateTime]::Now.ToString("yyyyMMdd_HHmmss"))
$script:positionTelemetryCsvPath = Join-Path $script:runtimeDir (
    "position_telemetry_{0}.csv" -f [DateTime]::Now.ToString("yyyyMMdd_HHmmss"))
$telemetryHeader = "timestamp,left_target_pps,left_speed_pps,right_target_pps,right_speed_pps,left_output_permille,right_output_permille`r`n"
$positionTelemetryHeader = "timestamp,left_target_count,left_count,right_target_count,right_count,left_speed_target_pps,right_speed_target_pps`r`n"
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
        $line -match '^pos run(?: stress)?$') {
        return $true
    }
    if ($line -match '^spd set [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+$') {
        return $true
    }
    return $line -match '^pos set [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?\d+ [+-]?(?:\d+(?:\.\d*)?|\.\d+) \d+ \d+(?: [+-]?(?:\d+(?:\.\d*)?|\.\d+) [+-]?(?:\d+(?:\.\d*)?|\.\d+))?$'
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
                "BRIDGE READY control=$script:controlPort wave=6`n"))
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
    Broadcast-ControlLine $line
    if ($line.StartsWith('OK CFG ')) {
        Update-ConfigFromLine $line
    } elseif ($line.StartsWith('OK PCFG ')) {
        Update-PositionConfigFromLine $line
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

$connectionGroup.Controls.Add((New-Label "9600 8N1" 400 28 100))
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
[void]$parameterTabs.TabPages.Add($speedTab)
[void]$parameterTabs.TabPages.Add($positionTab)
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

$parameterTabs.Add_SelectedIndexChanged({
    $isPosition = ($parameterTabs.SelectedTab -eq $positionTab)
    $script:activeMode = if ($isPosition) { "position" } else { "speed" }
    $statusGroup.Visible = -not $isPosition
    $positionStatusGroup.Visible = $isPosition
    $script:lastStatusPoll = [DateTime]::Now
    if (($null -ne $script:serial) -and $script:serial.IsOpen) {
        if ($isPosition) {
            [void](Send-Command "pos get")
            [void](Send-Command "pos stat" $true)
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
    if (($null -eq $script:serial) -or (-not $script:serial.IsOpen)) { return }
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
            } else {
                [void](Send-Command "spd get")
            }
            $script:initialReadPending = $false
        }
        $externalControlActive =
            (($now - $script:lastExternalCommandAt).TotalMilliseconds -lt 900)
        $statusPollMs = if ($script:activeMode -eq "position") { 600 } else { 400 }
        if ((-not $externalControlActive) -and
            (($now - $script:lastStatusPoll).TotalMilliseconds -ge $statusPollMs)) {
            if ($script:activeMode -eq "position") {
                [void](Send-Command "pos stat" $true)
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
if ($StartMinimized) {
    $form.WindowState = [System.Windows.Forms.FormWindowState]::Minimized
}
if ($AutoConnect) {
    $form.Add_Shown({ $connectButton.PerformClick() })
}
[void]$form.ShowDialog()
