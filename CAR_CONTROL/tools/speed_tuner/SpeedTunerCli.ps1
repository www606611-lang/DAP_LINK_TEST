[CmdletBinding()]
param(
    [string]$Port = "COM6",
    [ValidateSet(
        "Get", "Set", "Run", "Step", "Reverse", "Sweep", "Lease",
        "Stop", "Status", "PositionGet", "PositionSet", "PositionRun",
        "PositionStress", "PositionStop", "PositionStatus")]
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
    [uint32]$PollMs = 300,
    [uint32]$RunTimeoutMs = 15000
)

$ErrorActionPreference = "Stop"
$baudRate = 9600
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
            if ($line.StartsWith("wave:")) { continue }
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

if ($Takeover) {
    Stop-GuiOwner
}

try {
    $runCommand = $null
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
} finally {
    Close-Bridge
    if ($null -ne $serial) {
        if ($serial.IsOpen) { $serial.Close() }
        $serial.Dispose()
    }
}
