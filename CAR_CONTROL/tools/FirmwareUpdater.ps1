[CmdletBinding()]
param(
    [string]$Port = "COM6",
    [int]$BaudRate = 115200,
    [string]$Image = "",
    [switch]$SkipEnter,
    [switch]$NoRun,
    [switch]$ValidateOnly,
    [int]$StopAfterBytes = 0
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$Magic0 = [byte]0x55
$Magic1 = [byte]0xAA
$ProtocolVersion = [byte]1
$CommandPing = [byte]1
$CommandBegin = [byte]2
$CommandData = [byte]3
$CommandEnd = [byte]4
$CommandRun = [byte]5
$ChunkSize = 1024

if (-not ("FirmwareCrc32" -as [type])) {
    Add-Type -TypeDefinition @"
using System;

public static class FirmwareCrc32
{
    public static UInt32 Compute(byte[] data)
    {
        UInt32 crc = 0xFFFFFFFFu;
        foreach (byte value in data)
        {
            crc ^= value;
            for (int bit = 0; bit < 8; bit++)
            {
                crc = (crc >> 1) ^ ((crc & 1u) != 0u ? 0xEDB88320u : 0u);
            }
        }
        return crc ^ 0xFFFFFFFFu;
    }
}
"@
}

function Get-Crc32([byte[]]$Data) {
    return [FirmwareCrc32]::Compute($Data)
}

function Set-U16([byte[]]$Buffer, [int]$Offset, [uint16]$Value) {
    $Buffer[$Offset] = [byte]($Value -band 0xFF)
    $Buffer[$Offset + 1] = [byte](($Value -shr 8) -band 0xFF)
}

function Set-U32([byte[]]$Buffer, [int]$Offset, [uint32]$Value) {
    $Buffer[$Offset] = [byte]($Value -band 0xFF)
    $Buffer[$Offset + 1] = [byte](($Value -shr 8) -band 0xFF)
    $Buffer[$Offset + 2] = [byte](($Value -shr 16) -band 0xFF)
    $Buffer[$Offset + 3] = [byte](($Value -shr 24) -band 0xFF)
}

function Get-U16([byte[]]$Buffer, [int]$Offset) {
    return [uint16]([uint16]$Buffer[$Offset] -bor
        ([uint16]$Buffer[$Offset + 1] -shl 8))
}

function Get-U32([byte[]]$Buffer, [int]$Offset) {
    return [uint32]([uint32]$Buffer[$Offset] -bor
        ([uint32]$Buffer[$Offset + 1] -shl 8) -bor
        ([uint32]$Buffer[$Offset + 2] -shl 16) -bor
        ([uint32]$Buffer[$Offset + 3] -shl 24))
}

function New-BootFrame([byte]$Command, [uint16]$Sequence,
    [byte[]]$Payload) {
    [byte[]]$header = New-Object byte[] 6
    $header[0] = $ProtocolVersion
    $header[1] = $Command
    Set-U16 $header 2 $Sequence
    Set-U16 $header 4 ([uint16]$Payload.Length)
    [byte[]]$crcInput = [byte[]]($header + $Payload)
    [uint32]$crc = Get-Crc32 $crcInput
    [byte[]]$frame = New-Object byte[] (2 + $header.Length +
        $Payload.Length + 4)
    $frame[0] = $Magic0
    $frame[1] = $Magic1
    [Array]::Copy($header, 0, $frame, 2, $header.Length)
    [Array]::Copy($Payload, 0, $frame, 8, $Payload.Length)
    Set-U32 $frame (8 + $Payload.Length) $crc
    return $frame
}

function Read-SerialByte([System.IO.Ports.SerialPort]$Serial,
    [System.Diagnostics.Stopwatch]$Timer, [long]$DeadlineMs) {
    while ($Timer.ElapsedMilliseconds -lt $DeadlineMs) {
        try {
            return $Serial.ReadByte()
        }
        catch [System.TimeoutException] {
        }
    }
    return -1
}

function Read-Exact([System.IO.Ports.SerialPort]$Serial,
    [System.Diagnostics.Stopwatch]$Timer, [long]$DeadlineMs,
    [int]$Length) {
    [byte[]]$result = New-Object byte[] $Length
    for ($index = 0; $index -lt $Length; $index++) {
        $value = Read-SerialByte $Serial $Timer $DeadlineMs
        if ($value -lt 0) {
            return $null
        }
        $result[$index] = [byte]$value
    }
    return $result
}

function Read-BootFrame([System.IO.Ports.SerialPort]$Serial,
    [int]$TimeoutMs) {
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    [long]$deadline = $TimeoutMs
    $sawFirstMagic = $false
    $syncNoiseCount = 0

    while ($timer.ElapsedMilliseconds -lt $deadline) {
        $value = Read-SerialByte $Serial $timer $deadline
        if ($value -lt 0) {
            return $null
        }
        if (-not $sawFirstMagic) {
            $sawFirstMagic = ([byte]$value -eq $Magic0)
            if ($sawFirstMagic) {
                $syncNoiseCount = 0
            }
            continue
        }
        if ([byte]$value -eq $Magic1) {
            [byte[]]$header = Read-Exact $Serial $timer $deadline 6
            if ($null -eq $header) {
                return $null
            }
            [int]$length = Get-U16 $header 4
            if ($length -gt 64) {
                $sawFirstMagic = $false
                $syncNoiseCount = 0
                continue
            }
            [byte[]]$payload = Read-Exact $Serial $timer $deadline $length
            [byte[]]$crcBytes = Read-Exact $Serial $timer $deadline 4
            if (($null -eq $payload) -or ($null -eq $crcBytes)) {
                return $null
            }
            [uint32]$receivedCrc = Get-U32 $crcBytes 0
            [uint32]$calculatedCrc = Get-Crc32 ([byte[]]($header + $payload))
            if ($receivedCrc -ne $calculatedCrc) {
                $sawFirstMagic = $false
                $syncNoiseCount = 0
                continue
            }
            return [pscustomobject]@{
                Version = $header[0]
                Command = $header[1]
                Sequence = Get-U16 $header 2
                Payload = $payload
            }
        }
        if ([byte]$value -eq $Magic0) {
            $syncNoiseCount = 0
        }
        else {
            $syncNoiseCount++
            if ($syncNoiseCount -gt 4) {
                $sawFirstMagic = $false
                $syncNoiseCount = 0
            }
        }
    }
    return $null
}

function Invoke-BootCommand([System.IO.Ports.SerialPort]$Serial,
    [byte]$Command, [uint16]$Sequence, [byte[]]$Payload,
    [int]$TimeoutMs, [int]$Attempts = 3) {
    [byte[]]$frame = New-BootFrame $Command $Sequence $Payload
    for ($attempt = 1; $attempt -le $Attempts; $attempt++) {
        $Serial.BaseStream.Write($frame, 0, $frame.Length)
        $Serial.BaseStream.Flush()
        $response = Read-BootFrame $Serial $TimeoutMs
        if (($null -ne $response) -and
            ($response.Version -eq $ProtocolVersion) -and
            ($response.Command -eq ($Command -bor 0x80)) -and
            ($response.Sequence -eq $Sequence)) {
            if ($response.Payload.Length -lt 1) {
                throw "Bootloader returned an empty response."
            }
            if ($response.Payload[0] -ne 0) {
                throw "Bootloader status $($response.Payload[0]) for command $Command."
            }
            return $response.Payload
        }
    }
    throw "Bootloader command $Command timed out after $Attempts attempts."
}

if ([string]::IsNullOrWhiteSpace($Image)) {
    $repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
    $Image = Join-Path $repoRoot "build-gcc/CAR_CONTROL/car_control.bin"
}
$imagePath = (Resolve-Path $Image).Path
[byte[]]$imageBytes = [IO.File]::ReadAllBytes($imagePath)
[uint32]$imageCrc = Get-Crc32 $imageBytes

Write-Host ("Image: {0}" -f $imagePath)
Write-Host ("Size : {0} bytes" -f $imageBytes.Length)
Write-Host ("CRC32: 0x{0:X8}" -f $imageCrc)

if ($ValidateOnly) {
    [byte[]]$known = [Text.Encoding]::ASCII.GetBytes("123456789")
    if ((Get-Crc32 $known) -ne [uint32]3421780262) {
        throw "CRC32 self-test failed."
    }
    [byte[]]$testFrame = New-BootFrame $CommandData 300 $known
    if (($testFrame.Length -ne 21) -or
        ((Get-U16 $testFrame 4) -ne 300)) {
        throw "Frame encoding self-test failed."
    }
    Write-Host "Updater self-test passed."
    exit 0
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port, $BaudRate, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.ReadTimeout = 50
$serial.WriteTimeout = 2000
$serial.NewLine = "`r`n"
$serial.DtrEnable = $false
$serial.RtsEnable = $false

$totalTimer = [System.Diagnostics.Stopwatch]::StartNew()
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    if (-not $SkipEnter) {
        $serial.WriteLine("fw update")
        Start-Sleep -Milliseconds 250
    }

    [uint16]$sequence = 1
    [byte[]]$empty = [byte[]]::new(0)
    [byte[]]$ping = Invoke-BootCommand $serial $CommandPing $sequence $empty 700 5
    if ($ping.Length -lt 7) {
        throw "Bootloader PING response is incomplete."
    }
    [uint32]$maximumSize = Get-U32 $ping 1
    if ($imageBytes.Length -gt $maximumSize) {
        throw "Image exceeds bootloader maximum size $maximumSize."
    }

    $sequence++
    [byte[]]$beginPayload = New-Object byte[] 8
    Set-U32 $beginPayload 0 ([uint32]$imageBytes.Length)
    Set-U32 $beginPayload 4 $imageCrc
    [void](Invoke-BootCommand $serial $CommandBegin $sequence $beginPayload 10000 3)

    for ($offset = 0; $offset -lt $imageBytes.Length;
        $offset += $ChunkSize) {
        $sequence++
        [int]$count = [Math]::Min($ChunkSize, $imageBytes.Length - $offset)
        [byte[]]$dataPayload = New-Object byte[] (4 + $count)
        Set-U32 $dataPayload 0 ([uint32]$offset)
        [Array]::Copy($imageBytes, $offset, $dataPayload, 4, $count)
        [byte[]]$dataResponse = Invoke-BootCommand $serial $CommandData `
            $sequence $dataPayload 1500 3
        if (($dataResponse.Length -lt 5) -or
            ((Get-U32 $dataResponse 1) -ne ($offset + $count))) {
            throw "Bootloader acknowledged an unexpected image offset."
        }
        if (($StopAfterBytes -gt 0) -and
            (($offset + $count) -ge $StopAfterBytes)) {
            throw "Intentional interruption after $($offset + $count) bytes."
        }
        $percent = [int](100 * ($offset + $count) / $imageBytes.Length)
        Write-Progress -Activity "JDY-31 wireless firmware update" `
            -Status "$($offset + $count) / $($imageBytes.Length) bytes" `
            -PercentComplete $percent
    }
    Write-Progress -Activity "JDY-31 wireless firmware update" -Completed

    $sequence++
    [void](Invoke-BootCommand $serial $CommandEnd $sequence $empty 5000 3)

    if (-not $NoRun) {
        $sequence++
        [void](Invoke-BootCommand $serial $CommandRun $sequence $empty 1000 2)
    }

    Write-Host ("Update completed in {0:N1} seconds." -f
        $totalTimer.Elapsed.TotalSeconds)
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
