$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$elfPath = Join-Path $repoRoot "build-gcc/DAP_LINK_TEST/dap_link_test.elf"
$cmdFile = Join-Path $PSScriptRoot "flash_mspm0.jlink"

if (-not (Test-Path $elfPath)) {
    throw "Target file not found: $elfPath"
}

$candidates = @(
    "C:\Program Files (x86)\SEGGER\JLink_V634f\JLink.exe",
    "C:\Program Files\SEGGER\JLink\JLink.exe",
    "C:\Program Files (x86)\SEGGER\JLink\JLink.exe"
)

$jlinkExe = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1

if (-not $jlinkExe) {
    throw "JLink.exe not found. Please install SEGGER J-Link 7.88i or newer."
}

Push-Location $repoRoot
try {
    & $jlinkExe `
        -device MSPM0G3507 `
        -if SWD `
        -speed 4000 `
        -autoconnect 1 `
        -exitonerror 1 `
        -commandfile $cmdFile
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
