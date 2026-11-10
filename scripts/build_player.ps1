
param(
    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 255)]
    [int]$Player,

    [ValidateRange(2, 254)]
    [int]$Total = 4,

    [ValidateSet('baseline', 'ndt', 'ndt_focus')]
    [string]$Mode = 'ndt',

    [string]$Board = 'xiao_ble/nrf52840',

    [switch]$UsbConsole
)

# Run this script in an nRF Connect SDK terminal with 'west' available.
$ErrorActionPreference = 'Stop'

if (($Total % 2) -ne 0 -or $Player -gt $Total) {
    throw 'Total must be even, and Player must be between 1 and Total.'
}

$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildDir = "build/player${Player}_${Mode}"

# Periodic advertising is disabled only for the focused NDT experiment.
$enablePeriodic = if ($Mode -eq 'ndt_focus') { 0 } else { 1 }

# Devicetree overlays are passed as ONE semicolon-separated CMake value.
$overlays = @('boards/xiao_ble_nrf52840.overlay')

if ($UsbConsole) {
    $overlays += 'boards/usb_cdc.overlay'
}

# Optional configuration files are passed together.
$extraConfigs = @()

if ($Mode -ne 'baseline') {
    $extraConfigs += 'dm.conf'
}

if ($UsbConsole) {
    $extraConfigs += 'usb.conf'
}

$argsList = @(
    'build',
    '--no-sysbuild',
    '-b', $Board,
    '-p', 'always',
    '-d', $buildDir,
    '.', '--',
    "-DMY_WRISTBAND_ID=$Player",
    "-DNUM_WRISTBANDS=$Total",
    "-DENABLE_PERIODIC=$enablePeriodic",
    "-DDTC_OVERLAY_FILE=$($overlays -join ';')"
)

if ($extraConfigs.Count -gt 0) {
    $argsList += "-DEXTRA_CONF_FILE=$($extraConfigs -join ';')"
}

Write-Host "Building Player$Player of $Total | Mode=$Mode | ENABLE_PERIODIC=$enablePeriodic | Board=$Board"
Write-Host "Overlays: $($overlays -join ';')"
Write-Host "Extra config: $($extraConfigs -join ';')"

Push-Location $root

try {
    & west @argsList

    if ($LASTEXITCODE -ne 0) {
        throw "Build failed (west exit code: $LASTEXITCODE). Check the FIRST compiler or Kconfig error above."
    }

    Write-Host "Build successful: $buildDir"
    Write-Host "Firmware output should be under: $buildDir/zephyr/"
    Write-Host 'Check the firmware and board bootloader procedure before flashing.'
}
finally {
    Pop-Location
}
