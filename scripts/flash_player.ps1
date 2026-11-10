param(
    [Parameter(Mandatory=$true)][ValidateRange(1,255)][int]$Player,
    [ValidateSet('ndt','ndt_focus','baseline')][string]$Mode = 'ndt'
)
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$uf2 = Join-Path $root "build/player${Player}_${Mode}/zephyr/zephyr.uf2"
if (!(Test-Path $uf2)) { throw "Missing $uf2. Build first." }
Write-Host "Enter UF2 mode on the XIAO for Player$Player (double-tap RESET)."
Write-Host "Copy this file onto the XIAO BLE USB drive: $uf2"
Write-Host 'This script intentionally does not auto-select a drive or overwrite another device.'
