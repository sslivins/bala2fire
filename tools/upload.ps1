#requires -Version 5.1
<#
.SYNOPSIS
    Flash a freshly-built bala2.bin to the robot over Wi-Fi (OTA).

.DESCRIPTION
    Posts build/bala2.bin to http://<Ip>/update. The ESP32 streams it into
    its inactive OTA slot, marks it for next boot, and reboots. New firmware
    will mark itself valid after ~10 s; if it crashes before then, the
    bootloader rolls back automatically.

.PARAMETER Ip
    IP (or mDNS name) of the robot. Default 10.25.2.55.

.PARAMETER Bin
    Path to the firmware image. Default build/bala2.bin in repo root.

.EXAMPLE
    .\tools\upload.ps1
    .\tools\upload.ps1 -Ip 10.25.2.55
#>
[CmdletBinding()]
param(
    [string]$Ip  = '10.25.2.55',
    [string]$Bin = (Join-Path $PSScriptRoot '..\build\bala2.bin')
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path $Bin)) {
    Write-Error "Firmware image not found: $Bin (run 'idf.py build' first)"
    exit 1
}
$size = (Get-Item $Bin).Length
Write-Host ("Uploading {0} ({1:N0} bytes) to http://{2}/update ..." -f $Bin, $size, $Ip)

# Pre-flight: probe /info so we fail fast with a clear message if the
# device is unreachable or in armed state.
try {
    $info = Invoke-RestMethod -Uri "http://$Ip/info" -TimeoutSec 5
    Write-Host ("Device info: version={0} slot={1} armed={2} uptime={3}ms" -f `
        $info.version, $info.slot, $info.armed, $info.uptime_ms)
    if ($info.armed) {
        Write-Error "Robot is ARMED. Disarm before OTA (Btn A or Btn C)."
        exit 2
    }
} catch {
    Write-Warning "Could not reach http://$Ip/info ($($_.Exception.Message)). Proceeding anyway..."
}

# Use curl.exe — Invoke-WebRequest with -InFile loads the whole body into
# memory and is slower; curl streams it in 16 KB chunks.
$sw = [Diagnostics.Stopwatch]::StartNew()
& curl.exe --silent --show-error --fail --max-time 120 `
    -X POST `
    --data-binary "@$Bin" `
    -H 'Content-Type: application/octet-stream' `
    "http://$Ip/update"
$exit = $LASTEXITCODE
$sw.Stop()

if ($exit -ne 0) {
    Write-Error "OTA failed with curl exit code $exit"
    exit $exit
}
Write-Host ("`nUpload + reboot triggered in {0:N1}s. Wait ~5 s, then re-check /info." -f $sw.Elapsed.TotalSeconds)
