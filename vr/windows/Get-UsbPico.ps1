param([string]$DeviceSerial)
$ErrorActionPreference = 'Stop'
$adb = Join-Path $PSScriptRoot 'tools\platform-tools\adb.exe'
if (-not $DeviceSerial) {
    $detected = & $adb -d get-serialno
    if ($LASTEXITCODE -ne 0 -or -not $detected) { throw 'Connect one authorized USB headset, or specify -DeviceSerial.' }
    $DeviceSerial = "$detected".Trim()
}
if ($DeviceSerial -notmatch '^[A-Za-z0-9_-]+$' -or $DeviceSerial -match '^adb-.*-.*$') {
    throw 'Use the USB device serial from adb devices -l.'
}
$state = & $adb -s $DeviceSerial get-state
if ($LASTEXITCODE -ne 0 -or "$state".Trim() -ne 'device') { throw 'Authorize USB debugging in the headset.' }
$DeviceSerial
