$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'Stop-PICO-Feedback.ps1')
$statePath = Join-Path $PSScriptRoot 'pico-usb-sharing-state.json'
if (-not (Test-Path -LiteralPath $statePath)) {
    Write-Output 'No saved PICO sharing session.'
    exit 0
}
$state = Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
$adb = Join-Path $PSScriptRoot 'tools\platform-tools\adb.exe'
$oldAdb = $env:ADB
try {
    $env:ADB = $adb
    $deviceState = & $adb -s $state.DeviceSerial get-state 2>$null
    if ($LASTEXITCODE -eq 0 -and "$deviceState".Trim() -eq 'device') {
        & $state.RelayPath stop $state.DeviceSerial
        if ($LASTEXITCODE -ne 0) { throw 'Could not stop the PICO VPN client.' }
        & $adb -s $state.DeviceSerial reverse --remove localabstract:gnirehtet
    }
    $relayProcess = Get-Process -Id $state.RelayPid -ErrorAction SilentlyContinue
    if ($relayProcess -and $relayProcess.Path -eq $state.RelayPath -and
        $relayProcess.StartTime.ToString('o') -eq $state.RelayStartedAt) {
        Stop-Process -Id $relayProcess.Id
    }
    Write-Output 'PICO USB sharing stopped. Windows network priorities were not changed.'
} finally {
    $env:ADB = $oldAdb
}
