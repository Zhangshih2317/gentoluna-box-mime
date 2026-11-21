param([string]$DeviceSerial, [switch]$LunaEthernet)
$ErrorActionPreference = 'Stop'
$adb = Join-Path $PSScriptRoot 'tools\platform-tools\adb.exe'
$relay = Join-Path $PSScriptRoot 'tools\gnirehtet-v2.5.1\gnirehtet-rust-win64\gnirehtet.exe'
$statePath = Join-Path $PSScriptRoot 'pico-usb-sharing-state.json'
$DeviceSerial = & (Join-Path $PSScriptRoot 'Get-UsbPico.ps1') -DeviceSerial $DeviceSerial
$oldAdb = $env:ADB
try {
    $env:ADB = $adb
    $deviceState = & $adb -s $DeviceSerial get-state
    if ($LASTEXITCODE -ne 0 -or "$deviceState".Trim() -ne 'device') {
        throw 'Connect and authorize the PICO headset over USB first.'
    }
    $clientArgs = @('-p', '31416', '-r', '6.6.7.100/32')
    $source = Find-NetRoute -RemoteIPAddress '6.6.7.100' |
        Where-Object { $_.IPAddress -like '6.6.7.*' }
    if (-not $source -or (Get-NetAdapter -InterfaceIndex $source.InterfaceIndex).MediaType -ne '802.3') {
        throw 'The route to Luna must use its wired Ethernet subnet.'
    }
    $listener = @(Get-NetTCPConnection -LocalPort 31416 -State Listen -ErrorAction SilentlyContinue)
    if ($listener.Count -gt 0) {
        $relayProcess = Get-Process -Id $listener[0].OwningProcess
        if ($relayProcess.Path -ne $relay) { throw 'Port 31416 is occupied by another program.' }
        $stdout = $null
        $stderr = $null
        if (Test-Path -LiteralPath $statePath) {
            $previous = Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
            if ($previous.RelayPid -eq $relayProcess.Id -and
                $previous.RelayStartedAt -eq $relayProcess.StartTime.ToString('o')) {
                $stdout = $previous.Stdout
                $stderr = $previous.Stderr
            }
        }
    } else {
        $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
        $stdout = Join-Path $PSScriptRoot "pico-usb-sharing-$stamp.log"
        $stderr = Join-Path $PSScriptRoot "pico-usb-sharing-$stamp-error.log"
        $relayProcess = Start-Process -FilePath $relay -ArgumentList @('relay','-p','31416') `
            -WorkingDirectory (Split-Path $relay) -WindowStyle Hidden -PassThru `
            -RedirectStandardOutput $stdout -RedirectStandardError $stderr
        for ($attempt = 0; $attempt -lt 20; $attempt++) {
            $listener = @(Get-NetTCPConnection -LocalPort 31416 -State Listen -ErrorAction SilentlyContinue)
            if ($listener.Count -gt 0) { break }
            if ($relayProcess.HasExited) { throw 'The USB relay exited during startup; see its error log.' }
            Start-Sleep -Milliseconds 150
        }
        if ($listener.Count -eq 0) { throw 'The USB relay did not start listening.' }
    }
    @{
        DeviceSerial = $DeviceSerial
        RelayPid = $relayProcess.Id
        RelayPath = $relay
        RelayStartedAt = $relayProcess.StartTime.ToString('o')
        UpdatedAt = (Get-Date).ToString('o')
        Port = 31416
        Stdout = $stdout
        Stderr = $stderr
    } | ConvertTo-Json | Set-Content -LiteralPath $statePath -Encoding UTF8
    & $relay tunnel $DeviceSerial -p 31416
    if ($LASTEXITCODE -ne 0) { throw 'Could not establish the ADB USB tunnel.' }
    & $relay start $DeviceSerial @clientArgs
    if ($LASTEXITCODE -ne 0) { throw 'Could not request the PICO VPN connection.' }
    $tunnelAddress = ''
    for ($attempt = 0; $attempt -lt 10; $attempt++) {
        $tunnelAddress = & $adb -s $DeviceSerial shell ip -4 addr show tun0 2>$null
        if ("$tunnelAddress" -match 'inet 10\.0\.0\.2/') { break }
        Start-Sleep -Milliseconds 300
    }
    if ("$tunnelAddress" -match 'inet 10\.0\.0\.2/') {
        Write-Output 'PICO VPN interface is active at 10.0.0.2. USB sharing relay is running.'
        & (Join-Path $PSScriptRoot 'Start-PICO-Feedback.ps1') -DeviceSerial $DeviceSerial
    } else {
        Write-Output 'USB sharing requested; the PICO VPN interface is not active yet. Allow the VPN prompt inside PICO.'
    }
    Write-Output 'Apex robot address remains 6.6.7.100. Robot motion was not enabled.'
} finally {
    $env:ADB = $oldAdb
}
