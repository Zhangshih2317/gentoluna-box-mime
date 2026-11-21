param([string]$DeviceSerial)
$ErrorActionPreference = 'Stop'
$DeviceSerial = & (Join-Path $PSScriptRoot 'Get-UsbPico.ps1') -DeviceSerial $DeviceSerial
$worker = Join-Path $PSScriptRoot 'tools\pico-feedback\relay.cjs'
$statePath = Join-Path $PSScriptRoot 'pico-feedback-state.json'
if (Test-Path -LiteralPath $statePath) {
    $state = Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
    $existing = Get-CimInstance Win32_Process -Filter "ProcessId=$($state.pid)"
    if ($existing -and $existing.ExecutablePath -eq $state.nodePath -and
        $existing.CommandLine.Contains($worker)) {
        if ($state.serial -ne $DeviceSerial) { throw 'Feedback relay belongs to another PICO.' }
        Write-Output "PICO feedback relay is already running (PID $($state.pid))."
        return
    }
}
$node = (Get-Command node.exe -ErrorAction Stop).Source
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$stdout = Join-Path $PSScriptRoot "pico-feedback-$stamp.log"
$stderr = Join-Path $PSScriptRoot "pico-feedback-$stamp-error.log"
$relayProcess = Start-Process -FilePath $node -ArgumentList @("`"$worker`"", $DeviceSerial) `
    -WorkingDirectory $PSScriptRoot -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput $stdout -RedirectStandardError $stderr
for ($attempt = 0; $attempt -lt 40; $attempt++) {
    if ($relayProcess.HasExited) {
        $detail = Get-Content -LiteralPath $stderr -Raw -ErrorAction SilentlyContinue
        throw "PICO feedback relay exited. $detail"
    }
    if (Test-Path -LiteralPath $statePath) {
        $state = Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
        if ($state.pid -eq $relayProcess.Id -and $state.phase -eq 'connected') {
            Write-Output "PICO feedback relay started (PID $($relayProcess.Id))."
            return
        }
    }
    Start-Sleep -Milliseconds 250
}
throw "Feedback startup is still pending. Check $stderr and $statePath."
