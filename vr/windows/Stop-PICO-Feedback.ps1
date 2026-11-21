$ErrorActionPreference = 'Stop'
$statePath = Join-Path $PSScriptRoot 'pico-feedback-state.json'
$adb = Join-Path $PSScriptRoot 'tools\platform-tools\adb.exe'
if (-not (Test-Path -LiteralPath $statePath)) { return }
$state = Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
$worker = Join-Path $PSScriptRoot 'tools\pico-feedback\relay.cjs'
$existing = Get-CimInstance Win32_Process -Filter "ProcessId=$($state.pid)"
if ($existing -and $existing.ExecutablePath -eq $state.nodePath -and
    $existing.CommandLine.Contains($worker)) {
    Stop-Process -Id $state.pid
}
$mappings = & $adb -s $state.serial forward --list
if ($LASTEXITCODE -eq 0 -and
    @($mappings | Where-Object { $_.Trim() -eq "$($state.serial) tcp:39002 tcp:39002" }).Count -gt 0) {
    & $adb -s $state.serial forward --remove tcp:39002
}
$state.phase = 'stopped'
$state.updatedAt = [DateTime]::UtcNow.ToString('o')
$state | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $statePath -Encoding UTF8
Write-Output 'PICO feedback relay stopped.'
