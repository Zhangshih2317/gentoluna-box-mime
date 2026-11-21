param([string]$PlatformZip, [string]$GnirehtetZip)
$ErrorActionPreference = 'Stop'
$tools = Join-Path $PSScriptRoot 'tools'
$cache = Join-Path $tools 'downloads'
New-Item -ItemType Directory -Force $cache | Out-Null
function Expand-Verified($inputZip, $name, $url, $sha, $destination) {
    if (-not $inputZip) {
        $inputZip = Join-Path $cache $name
        if (-not (Test-Path -LiteralPath $inputZip)) {
            Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $inputZip
        }
    }
    if ((Get-FileHash -Algorithm SHA256 -LiteralPath $inputZip).Hash -ne $sha) {
        throw "SHA256 mismatch for $inputZip. Supply the verified release archive."
    }
    Expand-Archive -Force -LiteralPath $inputZip -DestinationPath $destination
}
Expand-Verified $PlatformZip 'platform-tools.zip' 'https://dl.google.com/android/repository/platform-tools_r37.0.1-win.zip' '45f4d63113e895ebde0c90f194099a4676b6ac653bd28d54314a9e022bbc1a99' $tools
Expand-Verified $GnirehtetZip 'gnirehtet.zip' 'https://github.com/Genymobile/gnirehtet/releases/download/v2.5.1/gnirehtet-rust-win64-v2.5.1.zip' '7f5b1063e7895182aa60def1437e50363c3758144088dcd079037bb7c3c46a1c' (Join-Path $tools 'gnirehtet-v2.5.1')
& (Join-Path $tools 'platform-tools\adb.exe') version
if ($LASTEXITCODE -ne 0) { throw 'adb installation failed.' }
