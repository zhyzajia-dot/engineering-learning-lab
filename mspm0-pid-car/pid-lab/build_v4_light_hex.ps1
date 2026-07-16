param(
    [string]$OutputHex = (Join-Path $PSScriptRoot 'Debug\pid_lab_mspm0_v4_turnguard_rebuilt.hex')
)

$ErrorActionPreference = 'Stop'
$Root = (Resolve-Path $PSScriptRoot).Path
$archive = Join-Path $Root 'releases\V4_LIGHT_TURNGUARD_PROJECT_20260715.zip'
$expectedArchiveSha256 = '33F8D90520A3CA89A686D871E32120E8A1BCFE5A19B1CA64F1F6C1DF83D12BAD'
$expectedSha256 = 'E8EA86F3A9CD9BA089644B90D8C7E6922221B08E38C6136FD7068A4648A36A93'
$extractRoot = Join-Path $Root 'tmp\v4_light_frozen_build'

if (-not [System.IO.Path]::IsPathRooted($OutputHex)) {
    $OutputHex = Join-Path $Root $OutputHex
}

$archiveSha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash
if ($archiveSha256 -ne $expectedArchiveSha256) {
    throw "Frozen V4 LIGHT project archive SHA-256 mismatch: $archiveSha256"
}

New-Item -ItemType Directory -Force -Path $extractRoot | Out-Null
Expand-Archive -LiteralPath $archive -DestinationPath $extractRoot -Force
$frozenBuild = Join-Path $extractRoot 'build_mspm0_hex.ps1'

& $frozenBuild `
    -OutputHex $OutputHex `
    -Defines 'LAB_ENABLE_DUAL_PROFILE=0'
if ($LASTEXITCODE -ne 0) {
    throw 'Frozen V4 LIGHT firmware build failed'
}

$actualSha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $OutputHex).Hash
if ($actualSha256 -ne $expectedSha256) {
    throw "V4 LIGHT SHA-256 mismatch: $actualSha256"
}

$info = Get-Item -LiteralPath $OutputHex
Write-Host "V4 LIGHT verified: $($info.FullName) ($($info.Length) bytes)"
Write-Host "SHA-256: $actualSha256"
