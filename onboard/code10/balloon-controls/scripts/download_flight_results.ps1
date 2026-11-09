param(
    [string]$JetsonAddress = 'uav@192.168.55.1',
    [string]$OutputDirectory = (Join-Path $HOME 'uav-flight-results'),
    [switch]$TelemetryOnly,
    [switch]$SkipReplay
)
$ErrorActionPreference = 'Stop'

$collector = Join-Path $PSScriptRoot 'collect_flight_results.py'
$runtime = Join-Path (Split-Path $PSScriptRoot) '../recognition/runtime'
$finisher = Join-Path $runtime 'finish_flight_download.py'
$download = Join-Path $OutputDirectory ((Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [Guid]::NewGuid().ToString('N').Substring(0, 6))
$remoteCommand = if ($TelemetryOnly) { 'python3 - --telemetry-only' } else { 'python3 -' }

$receiptOutput = Get-Content -Raw -LiteralPath $collector |
    & ssh -o StrictHostKeyChecking=yes -o ConnectTimeout=10 -o ServerAliveInterval=15 -o ServerAliveCountMax=4 -- $JetsonAddress $remoteCommand
if ($LASTEXITCODE -ne 0) { throw 'Code10 flight evidence collection failed.' }
$receipt = ($receiptOutput -join "`n") | ConvertFrom-Json
if ($receipt.archive -cnotmatch '^/home/uav/flight_results_[a-f0-9]{8}_[a-f0-9]{10}\.tar\.gz$' -or
    $receipt.sha256 -cnotmatch '^[a-f0-9]{64}$') {
    throw 'Invalid Code10 flight evidence receipt.'
}

New-Item -ItemType Directory -Path $download -Force | Out-Null
& scp -o StrictHostKeyChecking=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=4 -- "${JetsonAddress}:$($receipt.archive)" $download
if ($LASTEXITCODE -ne 0) { throw 'Download failed. The archive remains on Code10 for retry.' }
$archive = Join-Path $download ([IO.Path]::GetFileName($receipt.archive))
$hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash.ToLowerInvariant()
if ($hash -cne $receipt.sha256) { throw 'Download SHA256 verification failed.' }
$receipt | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $download 'download_receipt.json') -Encoding UTF8
foreach ($warning in $receipt.warnings) { Write-Warning $warning }

$processing = @($archive)
if ($SkipReplay -or $TelemetryOnly) { $processing += '--skip-replay' }
& python -B $finisher @processing
if ($LASTEXITCODE -ne 0) { throw "Downloaded archive retained at $archive. Processing did not complete." }
Write-Output $archive
