param(
    [string]$SpeechDir = '',
    [string]$Out = 'sim-box',
    [double]$Snr = 10,
    [double]$T60 = 0.3,
    [ValidateSet('pink', 'white')][string]$Noise = 'pink',
    [switch]$Coherent,
    [int]$Limit = 0,
    [switch]$Report,
    [string]$Python = ''
)

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

# Preferred interpreter: the uv-managed environment inside this project.
$venv = Join-Path $here '.venv\Scripts\python.exe'
if ($Python) { $candidates = @($Python) }
elseif (Test-Path $venv) { $candidates = @($venv) }
else { $candidates = @('python') }

$python = $null
foreach ($candidate in $candidates) {
    & $candidate -c "import numpy, pyroomacoustics, scipy, soundfile" 2>&1 | Out-Null
    if ($LASTEXITCODE -eq 0) { $python = $candidate; break }
}
if (-not $python) {
    throw "no usable interpreter; create it with: uv venv .venv --python 3.12; uv pip install -r requirements.txt"
}
# Only fall back to the bundled pydeps when the venv is not in use.
if ($python -ne $venv) {
    $deps = Join-Path $here 'pydeps'
    if (Test-Path $deps) { $env:PYTHONPATH = $deps }
}
if (-not $Report -and -not $SpeechDir) { throw 'SpeechDir is required unless -Report is used' }

$simArgs = @((Join-Path $here 'box_sim.py'), '--out', $Out, '--snr', $Snr, '--t60', $T60, '--noise', $Noise)
if ($SpeechDir) { $simArgs += @('--speech-dir', $SpeechDir) }
if ($Coherent) { $simArgs += '--coherent' }
if ($Limit -gt 0) { $simArgs += @('--limit', $Limit) }
if ($Report) { $simArgs += '--report' }

Write-Host "python: $python"
& $python @simArgs
if ($LASTEXITCODE -ne 0) { throw "box_sim failed with exit $LASTEXITCODE" }
