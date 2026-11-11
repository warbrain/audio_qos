param(
    [Parameter(Mandatory = $true)][string]$Manifest,
    [string]$Out = 'results/cer',
    [string]$Modes = '0,1,4,5',
    [string]$Downmix = 'mean',
    [int]$Bootstrap = 500,
    [string]$Project = '',
    [string]$EvalTool = '',
    [string]$Python = ''
)

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = Split-Path -Parent (Split-Path -Parent $here)
if (-not $Project) { $Project = Join-Path $repo 'hmi\native-console' }
if (-not $EvalTool) { $EvalTool = Join-Path $repo 'firmware\tools\asr_eval\asr_cer_eval.py' }

$venv = Join-Path $here '.venv\Scripts\python.exe'
if ($Python) { $python = $Python }
elseif (Test-Path $venv) { $python = $venv }
else { $python = 'python' }

& $python -c "import numpy" 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) { throw "no Python with numpy: $python" }
if (-not (Test-Path $EvalTool)) { throw "eval tool not found: $EvalTool" }

& $python $EvalTool --manifest $Manifest --project $Project --modes $Modes --downmix $Downmix --bootstrap $Bootstrap --out $Out
if ($LASTEXITCODE -ne 0) { throw "CER evaluation failed with exit $LASTEXITCODE" }

$summary = Join-Path $Out 'summary.json'
if (Test-Path $summary) {
    Write-Host "`nsummary: $summary"
    Get-Content $summary -Raw | ConvertFrom-Json | Select-Object -ExpandProperty conditions
}
