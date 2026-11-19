# 手动测试入口：统一用 .venv 的解释器 + UTF-8，避免中文崩在采集/渲染循环里。
#
# 硬件线（需要音箱 + 麦克风 + 链路）：
#   .\run.ps1 check                      环境/设备/通道声学响应自检
#   .\run.ps1 out34 [--audible]          4 路输出测量 / 人耳确认
#   .\run.ps1 spatial --modes 0,1,4      空间分离噪声下的模式对比
#   .\run.ps1 whisper                    小声/耳语 + 噪声矩阵
#
# 仿真线（纯 pyroomacoustics，不需要硬件）：
#   .\run.ps1 sim --clips 24 --snr 0 --out _asr_sim/sim_snr0
#   .\run.ps1 score --dir _asr_sim/sim_snr0
#   .\run.ps1 sweep --matrix geometry --clips 24
param(
    [Parameter(Position = 0)][string]$Command = 'check',
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$Rest
)

$ErrorActionPreference = 'Continue'
$bench = $PSScriptRoot
$root = Split-Path (Split-Path $bench -Parent) -Parent
$py = Join-Path $root '.venv\Scripts\python.exe'

if (-not (Test-Path $py)) {
    Write-Host "找不到 $py，请先运行 setup_env.ps1" -ForegroundColor Red
    exit 2
}

$env:PYTHONIOENCODING = 'utf-8'
$env:PYTHONUTF8 = '1'
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}

$map = @{
    'check'     = 'step1_preflight.py'
    'preflight' = 'step1_preflight.py'
    'out34'     = 'step2_out34.py'
    'spatial'   = 'step3_spatial_sep.py'
    'whisper'   = 'step4_whisper_noise.py'
    'sim'       = 'sim_room.py'
    'score'     = 'score_dir.py'
    'sweep'     = 'sweep.py'
}

$script = $map[$Command]
if (-not $script) {
    Write-Host "未知命令: $Command" -ForegroundColor Red
    Write-Host "硬件: check | out34 | spatial | whisper"
    Write-Host "仿真: sim | score | sweep"
    exit 2
}

$path = Join-Path $bench $script
Write-Host ">>> $py -X utf8 $path $($Rest -join ' ')" -ForegroundColor Cyan
& $py -X utf8 $path @Rest
exit $LASTEXITCODE