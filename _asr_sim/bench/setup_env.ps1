# 重建 .venv（幂等）。Smart App Control 会拦截新下载的 numpy/scipy 编译扩展，
# 所以这里用"DSH 自带 Python + 从 _praenv 复制已验证可运行的副本"的方式：
#   * numpy/scipy/pyroomacoustics/hid 与 _praenv 中字节相同，SAC 已放行；
#   * 纯 Python 包（sounddevice / pyserial）从 PyPI 正常安装。
#
# 用法：  powershell -ExecutionPolicy Bypass -File _asr_sim\bench\setup_env.ps1
$ErrorActionPreference = 'Stop'
Set-Location (Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent)

$root      = (Get-Location).Path
$venv      = Join-Path $root '.venv'
$donor     = Join-Path $root '_praenv\Lib\site-packages'
$dsnPython = 'C:\Users\baica\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe'

Write-Host "=== 1) 创建 .venv（复用系统 site-packages） ==="
& uv venv $venv --python $dsnPython --clear --system-site-packages

Write-Host "=== 2) 安装纯 Python 依赖 ==="
& uv pip install --python (Join-Path $venv 'Scripts\python.exe') sounddevice pyserial

Write-Host "=== 3) 从 _praenv 复制 SAC 放行的编译包 ==="
$dst = Join-Path $venv 'Lib\site-packages'
foreach ($pat in @('numpy', 'numpy.libs', 'numpy-*', 'scipy', 'scipy.libs', 'scipy-*',
                   'pyroomacoustics', 'pyroomacoustics-*', 'hid.cp312-win_amd64.pyd',
                   'hidapi-*.dist-info')) {
    Get-ChildItem -LiteralPath $donor -Filter $pat -ErrorAction SilentlyContinue | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $dst -Recurse -Force
    }
}

Write-Host "=== 4) 校验 ==="
& (Join-Path $venv 'Scripts\python.exe') -X utf8 -c @"
import sys, numpy, scipy, sounddevice, serial, pyroomacoustics, hid
print('python      ', sys.version.split()[0])
print('numpy       ', numpy.__version__)
print('scipy       ', scipy.__version__)
print('sounddevice ', sounddevice.__version__)
print('pyserial    ', serial.__version__)
print('pyroomacoust', pyroomacoustics.__version__)
print('hid devices ', len(hid.enumerate()))
"@
Write-Host "=== 完成 ==="
