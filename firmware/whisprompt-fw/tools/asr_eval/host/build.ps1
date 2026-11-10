# Build the host DSP tool with MSVC (or clang/gcc fallback).
# Usage: .\build.ps1 [-Firmware <firmware-root>]
# Default to the firmware tree containing this script.
[CmdletBinding()]
param(
    [string]$Firmware = (Join-Path $PSScriptRoot '..\..\..')
)

$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$Firmware = (Resolve-Path -LiteralPath $Firmware).ProviderPath
$codec = Join-Path $Firmware 'components\dual_end_audio_codec'
if (-not (Test-Path -LiteralPath $codec)) { throw "firmware codec component not found: $codec" }

$src = @(
    (Join-Path $here 'win_spatial_wav.c'),
    (Join-Path $codec 'dual_end_spatial.c'),
    (Join-Path $codec 'dual_end_wiener.c')
)
$buildRoot = Join-Path $Firmware 'build\asr-eval-host'
$out = Join-Path $buildRoot 'bin'
$objects = Join-Path $buildRoot 'obj'
New-Item -ItemType Directory -Force -Path $out, $objects | Out-Null

function Find-Msvc {
    $vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path -LiteralPath $vswhere)) { return $null }
    $root = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $root) { return $null }
    return (Join-Path $root 'VC\Auxiliary\Build\vcvars64.bat')
}

# Both compiler branches run from build, and compiler scratch/PDB/link files
# stay there even when this script is launched from a source directory.
$temporaryVariables = @('TMP', 'TEMP', 'TMPDIR')
$previousTemporaryValues = @{}
foreach ($variable in $temporaryVariables) {
    $previousTemporaryValues[$variable] = [Environment]::GetEnvironmentVariable($variable, 'Process')
}
Push-Location -LiteralPath $objects
try {
    foreach ($variable in $temporaryVariables) {
        [Environment]::SetEnvironmentVariable($variable, $objects, 'Process')
    }
    $vcvars = Find-Msvc
    if ($vcvars) {
        $argsLine = '/nologo /O2 /W3 /std:c11 /D_CRT_SECURE_NO_WARNINGS /I "' + (Join-Path $codec 'include') + '" ' +
            (($src | ForEach-Object { '"' + $_ + '"' }) -join ' ') +
            ' /Fe:"' + (Join-Path $out 'win_spatial_wav.exe') + '"' +
            ' /Fd:"' + (Join-Path $objects 'win_spatial_wav-compile.pdb') + '"' +
            ' /link /PDB:"' + (Join-Path $out 'win_spatial_wav.pdb') + '"' +
            ' /ILK:"' + (Join-Path $objects 'win_spatial_wav.ilk') + '"'
        $cmd = 'call "' + $vcvars + '" >nul 2>&1 && cd /d "' + $objects + '" && cl ' + $argsLine
        & $env:ComSpec /d /c $cmd
        if ($LASTEXITCODE -ne 0) { throw "MSVC build failed (exit $LASTEXITCODE)" }
        Write-Host "built: $(Join-Path $out 'win_spatial_wav.exe')"
        return
    }

    foreach ($cc in 'clang', 'gcc', 'cc') {
        $exe = Get-Command $cc -CommandType Application -ErrorAction SilentlyContinue
        if (-not $exe) { continue }
        & $exe.Source -O2 -std=c11 -I (Join-Path $codec 'include') @src -lm `
            -o (Join-Path $out 'win_spatial_wav.exe')
        if ($LASTEXITCODE -ne 0) { throw "$cc build failed" }
        Write-Host "built with $cc`: $(Join-Path $out 'win_spatial_wav.exe')"
        return
    }

    throw 'No C compiler found. Install MSVC Build Tools, or put clang/gcc on PATH.'
}
finally {
    foreach ($variable in $temporaryVariables) {
        [Environment]::SetEnvironmentVariable($variable, $previousTemporaryValues[$variable], 'Process')
    }
    Pop-Location
}
