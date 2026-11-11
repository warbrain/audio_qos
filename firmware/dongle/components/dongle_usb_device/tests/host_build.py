"""Compile and run the real audio recovery C implementation on the host.

Requires pytest and GCC, Clang, or MSVC with C11 support. Set CC to override
compiler discovery. Windows MSVC/Clang needs MSVC Build Tools and a Windows SDK;
VCToolsInstallDir and WindowsSdkDir can override their discovered locations.
"""

from __future__ import annotations

import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[4]
RECOVERY = ROOT / "components" / "dual_end_audio_recovery"
SUCCESS_MESSAGE = "audio recovery host tests passed"


def _version_key(path: Path) -> tuple[int, ...]:
    return tuple(int(part) for part in re.findall(r"\d+", path.name))


def _compiler_command() -> list[str]:
    override = os.environ.get("CC", "").strip()
    if override:
        if Path(override).is_file():
            return [override]
        parts = shlex.split(override, posix=os.name != "nt")
        return [part[1:-1] if part[:1] == part[-1:] == '"' else part for part in parts]

    for name in ("clang", "gcc", "cc"):
        compiler = shutil.which(name)
        if compiler:
            return [compiler]

    if os.name == "nt":
        # ESP Clang can parse x86 C but may omit its x86 code-generation backend.
        # Use the installed native MSVC compiler instead of assuming otherwise.
        return [str(_msvc_tools() / "bin/Hostx64/x64/cl.exe")]

    pytest.fail(
        "No host C compiler found. Install GCC or Clang and add it to PATH, or "
        "set CC to its executable. Windows also supports MSVC Build Tools with "
        "a Windows SDK. The C behavior tests cannot be skipped.",
        pytrace=False,
    )


def _msvc_tools() -> Path:
    override = os.environ.get("VCToolsInstallDir")
    if override:
        candidates = [Path(override)]
    else:
        candidates = []
        for variable, default in (
            ("ProgramFiles(x86)", r"C:\Program Files (x86)"),
            ("ProgramFiles", r"C:\Program Files"),
        ):
            root = Path(os.environ.get(variable, default)) / "Microsoft Visual Studio"
            candidates.extend(root.glob("*/*/VC/Tools/MSVC/*"))
        candidates.sort(key=_version_key, reverse=True)
    for candidate in candidates:
        if (candidate / "bin/Hostx64/x64/link.exe").is_file() and (
            candidate / "lib/x64/libcmt.lib"
        ).is_file():
            return candidate
    pytest.fail(
        "The Windows host test needs the MSVC x64 C++ Build Tools. Install that workload "
        "or set VCToolsInstallDir to its VC/Tools/MSVC/<version> directory.",
        pytrace=False,
    )


def _windows_sdk() -> tuple[Path, str]:
    roots = []
    override = os.environ.get("WindowsSdkDir")
    if override:
        roots.append(Path(override))
    else:
        import winreg

        for key_name in (
            r"SOFTWARE\Microsoft\Windows Kits\Installed Roots",
            r"SOFTWARE\WOW6432Node\Microsoft\Windows Kits\Installed Roots",
        ):
            try:
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, key_name) as key:
                    roots.append(Path(winreg.QueryValueEx(key, "KitsRoot10")[0]))
            except OSError:
                pass
        roots.append(
            Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
            / "Windows Kits/10"
        )

    requested_version = os.environ.get("WindowsSDKVersion", "").strip("\\/")
    for root in roots:
        versions = (
            [root / "Include" / requested_version]
            if requested_version
            else sorted((root / "Include").glob("10.*"), key=_version_key, reverse=True)
        )
        for include in versions:
            if all(
                path.is_file()
                for path in (
                    include / "ucrt/assert.h",
                    root / "Lib" / include.name / "ucrt/x64/libucrt.lib",
                    root / "Lib" / include.name / "um/x64/kernel32.lib",
                )
            ):
                return root, include.name
    pytest.fail(
        "The Windows host test needs a Windows 10/11 SDK with x64 UCRT and UM libraries. "
        "Install it or set WindowsSdkDir and, optionally, WindowsSDKVersion.",
        pytrace=False,
    )


def _run(command: list[str], *, cwd: Path, env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            env=env,
            capture_output=True,
            text=True,
            errors="replace",
            timeout=120,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        pytest.fail(f"Host C command could not run: {command!r}\n{error}", pytrace=False)
    assert result.returncode == 0, (
        f"Host C command failed with exit {result.returncode}: {command!r}\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
    return result


def _build_host_executable(
    sources: list[Path],
    build_dir: Path,
    *,
    extra_includes: tuple[Path, ...] = (),
    extra_defines: tuple[str, ...] = (),
) -> tuple[Path, dict[str, str]]:
    build_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    # Keep compiler intermediates inside the test's allocated directory too.
    for variable in ("TMP", "TEMP", "TMPDIR"):
        env[variable] = str(build_dir)
    command = _compiler_command()
    is_msvc = any(Path(part).name.lower() in ("cl", "cl.exe") for part in command)
    needs_msvc_runtime = is_msvc
    if not is_msvc:
        version = _run(command + ["--version"], cwd=build_dir, env=env)
    if os.name == "nt" and not is_msvc and "clang" in (version.stdout + version.stderr).lower():
        target = _run(command + ["-dumpmachine"], cwd=build_dir, env=env).stdout.strip()
        # A MinGW Clang already has its own host headers and runtime libraries.
        needs_msvc_runtime = "mingw" not in target and "windows-gnu" not in target
    if needs_msvc_runtime:
        msvc = _msvc_tools()
        sdk, sdk_version = _windows_sdk()
        env.update(
            VCToolsInstallDir=str(msvc) + os.sep,
            WindowsSdkDir=str(sdk) + os.sep,
            WindowsSDKVersion=sdk_version + os.sep,
            UniversalCRTSdkDir=str(sdk) + os.sep,
            UCRTVersion=sdk_version,
        )
        include = sdk / "Include" / sdk_version
        libraries = sdk / "Lib" / sdk_version
        env["INCLUDE"] = os.pathsep.join(
            str(path) for path in (msvc / "include", include / "ucrt", include / "shared", include / "um")
        )
        env["LIB"] = os.pathsep.join(
            str(path) for path in (msvc / "lib/x64", libraries / "ucrt/x64", libraries / "um/x64")
        )
        env["PATH"] = str(msvc / "bin/Hostx64/x64") + os.pathsep + env.get("PATH", "")
        env["VSLANG"] = "1033"

    for source in sources:
        assert source.is_file(), f"Missing actual C test/implementation source: {source}"
    executable = build_dir / ("audio_recovery_host_test.exe" if os.name == "nt" else "audio_recovery_host_test")
    includes = list(extra_includes)
    if is_msvc:
        flags = ["/nologo", "/std:c11", "/utf-8", "/W4", "/WX"]
        flags += ["/I" + str(path) for path in includes]
        flags += ["/D" + define for define in extra_defines]
        outputs = ["/Fe:" + str(executable), "/Fo:" + str(build_dir) + os.sep]
    else:
        flags = ["-std=c11", "-Wall", "-Wextra", "-Werror"]
        if needs_msvc_runtime:
            flags.append("--target=x86_64-pc-windows-msvc")
        for path in includes:
            flags += ["-I", str(path)]
        flags += ["-D" + define for define in extra_defines]
        outputs = ["-o", str(executable)]
    _run(
        command
        + flags
        + [str(source) for source in sources]
        + outputs,
        cwd=build_dir,
        env=env,
    )
    assert executable.is_file(), f"The compiler did not produce {executable}"
    return executable, env


