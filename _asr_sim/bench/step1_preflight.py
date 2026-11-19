"""第 1 步：环境与路由自检（不需要人工听，可反复运行）。

检查顺序是按"历史上最容易被忽略、又最贵"的问题排的：

  1. Python 环境与编码        —— 打印不了中文会让采集循环静默崩在第 1 条
  2. 声卡端点                 —— 同名设备可能有 4 个实例，必须挑对
  3. UAC 是否在出数据         —— 链路假死时是数字静音，不是噪声
  4. **Out2/Out3/Out4 声学响应** —— 逐个通道放音实测，而不是只看枚举
  5. ASR 引擎                 —— native.dll 可能被 SAC 拦掉部分副本
  6. 历史采集完整性           —— 找出全零目录，避免用废数据算结论

用法：
  .venv\\Scripts\\python.exe _asr_sim\\bench\\step1_preflight.py
  .venv\\Scripts\\python.exe _asr_sim\\bench\\step1_preflight.py --quick   # 跳过声学探测
"""
from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
import common as C
import devices


def section(title: str) -> None:
    print(f"\n=== {title} ===")


def check_env() -> list[str]:
    section("1. Python 环境与编码")
    print("  解释器:", sys.executable)
    print("  Python :", sys.version.split()[0])
    print("  stdout :", getattr(sys.stdout, "encoding", "?"))
    problems = []
    if (getattr(sys.stdout, "encoding", "") or "").lower() not in ("utf-8", "utf8"):
        problems.append("标准输出不是 UTF-8：中文日志会在采集循环里抛异常")
    for mod in ("numpy", "sounddevice"):
        try:
            m = __import__(mod)
            print(f"  {mod:12s}: {getattr(m, '__version__', '?')}")
        except Exception as exc:
            problems.append(f"缺少 {mod}: {exc}")
    return problems


def check_devices() -> list[str]:
    section("2. 声卡端点")
    problems = []
    outs, ins = devices.outputs(), devices.inputs()
    print("  US-4x4HR（4 通道输出）:")
    for i, d in outs:
        print(f"    [{i}] out={d['max_output_channels']} sr={int(d['default_samplerate'])}")
    print("  WhisPrompt（2 通道输入）:")
    for i, d in ins:
        print(f"    [{i}] in={d['max_input_channels']} sr={int(d['default_samplerate'])}")
    if not outs:
        problems.append("找不到 4 通道的 US-4x4HR 输出端点")
    if not ins:
        problems.append("找不到 2 通道的 WhisPrompt 输入端点")
    if not problems:
        out_dev, in_dev = devices.pick_pair()
        print(f"  -> 选定 输出={out_dev} 输入={in_dev}")
    return problems


def check_uac_alive() -> list[str]:
    section("3. WhisPrompt UAC 是否在出数据")
    out_dev, in_dev = devices.pick_pair()
    if in_dev is None:
        return ["无输入端点，跳过"]
    try:
        import numpy as np
        import sounddevice as sd
        rec = sd.rec(int(1.0 * C.RATE), samplerate=C.RATE, channels=2,
                     device=in_dev, dtype="int16")
        sd.wait()
        x = rec.astype(np.float64) / 32768.0
        print(f"  1 秒静默录音: RMS {C.dbfs(x):7.1f} dBFS  峰值 {abs(x).max():.6f}")
        if devices.C.is_silent(x[:, 0]):
            return ["UAC 输出全零：ESP-NOW 链路假死，请断电重启 Scepter"]
        print("  OK 有数据")
    except Exception as exc:
        return [f"录音失败: {exc}"]
    return []


def check_channel_routing() -> list[str]:
    section("4. 通道声学响应（逐个放音实测）")
    try:
        info = devices.probe_channels(seconds=1.6)
    except Exception as exc:
        return [f"探测失败: {exc}"]
    base = info["baseline"]
    print(f"  静音基线: L {base['L']:.1f} / R {base['R']:.1f} dBFS")
    print(f"\n  {'通道':<14s} {'L':>8s} {'R':>8s} {'提升':>8s}  结论")
    print("  " + "-" * 56)
    problems = []
    for ch in range(4):
        c = info["channels"][ch]
        ok = c["delta_db"] >= 6.0
        print(f"  {c['name']:<14s} {c['L']:8.1f} {c['R']:8.1f} {c['delta_db']:+7.1f} dB  "
              f"{'OK' if ok else '无响应'}")
        if ch in (1, 2, 3) and not ok:
            problems.append(f"{c['name']} 在麦克风端无响应")
    if problems:
        problems.append("排查顺序：音箱接线 -> 接口控制面板该路音量 -> Windows 端点实例 -> 隔离门")
    return problems


def check_asr() -> list[str]:
    section("5. ASR 引擎")
    try:
        import asr_harness as AH
        print("  native.dll:", AH.resolve_dll())
        eng = AH.AsrEngine(backend="vulkan")
        print(f"  OK family={eng.family} backend={eng.backend}")
        eng.close()
    except Exception as exc:
        return [f"引擎加载失败: {exc}（检查 SAC 是否拦了所有 native.dll 副本）"]
    return []


def check_history() -> list[str]:
    section("6. 历史采集完整性")
    base = os.path.join(C.ROOT, "spatial_eval")
    if not os.path.isdir(base):
        print("  （无 spatial_eval 目录）")
        return []
    bad = []
    for name in sorted(os.listdir(base)):
        d = os.path.join(base, name)
        if not os.path.isdir(d):
            continue
        info = C.check_capture_dir(d)
        if info["total"] == 0:
            continue
        if info["silent"]:
            bad.append(name)
            print(f"  {name:24s} {info['good']:3d}/{info['total']:<3d}  全零 {info['silent']} ⚠")
        else:
            print(f"  {name:24s} {info['good']:3d}/{info['total']:<3d}  ok")
    if bad:
        print("  提示：以上目录含全零文件，最早是 box_capture 的噪声路由 bug 造成，勿用于结论")
    return []


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="跳过需要放音的声学探测")
    args = ap.parse_args()

    problems: list[str] = []
    problems += check_env()
    problems += check_devices()
    if not problems:
        problems += check_uac_alive()
    if not problems and not args.quick:
        problems += check_channel_routing()
    problems += check_asr()
    check_history()

    section("结论")
    if problems:
        for p in problems:
            print("  FAIL", p)
        return 1
    print("  OK 全部通过，可以开始采集")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
