"""测量麦克风体检。

做四件事：
  1. 找出哪一路输入接了测量麦（逐通道电平对比）；
  2. 本底噪声（静默录音）；
  3. 频响：放对数扫频 100 Hz - 7.5 kHz，用逆滤波求传递函数，看平坦度；
  4. 给出结论：能不能用于声学测量（T60 / 距离标定 / 频响校准）。

用法：
  python check_measurement_mic.py
  python check_measurement_mic.py --in-device 1 --play-dbfs -20
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
import numpy as np
import sounddevice as sd

import measure_t60 as M

PLAY_RATE = 48000
SWEEP_S = 3.0
F0, F1 = 100.0, 7500.0


def log_sweep(fs: int, seconds: float, f0: float, f1: float) -> np.ndarray:
    """指数扫频（对数扫频），带淡入淡出。"""
    n = int(seconds * fs)
    t = np.arange(n) / fs
    k = math.log(f1 / f0) / seconds
    phase = 2 * np.pi * f0 * (np.exp(k * t) - 1.0) / k
    x = np.sin(phase)
    fade = int(0.02 * fs)
    env = np.ones(n)
    env[:fade] = np.linspace(0, 1, fade)
    env[-fade:] = np.linspace(1, 0, fade)
    return (x * env).astype(np.float32)


def record_play(block: np.ndarray, in_dev: int, out_dev: int, ch: int,
                fs: int, seconds: float, play_rate: int = PLAY_RATE) -> np.ndarray:
    frames = int((seconds + 0.8) * fs)
    buf = np.zeros((frames, ch), dtype=np.float32)
    pos = {"n": 0}

    def cb(indata, n, t, status):  # noqa: ARG001
        k = min(n, frames - pos["n"])
        if k > 0:
            buf[pos["n"]:pos["n"] + k] = indata[:k]
            pos["n"] += k

    st = sd.InputStream(device=in_dev, samplerate=fs, channels=ch,
                        dtype="float32", callback=cb)
    st.start()
    try:
        sd.play(block, play_rate, device=out_dev, blocking=True)
        time.sleep(0.4)
    finally:
        st.stop()
        st.close()
    return buf[:max(pos["n"], 1)]


def db(x: float) -> float:
    return 20 * math.log10(x + 1e-12)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-device", default="1")
    ap.add_argument("--out-device", default=None)
    ap.add_argument("--out-channel", type=int, default=1, help="1 = Out2 人工嘴")
    ap.add_argument("--play-dbfs", type=float, default=-20.0)
    ap.add_argument("--fs", type=int, default=0, help="强制采样率，0=设备默认")
    args = ap.parse_args()

    in_dev, info = M.resolve_device(args.in_device, "US-4x4HR", "in")
    out_dev, _ = M.resolve_device(args.out_device, "US-4x4HR", "out")
    if in_dev is None or out_dev is None:
        print("设备未找到")
        return 2
    ch = min(4, info["max_input_channels"])
    fs = args.fs or int(info[default_samplerate])
    out_info = sd.query_devices(out_dev)
    play_fs = args.fs or int(out_info[default_samplerate])
    print(f"  输入 [{in_dev}] {info['name']}  {ch} ch @ {fs} Hz")
    print(f"  输出 [{out_dev}] @{play_fs} Hz  激励通道 {args.out_channel}（1 = Out2 人工嘴）\n")

    # 1) 本底
    silence = np.zeros((int(1.0 * play_fs), 4), dtype=np.float32)
    quiet = record_play(silence, in_dev, out_dev, ch, fs, 1.0, play_fs)
    floors = [db(float(np.sqrt(np.mean(quiet[:, c].astype(np.float64) ** 2))))
              for c in range(ch)]

    # 2) 扫频
    sweep = log_sweep(play_fs, SWEEP_S, F0, F1) * (10 ** (args.play_dbfs / 20.0))
    pre, post = int(0.3 * play_fs), int(1.0 * play_fs)
    block = np.zeros((pre + len(sweep) + post, 4), dtype=np.float32)
    block[pre:pre + len(sweep), args.out_channel] = sweep
    rec = record_play(block, in_dev, out_dev, ch, fs, len(block) / play_fs, play_fs)

    levels = [db(float(np.sqrt(np.mean(rec[:, c].astype(np.float64) ** 2))))
              for c in range(ch)]
    best = int(np.argmax(levels))
    print(f"  {'通道':>4s} {'本底':>9s} {'扫频RMS':>9s} {'信噪比':>8s}  结论")
    print("  " + "-" * 48)
    for c in range(ch):
        snr = levels[c] - floors[c]
        verdict = "✅ 测量麦" if c == best and snr > 20 else ("—" if snr < 10 else "有信号")
        print(f"  ch{c:<2d} {floors[c]:8.1f} {levels[c]:8.1f} {snr:7.1f} dB  {verdict}")
    print(f"\n  → 选定 ch{best}\n")

    # 3) 频响（逆滤波求传递函数）
    x = rec[:, best].astype(np.float64)
    # 对齐：用互相关找扫频起点
    corr = np.correlate(x[: int((SWEEP_S + 1.5) * fs)], sweep.astype(np.float64), mode="valid")
    start = int(np.argmax(np.abs(corr)))
    seg = x[start:start + len(sweep)]
    if len(seg) < len(sweep):
        print("  扫频段截断，跳过频响")
        return 0
    nfft = 1 << int(math.ceil(math.log2(len(sweep))))
    X = np.fft.rfft(sweep.astype(np.float64), nfft)
    Y = np.fft.rfft(seg, nfft)
    H = Y / (X + 1e-12)
    f = np.fft.rfftfreq(nfft, 1.0 / play_fs)
    mag = 20 * np.log10(np.abs(H) + 1e-12)

    bands = [(100, 300), (300, 1000), (1000, 3000), (3000, 5000), (5000, 7500)]
    print("  频响（相对 1 kHz 归一）：")
    ref_idx = np.argmin(np.abs(f - 1000))
    ref = mag[ref_idx]
    vals = []
    for lo, hi in bands:
        m = (f >= lo) & (f < hi)
        if not m.any():
            continue
        v = float(np.median(mag[m] - ref))
        vals.append(v)
        print(f"    {lo:5d}-{hi:5d} Hz  {v:+6.1f} dB")
    spread = max(vals) - min(vals) if vals else 0.0
    print(f"\n  频响起伏（带内最大-最小）: {spread:.1f} dB")
    print(f"  本底: {floors[best]:.1f} dBFS   扫频信噪比: {levels[best]-floors[best]:.1f} dB")

    # 4) 结论
    print("\n  === 结论 ===")
    ok_snr = (levels[best] - floors[best]) > 30
    ok_flat = spread < 15
    if ok_snr and ok_flat:
        print("  ✅ 测量麦可用：信噪比充足、频响平坦，适合做 T60 / 距离标定")
    elif ok_snr:
        print(f"  ⚠ 测量麦可用但频响起伏 {spread:.0f} dB 偏大——做距离标定可以，做频响校准需先补偿")
    else:
        print("  ❌ 信噪比不足：调高接口输入增益，或把麦移近声源")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
