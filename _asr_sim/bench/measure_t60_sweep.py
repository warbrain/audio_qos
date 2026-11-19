"""用扫频 + 反卷积测箱体 T60（比短突发可靠得多）。

原理
----
播放 3 秒对数扫频，记录后做正则化反卷积得到房间冲激响应（RIR），
再对 RIR 做 Schroeder 反向积分得到衰减曲线，拟合 T60 / EDT。

相比"中断噪声法"的优势：
* 扫频能量散布在 3 秒内，信噪比高（实测 37 dB）；
* 反卷积后 RIR 是干净的时间序列，不受激励包络拖尾影响；
* 可以同时得到 EDT（早期衰减，对噪声更稳健）。

用法：
  python measure_t60_sweep.py --in-device 8 --out-device 12 --fs 48000
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

SWEEP_S = 3.0
F0, F1 = 100.0, 7500.0


def log_sweep(fs: int, seconds: float, f0: float, f1: float) -> np.ndarray:
    n = int(seconds * fs)
    t = np.arange(n) / fs
    k = math.log(f1 / f0) / seconds
    x = np.sin(2 * math.pi * f0 * (np.exp(k * t) - 1.0) / k)
    fade = int(0.02 * fs)
    env = np.ones(n)
    env[:fade] = np.linspace(0, 1, fade)
    env[-fade:] = np.linspace(1, 0, fade)
    return (x * env).astype(np.float32)


def record(block: np.ndarray, in_dev: int, out_dev: int, ch: int, fs: int,
           seconds: float) -> np.ndarray:
    frames = int((seconds + 0.8) * fs)
    buf = np.zeros((frames, ch), dtype=np.float32)
    pos = {"n": 0}

    def cb(indata, n, t, s):  # noqa: ARG001
        k = min(n, frames - pos["n"])
        if k > 0:
            buf[pos["n"]:pos["n"] + k] = indata[:k]
            pos["n"] += k

    st = sd.InputStream(device=in_dev, samplerate=fs, channels=ch,
                        dtype="float32", callback=cb)
    st.start()
    try:
        sd.play(block, fs, device=out_dev, blocking=True)
        time.sleep(0.4)
    finally:
        st.stop()
        st.close()
    return buf[:max(pos["n"], 1)]


def deconvolve(rec: np.ndarray, sweep: np.ndarray, nfft: int) -> np.ndarray:
    X = np.fft.rfft(sweep.astype(np.float64), nfft)
    Y = np.fft.rfft(rec.astype(np.float64), nfft)
    lam = 1e-4 * float(np.max(np.abs(X) ** 2) + 1e-20)
    H = Y * np.conj(X) / (np.abs(X) ** 2 + lam)
    return np.fft.irfft(H, nfft)


def schroeder_t60(rir: np.ndarray, fs: int) -> tuple[float | None, float | None, dict]:
    """对 RIR 做 Schroeder 积分，返回 (T60, EDT, 诊断)。"""
    e = rir.astype(np.float64) ** 2
    peak = int(np.argmax(e))
    seg = e[peak:]
    if len(seg) < int(0.1 * fs):
        return None, None, {"note": "RIR 过短"}
    rev = np.cumsum(seg[::-1])[::-1]
    edc = 10 * np.log10(rev / (rev[0] + 1e-20) + 1e-20)
    total_db = edc[-1]
    # 只用高于本底 10 dB 的部分
    # 拟合到 -15 dB 更稳健：晚期尾巴常被瞬态/伪影污染
    lo_limit = max(-15.0, total_db + 10.0)

    def cross(level: float) -> float | None:
        idx = np.where(edc <= level)[0]
        if len(idx) == 0:
            return None
        i = int(idx[0])
        if i == 0:
            return 0.0
        y0, y1 = edc[i - 1], edc[i]
        return (i - 1 + ((level - y0) / (y1 - y0) if y1 != y0 else 0.0)) / fs

    t5, t25 = cross(-5.0), cross(lo_limit)
    t60 = None
    if t5 is not None and t25 is not None and t25 > t5:
        t60 = 60.0 * (t25 - t5) / (5.0 + (-lo_limit))
    t10 = cross(-10.0)
    edt = None
    if t10 is not None and t10 > 0:
        edt = 60.0 * t10 / 10.0
    return t60, edt, {"total_db": round(float(total_db), 1),
                      "lo_limit": round(lo_limit, 1),
                      "peak_index": peak}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-device", default="8")
    ap.add_argument("--out-device", default="12")
    ap.add_argument("--out-channel", type=int, default=1)
    ap.add_argument("--fs", type=int, default=48000)
    ap.add_argument("--play-dbfs", type=float, default=-20.0)
    ap.add_argument("--trials", type=int, default=6)
    ap.add_argument("--in-channels", type=int, default=4)
    args = ap.parse_args()

    in_dev, info = M.resolve_device(args.in_device, "US-4x4HR", "in")
    out_dev, _ = M.resolve_device(args.out_device, "US-4x4HR", "out")
    if in_dev is None or out_dev is None:
        print("设备未找到")
        return 2
    ch = min(args.in_channels, info["max_input_channels"])
    fs = args.fs
    print(f"  输入 [{in_dev}] {info['name']}  {ch} ch @ {fs} Hz")
    print(f"  输出 [{out_dev}]  通道 {args.out_channel}\n")

    sweep = log_sweep(fs, SWEEP_S, F0, F1) * (10 ** (args.play_dbfs / 20.0))
    pre, post = int(0.3 * fs), int(1.5 * fs)
    block = np.zeros((pre + len(sweep) + post, 4), dtype=np.float32)
    block[pre:pre + len(sweep), args.out_channel] = sweep
    nfft = 1 << int(math.ceil(math.log2(len(block) + len(sweep))))

    t60s, edts = [], []
    for i in range(args.trials):
        rec = record(block, in_dev, out_dev, ch, fs, len(block) / fs)
        lv = [20 * math.log10(float(np.sqrt(np.mean(rec[:, c].astype(np.float64) ** 2))) + 1e-12)
              for c in range(ch)]
        best = int(np.argmax(lv))
        rir = deconvolve(rec[:, best], sweep, nfft)
        t60, edt, diag = schroeder_t60(rir, fs)
        if i == 0:
            print(f"  通道电平: " + " ".join(f"ch{c}={v:.0f}" for c, v in enumerate(lv)))
            print(f"  诊断: EDC 总衰减 {diag.get('total_db')} dB，拟合下限 {diag.get('lo_limit')} dB")
        if t60:
            t60s.append(t60)
        if edt:
            edts.append(edt)
        print(f"  第 {i+1:2d} 次: ch{best}  T60 {t60*1000 if t60 else float('nan'):8.1f} ms   "
              f"EDT {edt*1000 if edt else float('nan'):8.1f} ms")
        time.sleep(0.15)

    print()
    if t60s:
        print(f"  ✅ T60 中位 {np.median(t60s)*1000:7.1f} ms   "
              f"四分位 {np.percentile(t60s,25)*1000:.0f}~{np.percentile(t60s,75)*1000:.0f} ms")
    else:
        print("  ❌ 无有效 T60")
    if edts:
        print(f"     EDT 中位 {np.median(edts)*1000:7.1f} ms")
    if t60s:
        med = float(np.median(t60s))
        print()
        print("  对照（20 cm → 5 cm 的总功率提升）：")
        print("    T60 0.05s→+8.2dB  0.10→+3.5  0.20→+0.9  0.30→+0.3")
        if med <= 0.10:
            print("  → 箱体接近无混响：缩短嘴距收益最大。")
        elif med <= 0.20:
            print("  → 中等混响：缩距离收益有限，优先吸声。")
        else:
            print("  → 混响偏强：先做吸声。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
