"""审计双麦采集数据集：L/R 对齐、电平、削顶，以及干净参考的覆盖情况。

用法：
    python audit_dataset.py --dirs dirA,dirB --sample 40
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import wave

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

import numpy as np


def read_wav(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as w:
        n, ch, sr = w.getnframes(), w.getnchannels(), w.getframerate()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch)[:, 0]
    return x, sr


def lag_estimate(a: np.ndarray, b: np.ndarray, max_lag: int = 64) -> tuple[int, float]:
    """互相关估计 b 相对 a 的延迟（样本）与归一化峰值。"""
    n = min(len(a), len(b))
    if n < 4096:
        return 0, 0.0
    a = a[:n] - a[:n].mean()
    b = b[:n] - b[:n].mean()
    best, best_lag = -1.0, 0
    na = np.linalg.norm(a) + 1e-12
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            x, y = a[: n - lag], b[lag:]
        else:
            x, y = a[-lag:], b[: n + lag]
        d = np.linalg.norm(x) * np.linalg.norm(y) + 1e-12
        c = float(np.dot(x, y) / d)
        if c > best:
            best, best_lag = c, lag
    return best_lag, best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dirs", required=True)
    ap.add_argument("--sample", type=int, default=40)
    ap.add_argument("--ref-dir", default="", help="干净参考目录（可选）")
    args = ap.parse_args()

    dirs = [d.strip() for d in args.dirs.split(",") if d.strip()]
    pairs = []
    for d in dirs:
        for f in sorted(os.listdir(d)):
            if f.endswith("_L.wav"):
                cid = f[: -len("_L.wav")]
                rp = os.path.join(d, cid + "_R.wav")
                if os.path.exists(rp):
                    pairs.append((cid, os.path.join(d, f), rp))
    print(f"  成对文件 {len(pairs)} 条")
    if not pairs:
        return 1
    step = max(1, len(pairs) // args.sample) if args.sample else 1
    sample = pairs[::step][: args.sample]

    lags, corrs, rms_l, rms_r, clips = [], [], [], [], 0
    for cid, lp, rp in sample:
        l, sr = read_wav(lp)
        r, _ = read_wav(rp)
        lag, corr = lag_estimate(l, r)
        lags.append(lag)
        corrs.append(corr)
        rms_l.append(20 * np.log10(np.sqrt(np.mean(l ** 2)) + 1e-12))
        rms_r.append(20 * np.log10(np.sqrt(np.mean(r ** 2)) + 1e-12))
        if np.max(np.abs(l)) > 0.94 or np.max(np.abs(r)) > 0.94:
            clips += 1

    lags = np.array(lags)
    corrs = np.array(corrs)
    print(f"  抽样 {len(sample)} 条   采样率 {sr}")
    print(f"  L/R 延迟: 中位 {np.median(lags):+.0f} 样本  范围 [{lags.min():+d}, {lags.max():+d}]"
          f"   |lag|>1 的比例 {100*np.mean(np.abs(lags)>1):.0f}%")
    print(f"  波形相关: 中位 {np.median(corrs):.4f}   最低 {corrs.min():.4f}")
    print(f"  L 电平: 中位 {np.median(rms_l):.1f} dBFS   R 电平: 中位 {np.median(rms_r):.1f} dBFS"
          f"   R-L 差 {np.median(np.array(rms_r)-np.array(rms_l)):+.1f} dB")
    print(f"  接近削顶(>0.94)的片段: {clips}/{len(sample)}")

    if args.ref_dir and os.path.isdir(args.ref_dir):
        refs = {f[: -len("_L.wav")] for f in os.listdir(args.ref_dir) if f.endswith("_L.wav")}
        ids = {c for c, _, _ in pairs}
        print(f"  干净参考覆盖: {len(ids & refs)}/{len(ids)} 条")
    else:
        print("  干净参考: 未提供（训练需要目标信号）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
