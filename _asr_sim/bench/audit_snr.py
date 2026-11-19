"""逐条审计采集数据的实际带内信噪比（对齐无关的稳健估计）。

背景：采集时的 SNR 是按"播放电平 + 标定耦合"推算的，耦合随工位与频谱变化，
所以必须从录音本身复核。

算法（对每条录音独立计算）：
  1. 切成 20 ms 帧，计算带内（300-3400 Hz）帧能量；
  2. 播放起点：第一个持续 3 帧高于本底 +6 dB 的位置（本底取 10 分位）；
  3. 统计区域：播放起点之后 0.7 s 到文件末尾（跳过前导，避免边界效应）；
  4. 噪声功率 = 区域内帧能量的 **5 分位**（对应词间停顿，只有噪声）；
  5. 总功率 = 区域内帧能量的 **均值**；
  6. 语音功率 = 总功率 − 噪声功率；SNR = 10log10(语音/噪声)。

这种估计不依赖"语音段偏移"，只要区域内既有语音又有停顿即可。

用法：
    python audit_snr.py --dir ../snr_threshold/snr+0_m0
    python audit_snr.py --dir ../snr_threshold/snr+0_m0 --limit 50
"""
from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

import numpy as np

import simcore as S

BAND = (300.0, 3400.0)
FRAME_S = 0.02
SKIP_S = 0.7          # 跳过播放起点后的前 0.7 s（前导）
NOISE_PCT = 5.0       # 噪声取帧能量的低分位


def band_power(x: np.ndarray, fs: int) -> float:
    if len(x) < 16:
        return 1e-20
    X = np.fft.rfft(x.astype(np.float64))
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    X[(f < BAND[0]) | (f > BAND[1])] = 0.0
    y = np.fft.irfft(X, len(x))
    return float(np.mean(y ** 2)) + 1e-20


def audit_one(path: str) -> dict | None:
    x, fs = S.load_wav_mono(path)
    if len(x) == 0:
        return None
    x = x.astype(np.float64)
    n = int(FRAME_S * fs)
    frames = len(x) // n
    if frames < 40:
        return None
    e = np.array([band_power(x[i * n:(i + 1) * n], fs) for i in range(frames)])
    floor = float(np.percentile(e, 10))
    thr = floor * 4.0
    start_frame = 0
    for i in range(frames - 2):
        if e[i] > thr and e[i + 1] > thr and e[i + 2] > thr:
            start_frame = i
            break

    begin = start_frame + int(SKIP_S / FRAME_S)
    if begin >= frames - 20:
        return None
    region = e[begin:]

    noise = float(np.percentile(region, NOISE_PCT))
    total = float(np.mean(region))
    speech = max(total - noise, 1e-20)

    return {
        "snr": 10.0 * np.log10(speech / noise),
        "noise_db": 10 * np.log10(noise),
        "speech_db": 10 * np.log10(speech),
        "frames": len(region),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    files = sorted(f for f in os.listdir(args.dir) if f.endswith("_L.wav"))
    if args.limit:
        files = files[: args.limit]

    rows = []
    for f in files:
        r = audit_one(os.path.join(args.dir, f))
        if r:
            rows.append(r)
    if not rows:
        print("  没有可用的采集")
        return 1

    snrs = np.array([r["snr"] for r in rows])
    noise = np.array([r["noise_db"] for r in rows])
    speech = np.array([r["speech_db"] for r in rows])
    print(f"  目录 {os.path.basename(args.dir)}   审计 {len(rows)} 条")
    print(f"  带内 SNR  中位 {np.median(snrs):+6.2f} dB   "
          f"四分位 {np.percentile(snrs, 25):+.2f} ~ {np.percentile(snrs, 75):+.2f} dB")
    print(f"            最小 {snrs.min():+6.2f}   最大 {snrs.max():+.2f}   标准差 {snrs.std():.2f}")
    print(f"  噪声@麦   {np.median(noise):6.1f} dBFS")
    print(f"  语音@麦   {np.median(speech):6.1f} dBFS")

    hist, edges = np.histogram(snrs, bins=8)
    print("\n  分布：")
    for h, lo, hi in zip(hist, edges[:-1], edges[1:]):
        bar = "#" * int(40 * h / max(hist.max(), 1))
        print(f"    {lo:+6.1f} ~ {hi:+6.1f} dB  {h:3d}  {bar}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
