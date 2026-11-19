"""谱减法 v3：用**每条录音自己的前导段**估计噪声 PSD。

为什么需要它：
  * 耳语片段语音占比约 92%，最小统计（低分位）取到的是语音，不是噪声 → v2 失败。
  * 但采集时 box_capture 有 0.6 s 前导段，那一段**只有噪声**，且电平与正片一致。
  * 用它估计噪声 PSD，再做功率谱减法，理论上可逼近 oracle_irm。
"""
from __future__ import annotations

import argparse
import os
import sys
import wave

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FS = 16000
N_FFT = 512
HOP = 160
ROOT = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir"


def rd(p):
    with wave.open(p, "rb") as w:
        ch = w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32) / 32768.0
    return x.reshape(-1, ch)[:, 0] if ch > 1 else x


def wr(p, x):
    y = np.clip(np.round(x * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(p, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(y.tobytes())


def block_start(x: np.ndarray, fl: int = 320) -> int:
    """用帧能量找播放起点（前导之前的数字静音之后第一段有声内容）。"""
    n = len(x) // fl
    if n < 10:
        return 0
    e = np.array([np.mean(x[i * fl:(i + 1) * fl] ** 2) for i in range(n)])
    floor = np.percentile(e, 10) + 1e-12
    for i in range(n - 2):
        if e[i] > floor * 25 and e[i + 1] > floor * 25:
            return i * fl
    return 0


def enhance(x: np.ndarray, alpha: float, beta: float) -> np.ndarray:
    win = torch.hann_window(N_FFT)
    t = torch.from_numpy(x)
    X = torch.stft(t, N_FFT, HOP, window=win, return_complex=True)
    P = X.abs().pow(2)

    st = block_start(x)
    a = st + int(0.05 * FS)
    b = min(st + int(0.55 * FS), len(x))
    if b - a < N_FFT * 2:                      # 前导太短，退回最小统计
        N = torch.quantile(P, 0.15, dim=1, keepdim=True)
    else:
        N = torch.stft(t[a:b], N_FFT, HOP, window=win, return_complex=True).abs().pow(2).mean(
            dim=1, keepdim=True)
    N = torch.clamp(N, min=1e-12)

    gain = torch.sqrt(torch.clamp(1.0 - alpha * N / (P + 1e-12), min=beta ** 2))
    return torch.istft(X * gain, N_FFT, HOP, window=win, length=len(x)).numpy()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--alpha", type=float, default=1.5)
    ap.add_argument("--beta", type=float, default=0.02)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    files = sorted(f for f in os.listdir(args.src) if f.endswith("_L.wav") and "_clean_" not in f)
    if args.limit:
        files = files[: args.limit]
    for f in files:
        x = rd(os.path.join(args.src, f))
        wr(os.path.join(args.out, f), enhance(x, args.alpha, args.beta))
    print(f"  pre-roll PSD  alpha={args.alpha} beta={args.beta}  {len(files)} 条 -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
