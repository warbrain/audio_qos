"""谱减法 v2：噪声 PSD 用**逐文件最小统计**估计（无需外部噪声录音）。

为什么：公平 oracle 实验显示，只要噪声估计准确，理想掩码能把 CER 从 6.46% 降到 0%。
之前谱减法用外部录的 noise_only.wav（比实际低 15 dB），根本减不够，所以失败。

方法（经典最小统计 + 过减 + 谱底）：
  1. 对每帧每个频点求功率
  2. 噪声 PSD ≈ 每频点的低分位（默认 15%），即"最安静的那些帧"
  3. 增益 = sqrt(max(1 - alpha*N/|X|^2, beta^2))
  4. 用带噪相位重建
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


def enhance(x: np.ndarray, alpha: float, beta: float, pct: float, smooth: int) -> np.ndarray:
    win = torch.hann_window(N_FFT)
    X = torch.stft(torch.from_numpy(x), N_FFT, HOP, window=win, return_complex=True)
    P = X.abs().pow(2)                                   # (F,T)

    # 最小统计：逐频点取低分位作为噪声 PSD
    N = torch.quantile(P, pct / 100.0, dim=1, keepdim=True)   # (F,1)
    if smooth > 1:                                            # 频向平滑，稳定估计
        pad = smooth // 2
        v = N.squeeze(-1).view(1, 1, -1)
        v = torch.nn.functional.pad(v, (pad, pad), mode="replicate")
        N = torch.nn.functional.avg_pool1d(v, smooth, stride=1).view(-1, 1)
    N = torch.clamp(N, min=1e-10)

    gain = torch.sqrt(torch.clamp(1.0 - alpha * N / (P + 1e-10), min=beta ** 2))
    Y = X * gain
    return torch.istft(Y, N_FFT, HOP, window=win, length=len(x)).numpy()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--alpha", type=float, default=2.0)
    ap.add_argument("--beta", type=float, default=0.02)
    ap.add_argument("--pct", type=float, default=15.0)
    ap.add_argument("--smooth", type=int, default=5)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    files = sorted(f for f in os.listdir(args.src) if f.endswith("_L.wav") and "_clean_" not in f)
    if args.limit:
        files = files[: args.limit]
    for f in files:
        x = rd(os.path.join(args.src, f))
        wr(os.path.join(args.out, f), enhance(x, args.alpha, args.beta, args.pct, args.smooth))
    print(f"  alpha={args.alpha} beta={args.beta} pct={args.pct} smooth={args.smooth}  "
          f"{len(files)} 条 -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
