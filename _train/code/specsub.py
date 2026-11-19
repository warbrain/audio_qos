"""经典谱减法 / 判决引导维纳滤波（无需训练）。

适用理由：本项目的噪声是**平稳**粉噪（同一音箱、同电平、同方位），
且我们已录到 90 s 纯噪声，可直接估计噪声 PSD —— 正符合谱减法的假设。

两种方法：
  specsub : 功率谱减法，增益 = sqrt(max(1 - alpha*N/|X|^2, beta^2))
  wiener  : 判决引导维纳滤波（decision-directed a priori SNR）

用法：
    python specsub.py --method specsub --alpha 2.0 --beta 0.05 --limit 60 \
        --src ../spatial_dir/m5_2cm196 --out ../spatial_dir/ss_specsub_a2

存储约定：输出目录放在 _asr_sim/spatial_dir/ 下（已被 .gitignore 忽略），
每个参数组合一个目录，命名 <method>_a<alpha>_b<beta>，便于对比后删除。
"""
from __future__ import annotations

import argparse
import os
import sys
import wave

import numpy as np
import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

FS = 16000
N_FFT = 512
HOP = 160
ROOT = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir"
NOISE = os.path.join(ROOT, "noise_only.wav")


def read_wav(path: str) -> np.ndarray:
    with wave.open(path, "rb") as w:
        ch = w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch)[:, 0]
    return x


def write_wav(path: str, x: np.ndarray) -> None:
    y = np.clip(np.round(x * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(y.tobytes())


def stft(x: torch.Tensor) -> torch.Tensor:
    win = torch.hann_window(N_FFT, device=x.device, dtype=x.dtype)
    return torch.stft(x, N_FFT, HOP, window=win, return_complex=True)


def istft(spec: torch.Tensor, length: int) -> torch.Tensor:
    win = torch.hann_window(N_FFT, device=spec.device, dtype=torch.float32)
    return torch.istft(spec, N_FFT, HOP, window=win, length=length)


def noise_psd(path: str, dev) -> torch.Tensor:
    x = torch.from_numpy(read_wav(path)).to(dev)
    return stft(x).abs().pow(2).mean(dim=1, keepdim=True)   # (F,1)


def enhance(x: np.ndarray, N: torch.Tensor, method: str, alpha: float,
            beta: float, dev) -> np.ndarray:
    t = torch.from_numpy(x).to(dev)
    X = stft(t)
    P = X.abs().pow(2)

    if method == "specsub":
        ratio = alpha * N / (P + 1e-10)
        gain = torch.sqrt(torch.clamp(1.0 - ratio, min=beta ** 2))
        Y = X * gain
    elif method == "wiener":
        gamma = P / (N + 1e-10)                       # 后验 SNR
        xi = torch.clamp(gamma - 1.0, min=beta ** 2)  # 先验 SNR（粗略）
        gain = xi / (1.0 + xi)
        Y = X * gain
    elif method == "dd":                              # 判决引导
        gamma = P / (N + 1e-10)
        S_prev = torch.zeros_like(gamma)
        gain_out = torch.zeros_like(gamma)
        for k in range(gamma.shape[1]):
            xi = 0.98 * S_prev[:, k] + 0.02 * torch.clamp(gamma[:, k] - 1.0, min=0.0)
            g = xi / (1.0 + xi)
            gain_out[:, k] = g
            S_prev[:, k] = (g ** 2) * gamma[:, k]
        Y = X * gain_out
    else:
        raise SystemExit(f"unknown method {method}")

    y = istft(Y, len(x))
    return y.cpu().numpy()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", default="specsub", choices=["specsub", "wiener", "dd"])
    ap.add_argument("--alpha", type=float, default=2.0)
    ap.add_argument("--beta", type=float, default=0.05)
    ap.add_argument("--src", default=os.path.join(ROOT, "m5_2cm196"))
    ap.add_argument("--out", default="")
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    N = noise_psd(NOISE, dev)
    out = args.out or os.path.join(ROOT, f"ss_{args.method}_a{args.alpha:g}_b{args.beta:g}")
    os.makedirs(out, exist_ok=True)

    files = sorted(f for f in os.listdir(args.src) if f.endswith("_L.wav") and "_clean_" not in f)
    if args.limit:
        files = files[: args.limit]
    for f in files:
        x = read_wav(os.path.join(args.src, f))
        y = enhance(x, N, args.method, args.alpha, args.beta, dev)
        write_wav(os.path.join(out, f), y)
    print(f"  {args.method} alpha={args.alpha} beta={args.beta}  {len(files)} 条 -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
