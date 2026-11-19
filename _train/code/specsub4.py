"""谱减法 v4：噪声估计用「前导 + 尾部」两段平均，参数更温和。

背景（本轮实测）：
  * 无 AGC，链路在膝点以下是线性的。
  * 噪声在段内会漂移 3.3 dB（前导 −41.4 / 尾部 −44.7 dBFS），单用前导会高估。
  * 仿真 oracle 显示：噪声估计准确时 IRM 可把 CER 从 6.46% 降到 0.00%。
  * v3 用前导单点估计 + α=1.5/β=0.02 过于激进，反而把语音减掉（5.57% vs 基线 2.58%）。

本版改进：
  1. 噪声 PSD = 前导段与尾部段功率谱的平均（降低漂移影响）
  2. 时频平滑增益，抑制"音乐噪声"
  3. α、β 更保守，提供扫描接口
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


def smooth_time(g: torch.Tensor, k: int) -> torch.Tensor:
    """沿时间轴滑动平均，抑制音乐噪声。"""
    if k <= 1:
        return g
    pad = k // 2
    v = torch.nn.functional.pad(g.unsqueeze(0), (pad, pad, 0, 0), mode="replicate")
    return torch.nn.functional.avg_pool2d(v, (1, k), stride=1).squeeze(0)


def enhance(x: np.ndarray, alpha: float, beta: float, smooth: int) -> np.ndarray:
    win = torch.hann_window(N_FFT)
    t = torch.from_numpy(x)
    X = torch.stft(t, N_FFT, HOP, window=win, return_complex=True)
    P = X.abs().pow(2)

    # 块起点
    fl = 320
    n = len(x) // fl
    e = np.array([np.mean(x[i * fl:(i + 1) * fl] ** 2) for i in range(n)]) if n else np.array([1.0])
    floor = np.percentile(e, 10) + 1e-12
    on = 0
    for i in range(n - 1):
        if e[i] > floor * 25:
            on = i * fl
            break

    def psd(a, b):
        a = max(0, a)
        b = min(len(x), b)
        if b - a < N_FFT * 2:
            return None
        seg = t[a:b]
        return torch.stft(seg, N_FFT, HOP, window=win, return_complex=True).abs().pow(2).mean(
            dim=1, keepdim=True)

    # 前导段只有 0.6 s（box_capture 的 --pre-roll），窗口必须落在其中，
    # 否则会混入语音（实测使噪声估计高 ~2 dB）。
    pre = psd(on + int(0.05 * FS), on + int(0.55 * FS))
    # 尾部只有在前导电平接近时才可信（部分录音被截断到语音区）。
    pre_lvl = float(torch.log10(pre.mean() + 1e-20)) if pre is not None else None
    post = psd(len(x) - int(0.55 * FS), len(x) - int(0.05 * FS))
    cand = []
    if pre is not None:
        cand.append(pre)
    if post is not None and pre is not None:
        d = 10 * (float(torch.log10(post.mean() + 1e-20)) - pre_lvl)
        if abs(d) < 3.0:          # 电平接近才认为是同一种噪声
            cand.append(post)
    if cand:
        N = torch.stack(cand).mean(dim=0)
    else:
        N = torch.quantile(P, 0.2, dim=1, keepdim=True)
    N = torch.clamp(N, min=1e-12)

    gain = torch.sqrt(torch.clamp(1.0 - alpha * N / (P + 1e-12), min=beta ** 2))
    gain = smooth_time(gain, smooth)
    return torch.istft(X * gain, N_FFT, HOP, window=win, length=len(x)).numpy()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--alpha", type=float, default=1.0)
    ap.add_argument("--beta", type=float, default=0.05)
    ap.add_argument("--smooth", type=int, default=5)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    files = sorted(f for f in os.listdir(args.src) if f.endswith("_L.wav") and "_clean_" not in f)
    if args.limit:
        files = files[: args.limit]
    for f in files:
        wr(os.path.join(args.out, f), enhance(rd(os.path.join(args.src, f)),
                                              args.alpha, args.beta, args.smooth))
    print(f"  a={args.alpha} b={args.beta} s={args.smooth}  {len(files)} 条 -> {os.path.basename(args.out)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
