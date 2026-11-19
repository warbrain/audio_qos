"""公平的 oracle 基准：用**仿真混合**（干净 + 真实噪声，真值完全已知）测量可达成收益。

之前所有 oracle 数字都受"跨会话采集"污染（干净参考与含噪不是同一次录音）。
这里改用 干净录音 + 真实噪声数字混合，噪声真值精确已知，得到干净的上界。

条件（同一批 60 条，SNR = +1 dB，对应当前实测工作点）：
  clean      : 干净原始（绝对上限）
  mix        : 混合后不做处理（基线）
  oracle_mag : 干净幅度 + 混合相位（幅度恢复的绝对上界）
  oracle_irm : 理想比值掩码 |S|^2/(|S|^2+|N|^2)
"""
from __future__ import annotations

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
CLEAN = os.path.join(ROOT, "quiet392")
NOISE = os.path.join(ROOT, "noise_only.wav")
OUT = os.path.join(ROOT, "fair_oracle")
SNR_DB = 1.0
LIMIT = 60


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


def stft(x):
    win = torch.hann_window(N_FFT)
    return torch.stft(torch.from_numpy(x), N_FFT, HOP, window=win, return_complex=True)


def istft(sp, n):
    win = torch.hann_window(N_FFT)
    return torch.istft(sp, N_FFT, HOP, window=win, length=n).numpy()


def main() -> int:
    rng = np.random.default_rng(0)
    noise = rd(NOISE)
    files = sorted(f for f in os.listdir(CLEAN) if f.endswith("_L.wav"))[:LIMIT]
    for k in ("clean", "mix", "oracle_mag", "oracle_irm"):
        os.makedirs(os.path.join(OUT, k), exist_ok=True)

    for f in files:
        s = rd(os.path.join(CLEAN, f)).astype(np.float64)
        n = len(s)
        st = rng.integers(0, max(1, len(noise) - n))
        nz = noise[st:st + n].astype(np.float64)
        nz *= np.sqrt(np.mean(s ** 2) / (np.mean(nz ** 2) * (10 ** (SNR_DB / 10)) + 1e-20))
        mix = s + nz
        pk = np.max(np.abs(mix))
        if pk > 0.95:
            mix *= 0.95 / pk
            s = s * (0.95 / pk)

        S = stft(s)
        X = stft(mix)
        N = X - S

        wr(os.path.join(OUT, "clean", f), s)
        wr(os.path.join(OUT, "mix", f), mix)
        wr(os.path.join(OUT, "oracle_mag", f), istft(torch.abs(S) * torch.exp(1j * torch.angle(X)), n))
        irm = torch.abs(S) ** 2 / (torch.abs(S) ** 2 + torch.abs(N) ** 2 + 1e-10)
        wr(os.path.join(OUT, "oracle_irm", f), istft(X * irm, n))

    print(f"  生成 {len(files)} 条 × 4 种条件 -> {OUT}  (SNR {SNR_DB:+.0f} dB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
