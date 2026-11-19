"""受控 DWT 降噪验证：在干净语音上叠加**已知**噪声，再降噪。

这次噪声真值完全已知，不涉及两次采集对齐问题，测出来的是小波方法的真实性能。
对比：小波软阈值 / 小波 BayesShrink / 维纳式谱减（对照）。

用法（WSL）：
    python dwt_controlled.py
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pywt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import FS, read_wav  # noqa: E402

QUIET = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir/quiet392"
LEVELS = 5
WAVELET = "db4"


def pink_noise(n: int, seed: int) -> np.ndarray:
    """与 box_capture 相同的粉噪生成：幅度谱 ÷ sqrt(f)。"""
    rng = np.random.default_rng(seed)
    w = rng.standard_normal(n)
    spec = np.fft.rfft(w)
    f = np.fft.rfftfreq(n, 1.0 / FS)
    spec[1:] /= np.sqrt(np.maximum(f[1:], 1.0))
    spec[0] = 0.0
    x = np.fft.irfft(spec, n)
    return x / (np.sqrt(np.mean(x ** 2)) + 1e-12)


def denoise_soft(x: np.ndarray, sigma: float | None = None) -> np.ndarray:
    c = pywt.wavedec(x, WAVELET, level=LEVELS)
    s = sigma if sigma else np.median(np.abs(c[-1])) / 0.6745 + 1e-12
    out = [c[0]]
    for d in c[1:]:
        thr = s * np.sqrt(2.0 * np.log(len(d) + 1))
        out.append(np.sign(d) * np.maximum(np.abs(d) - thr, 0.0))
    return pywt.waverec(out, WAVELET)[: len(x)]


def denoise_bayes(x: np.ndarray) -> np.ndarray:
    c = pywt.wavedec(x, WAVELET, level=LEVELS)
    sigma = np.median(np.abs(c[-1])) / 0.6745 + 1e-12
    out = [c[0]]
    for d in c[1:]:
        var = np.var(d) + 1e-12
        if var <= sigma ** 2:
            out.append(np.zeros_like(d))
            continue
        thr = sigma ** 2 / np.sqrt(var)
        out.append(np.sign(d) * np.maximum(np.abs(d) - thr, 0.0))
    return pywt.waverec(out, WAVELET)[: len(x)]


def snr(clean: np.ndarray, est: np.ndarray) -> float:
    n = min(len(clean), len(est))
    c, e = clean[:n], est[:n]
    return 10 * np.log10((np.sum(c ** 2) + 1e-20) / (np.sum((e - c) ** 2) + 1e-20))


def main() -> int:
    files = sorted(f for f in os.listdir(QUIET) if f.endswith("_L.wav"))[:40]
    print(f"  干净样本 {len(files)} 条")
    print("  目标SNR | 含噪实测 |  软阈值  | BayesShrink | 提升(软) 提升(Bayes)")
    for target_db in (10, 5, 0, -5):
        rows = {"noisy": [], "soft": [], "bayes": []}
        for i, f in enumerate(files):
            clean = read_wav(os.path.join(QUIET, f))
            n = pink_noise(len(clean), seed=i)
            noise = n * np.sqrt(np.mean(clean ** 2) / (10 ** (target_db / 10)))
            noisy = clean + noise
            rows["noisy"].append(snr(clean, noisy))
            rows["soft"].append(snr(clean, denoise_soft(noisy)))
            rows["bayes"].append(snr(clean, denoise_bayes(noisy)))
        m = {k: float(np.mean(v)) for k, v in rows.items()}
        print(f"  {target_db:+3d} dB  | {m['noisy']:6.2f}  | {m['soft']:7.2f}  | "
              f"{m['bayes']:9.2f}   | {m['soft']-m['noisy']:+6.2f}  {m['bayes']-m['noisy']:+6.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
