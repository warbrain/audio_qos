"""双通道降噪训练数据：同域 16 kHz 立体声对 + 安静参考目标。

关键：含噪采集与安静参考是**两次独立采集**，播放起点不同，必须做互相关
对齐后切片；否则模型只能学到"输出静音"（随机时移的目标平均为零）。
"""
from __future__ import annotations

import os
import random
import wave

import numpy as np
import torch
from torch.utils.data import Dataset

FS = 16000
N_FFT = 512
HOP = 160

_CACHE: dict = {}   # pair index -> (L, R, target)


def read_wav(path: str) -> np.ndarray:
    with wave.open(path, "rb") as w:
        n, ch = w.getnframes(), w.getnchannels()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch)[:, 0]
    return x


def estimate_lag(a: np.ndarray, b: np.ndarray, max_lag: int = 8000, step: int = 8) -> int:
    """互相关估计 b 相对 a 的延迟（样本）；正数表示 b 需要前移。"""
    n = min(len(a), len(b), FS * 3)
    if n < FS:
        return 0
    a = a[:n] - a[:n].mean()
    b = b[:n] - b[:n].mean()
    best, best_lag = -1.0, 0
    for lag in range(0, max_lag + 1, step):
        x = a[lag:]
        y = b[: len(x)]
        c = float(np.dot(x, y) / ((np.linalg.norm(x) * np.linalg.norm(y)) + 1e-12))
        if c > best:
            best, best_lag = c, lag
    for lag in range(max(0, best_lag - step), min(max_lag, best_lag + step) + 1):
        x = a[lag:]
        y = b[: len(x)]
        c = float(np.dot(x, y) / ((np.linalg.norm(x) * np.linalg.norm(y)) + 1e-12))
        if c > best:
            best, best_lag = c, lag
    return best_lag


def index_pairs(noisy_dirs: list[str], quiet_dir: str,
                allowed: set[str] | None = None) -> list[tuple[str, str, str, str]]:
    """配对索引。支持两种布局：
      同块采集: <id>_clean_L.wav 与 <id>_L.wav 在同一目录
      旧布局  : 干净段在独立的 quiet 目录，文件名同为 <id>_L.wav
    allowed 为可选的 id 白名单（用于按对齐质量过滤）。"""
    out: list[tuple[str, str, str, str]] = []
    for d in noisy_dirs:
        for f in sorted(os.listdir(d)):
            if not f.endswith("_L.wav") or "_clean_" in f:
                continue
            cid = f[: -len("_L.wav")]
            rp = os.path.join(d, cid + "_R.wav")
            qp = os.path.join(quiet_dir, cid + "_clean_L.wav")
            if not os.path.exists(qp):
                qp = os.path.join(quiet_dir, cid + "_L.wav")
            if not (os.path.exists(rp) and os.path.exists(qp)):
                continue
            if allowed is not None and cid not in allowed:
                continue
            out.append((cid, os.path.join(d, f), rp, qp))
    return out


class PairedDataset(Dataset):
    """随机截取固定长度片段；训练时每次重新抽取，天然构成数据增强。"""

    def __init__(self, noisy_dirs: list[str], quiet_dir: str,
                 clip_s: float = 2.0, samples_per_clip: int = 4, seed: int = 0,
                 allowed: set[str] | None = None):
        self.pairs = index_pairs(noisy_dirs, quiet_dir, allowed)
        if not self.pairs:
            raise SystemExit(f"没有配对样本：{noisy_dirs} vs {quiet_dir}")
        self.n = int(clip_s * FS)
        self.spc = samples_per_clip
        self.rng = random.Random(seed)

    def __len__(self) -> int:
        return len(self.pairs) * self.spc

    def __getitem__(self, idx: int):
        key = idx % len(self.pairs)
        cached = _CACHE.get(key)
        if cached is None:
            cid, lp, rp, qp = self.pairs[key]
            l, r, t = read_wav(lp), read_wav(rp), read_wav(qp)
            lag = estimate_lag(l, t)
            if lag > 0:
                t = t[lag:]
            n0 = min(len(l), len(r), len(t))
            cached = (l[:n0].copy(), r[:n0].copy(), t[:n0].copy())
            _CACHE[key] = cached
        l, r, t = cached
        n = len(t)
        if n < self.n:
            pad = self.n - n
            l = np.pad(l, (0, pad))
            r = np.pad(r, (0, pad))
            t = np.pad(t, (0, pad))
            n = self.n
        start = self.rng.randint(0, max(0, n - self.n))
        sl = slice(start, start + self.n)
        return (torch.from_numpy(np.stack([l[sl], r[sl]])),
                torch.from_numpy(t[sl].copy()))


def stft(x: torch.Tensor) -> torch.Tensor:
    """x: (..., N) → complex (..., F, T)"""
    win = torch.hann_window(N_FFT, device=x.device, dtype=x.dtype)
    return torch.stft(x.reshape(-1, x.shape[-1]), N_FFT, HOP, window=win,
                      return_complex=True).view(*x.shape[:-1], N_FFT // 2 + 1, -1)


def istft(spec: torch.Tensor, length: int) -> torch.Tensor:
    win = torch.hann_window(N_FFT, device=spec.device, dtype=torch.float32)
    flat = spec.reshape(-1, spec.shape[-2], spec.shape[-1])
    y = torch.istft(flat, N_FFT, HOP, window=win, length=length)
    return y.view(*spec.shape[:-2], length)
