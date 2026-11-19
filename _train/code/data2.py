"""精确配对数据集：干净录音 + 真实噪声数字混合。

与旧的同块配对（两次播放）不同：
  目标 = 干净录音本身（样本级精确，无"另一次采集"的差异）
  输入 = 干净录音 + 从同一链路录到的真实噪声（noise_only.wav）

噪声每条随机取一段、随机 SNR，天然构成数据增强。
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
_CACHE: dict = {}


def read_wav(path: str) -> np.ndarray:
    with wave.open(path, "rb") as w:
        ch = w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch)
    return x


def stft(x: torch.Tensor) -> torch.Tensor:
    win = torch.hann_window(N_FFT, device=x.device, dtype=x.dtype)
    return torch.stft(x.reshape(-1, x.shape[-1]), N_FFT, HOP, window=win,
                      return_complex=True).view(*x.shape[:-1], N_FFT // 2 + 1, -1)


def istft(spec: torch.Tensor, length: int) -> torch.Tensor:
    win = torch.hann_window(N_FFT, device=spec.device, dtype=torch.float32)
    flat = spec.reshape(-1, spec.shape[-2], spec.shape[-1])
    y = torch.istft(flat, N_FFT, HOP, window=win, length=length)
    return y.view(*spec.shape[:-2], length)


class MixedPairDataset(Dataset):
    """从干净双通道录音 + 真实噪声构建精确配对。

    clean_dir   : 含 <id>_clean_L.wav / <id>_clean_R.wav 的目录
    noise_path  : 双通道噪声录音（noise_only.wav）
    snr_range   : (min, max) dB，每条随机取
    """

    def __init__(self, clean_dir: str, noise_path: str, clip_s: float = 2.0,
                 samples_per_clip: int = 4, snr_range: tuple[float, float] = (-5.0, 15.0),
                 seed: int = 0, ids: list[str] | None = None):
        self.ids = ids or sorted({f[: -len("_clean_L.wav")] for f in os.listdir(clean_dir)
                                  if f.endswith("_clean_L.wav")})
        if not self.ids:
            raise SystemExit(f"没有干净录音：{clean_dir}")
        self.dir = clean_dir
        self.n = int(clip_s * FS)
        self.spc = samples_per_clip
        self.snr = snr_range
        self.rng = random.Random(seed)

        nz = read_wav(noise_path)
        if nz.ndim == 1:
            nz = np.stack([nz, nz], axis=1)
        self.noise = nz.astype(np.float32)
        self._nz_cache: dict = {}

    def __len__(self) -> int:
        return len(self.ids) * self.spc

    def _clean(self, cid: str) -> np.ndarray:
        if cid not in _CACHE:
            l = read_wav(os.path.join(self.dir, cid + "_clean_L.wav"))
            r = read_wav(os.path.join(self.dir, cid + "_clean_R.wav"))
            l = l if l.ndim == 1 else l[:, 0]
            r = r if r.ndim == 1 else r[:, 0]
            n = min(len(l), len(r))
            _CACHE[cid] = np.stack([l[:n], r[:n]], axis=1)
        return _CACHE[cid]

    def __getitem__(self, idx: int):
        cid = self.ids[idx % len(self.ids)]
        clean = self._clean(cid)                       # (N,2)
        total = len(clean)
        if total < self.n:
            clean = np.pad(clean, ((0, self.n - total), (0, 0)))
            total = self.n
        st = self.rng.randint(0, total - self.n)
        seg = clean[st:st + self.n].copy()             # (n,2)

        # 随机噪声段 + 随机 SNR
        ns = self.rng.randint(0, len(self.noise) - self.n)
        nseg = self.noise[ns:ns + self.n].copy()       # (n,2)
        sp = float(np.mean(seg ** 2)) + 1e-12
        npow = float(np.mean(nseg ** 2)) + 1e-12
        target_snr = self.rng.uniform(*self.snr)
        nseg *= np.sqrt(sp / (npow * (10 ** (target_snr / 10))))

        mixed = seg + nseg
        peak = float(np.max(np.abs(mixed)))
        if peak > 0.95:
            mixed *= 0.95 / peak

        x = torch.from_numpy(mixed.T.astype(np.float32))   # (2,n)
        y = torch.from_numpy(seg[:, 0].astype(np.float32))  # 目标=干净左通道
        return x, y, target_snr
