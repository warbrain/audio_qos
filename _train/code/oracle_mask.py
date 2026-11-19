"""Oracle 实验：任何降噪方法的上界。

两种上界：
  1. oracle_mag   : 用干净幅度 + 含噪相位重建（幅度恢复的绝对上界）
  2. oracle_irm   : 理想比值掩码 IRM = |S|^2 / (|S|^2 + |N|^2)，N 由 含噪-干净 估计

若连 oracle 都把 CER 降不到很低，说明该噪声不可分离，任何模型都无法奏效。

用法（WSL）：
    python oracle_mask.py --out-root ../spatial_dir/oracle
"""
from __future__ import annotations

import argparse
import os
import sys
import wave

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import FS, estimate_lag, index_pairs, istft, read_wav, stft  # noqa: E402

ROOT = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir"


def write_wav(path: str, x: np.ndarray) -> None:
    y = np.clip(np.round(x * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(y.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-root", default=os.path.join(ROOT, "oracle"))
    args = ap.parse_args()

    pairs = index_pairs([os.path.join(ROOT, "m5_2cm196"),
                         os.path.join(ROOT, "v6_new196")],
                        os.path.join(ROOT, "quiet392"))
    print(f"  pairs={len(pairs)}")
    dev = "cpu"
    os.makedirs(os.path.join(args.out_root, "mag"), exist_ok=True)
    os.makedirs(os.path.join(args.out_root, "irm"), exist_ok=True)

    for k, (cid, lp, rp, qp) in enumerate(pairs, 1):
        l = read_wav(lp)
        r = read_wav(rp)
        t = read_wav(qp)
        lag = estimate_lag(l, t)
        if lag > 0:
            t = t[lag:]
        n = min(len(l), len(r), len(t))
        l, r, t = l[:n], r[:n], t[:n]
        noisy = 0.5 * (l + r)

        N = torch.from_numpy(noisy).unsqueeze(0).to(dev)
        S = torch.from_numpy(t).unsqueeze(0).to(dev)
        ns, ss = stft(N), stft(S)
        noise = ns - ss

        # 1) oracle magnitude + noisy phase
        y_mag = istft(torch.abs(ss) * torch.exp(1j * torch.angle(ns)), n)
        write_wav(os.path.join(args.out_root, "mag", cid + "_L.wav"), y_mag[0].numpy())

        # 2) ideal ratio mask
        mask = torch.abs(ss) ** 2 / (torch.abs(ss) ** 2 + torch.abs(noise) ** 2 + 1e-10)
        y_irm = istft(ns * mask, n)
        write_wav(os.path.join(args.out_root, "irm", cid + "_L.wav"), y_irm[0].numpy())

        if k % 100 == 0:
            print(f"  {k}", flush=True)
    print(f"  完成 {len(pairs)} 条 -> {args.out_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
