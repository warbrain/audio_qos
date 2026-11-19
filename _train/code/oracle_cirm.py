"""Oracle 复数掩码实验：验证"修相位"能把掩码路线的 4.74% 拉到多低。

变体：
  cirm_exact : 精确复数掩码（= 干净信号本身，仅作上界校验）
  psm        : 相位敏感掩码 clamp(Re(cIRM),0,1)（模型可预测的形态）
  psm_smooth : PSM 再做 3x3 平滑（更贴近真实模型的平滑性约束）
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


def smooth(m: torch.Tensor, k: int = 3) -> torch.Tensor:
    pad = k // 2
    m = torch.nn.functional.pad(m.unsqueeze(0), (pad, pad, pad, pad), mode="replicate")
    return torch.nn.functional.avg_pool2d(m, k, stride=1).squeeze(0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-root", default=os.path.join(ROOT, "oracle"))
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    pairs = index_pairs([os.path.join(ROOT, "m5_2cm196"),
                         os.path.join(ROOT, "v6_new196")],
                        os.path.join(ROOT, "quiet392"))
    if args.limit:
        pairs = pairs[: args.limit]
    print(f"  pairs={len(pairs)}")
    for v in ("cirm", "psm", "psm_smooth"):
        os.makedirs(os.path.join(args.out_root, v), exist_ok=True)

    for k, (cid, lp, rp, qp) in enumerate(pairs, 1):
        l = read_wav(lp)
        r = read_wav(rp)
        t = read_wav(qp)
        lag = estimate_lag(l, t)
        if lag > 0:
            t = t[lag:]
        n = min(len(l), len(r), len(t))
        noisy = 0.5 * (l[:n] + r[:n])
        clean = t[:n]

        N = torch.from_numpy(noisy).unsqueeze(0)
        S = torch.from_numpy(clean).unsqueeze(0)
        ns, ss = stft(N), stft(S)
        cirm = ss / (ns + 1e-8)

        y = istft(ns * cirm, n)
        write_wav(os.path.join(args.out_root, "cirm", cid + "_L.wav"), y[0].numpy())

        psm = torch.clamp(cirm.real, 0.0, 1.0).to(torch.complex64)
        y = istft(ns * psm, n)
        write_wav(os.path.join(args.out_root, "psm", cid + "_L.wav"), y[0].numpy())

        sm = smooth(torch.clamp(cirm.real, 0.0, 1.0)).to(torch.complex64)
        y = istft(ns * sm, n)
        write_wav(os.path.join(args.out_root, "psm_smooth", cid + "_L.wav"), y[0].numpy())

        if k % 100 == 0:
            print(f"  {k}", flush=True)
    print(f"  完成 {len(pairs)} 条")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
