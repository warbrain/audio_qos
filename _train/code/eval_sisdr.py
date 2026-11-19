"""用 SI-SDR 评估增强效果（不依赖 ASR，避免 DLL 被策略拦截的问题）。

对比：原始左通道 vs 模型输出，各自相对"安静参考"的 SI-SDR。
参考信号先与含噪信号做互相关对齐（跨会话采集，存在残余偏移）。
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import FS, estimate_lag, read_wav  # noqa: E402

ROOT = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir"
SRC = [os.path.join(ROOT, "m5_2cm196"), os.path.join(ROOT, "v6_new196")]
QUIET = os.path.join(ROOT, "quiet392")
MODEL = os.path.join(ROOT, "model392_d")


def si_sdr(ref: np.ndarray, est: np.ndarray) -> float:
    n = min(len(ref), len(est))
    r, e = ref[:n].astype(np.float64), est[:n].astype(np.float64)
    a = np.dot(e, r) / (np.dot(r, r) + 1e-12)
    target = a * r
    noise = e - target
    return float(10 * np.log10((np.sum(target ** 2) + 1e-20) / (np.sum(noise ** 2) + 1e-20)))


def main() -> int:
    pairs = []
    for d in SRC:
        for f in sorted(os.listdir(d)):
            if f.endswith("_L.wav"):
                cid = f[: -len("_L.wav")]
                qp = os.path.join(QUIET, f)
                mp = os.path.join(MODEL, f)
                if os.path.exists(qp) and os.path.exists(mp):
                    pairs.append((cid, os.path.join(d, f), qp, mp))
    print(f"  可评估 {len(pairs)} 条")

    base, model = [], []
    for cid, sp, qp, mp in pairs:
        s = read_wav(sp).astype(np.float64)
        q = read_wav(qp).astype(np.float64)
        m = read_wav(mp).astype(np.float64)
        lag = estimate_lag(s, q, max_lag=8000, step=8)
        if lag:
            q = q[lag:]
        n = min(len(s), len(q), len(m))
        q, s, m = q[:n], s[:n], m[:n]
        base.append(si_sdr(q, s))
        model.append(si_sdr(q, m))

    base, model = np.array(base), np.array(model)
    d = model - base
    print(f"  原始左通道 SI-SDR: 中位 {np.median(base):+6.2f} dB")
    print(f"  模型输出   SI-SDR: 中位 {np.median(model):+6.2f} dB")
    print(f"  >>> 改善: 中位 {np.median(d):+.2f} dB   均值 {d.mean():+.2f} dB")
    print(f"      变好 {int(np.sum(d > 0))} 条 / 变差 {int(np.sum(d < 0))} 条")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
