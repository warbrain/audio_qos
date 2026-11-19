"""验证含噪采集与安静参考的对齐是否生效。

对齐前两者时间错位，相关系数很低；对齐后应显著上升（同一句话、同一链路）。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import FS, estimate_lag, index_pairs, read_wav  # noqa: E402

ROOT = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir"
pairs = index_pairs([os.path.join(ROOT, "m5_2cm196"),
                     os.path.join(ROOT, "v6_new196")],
                    os.path.join(ROOT, "quiet392"))
print(f"  pairs={len(pairs)}")


def corr(a, b):
    n = min(len(a), len(b), FS * 3)
    a = a[:n] - a[:n].mean()
    b = b[:n] - b[:n].mean()
    return float(np.dot(a, b) / ((np.linalg.norm(a) * np.linalg.norm(b)) + 1e-12))


print("  id                              lag    对齐前   对齐后")
for cid, lp, rp, qp in pairs[:8]:
    l = read_wav(lp)
    t = read_wav(qp)
    mean = 0.5 * (l + read_wav(rp))
    before = corr(mean, t)
    lag = estimate_lag(l, t)
    after = corr(mean[lag:], t) if lag else before
    print(f"  {cid[:28]:30s} {lag:5d}  {before:7.4f}  {after:7.4f}")
