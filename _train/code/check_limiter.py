"""对比含噪采集与安静采集的峰值/削波情况，验证软限幅器是否在噪声下被触发。"""
import os
import wave

import numpy as np

DIRS = {
    "noisy": r"_asr_sim\spatial_dir\m5_2cm196",
    "quiet": r"_asr_sim\spatial_dir\quiet392",
}
KNEE = 20000 / 32768.0   # 固件软限幅膝点
CEIL = 26000 / 32768.0   # 上限


def stats(path):
    with wave.open(path, "rb") as w:
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64) / 32768.0
    over_knee = float(np.mean(np.abs(x) > KNEE)) * 100
    at_ceil = float(np.mean(np.abs(x) >= CEIL * 0.999)) * 100
    return float(np.max(np.abs(x))), over_knee, at_ceil


rows = []
for name, d in DIRS.items():
    files = sorted(f for f in os.listdir(d) if f.endswith("_L.wav"))[:60]
    pk, kn, ce = [], [], []
    for f in files:
        a, b, c = stats(os.path.join(d, f))
        pk.append(a)
        kn.append(b)
        ce.append(c)
    rows.append((name, np.median(pk), np.mean(kn), np.mean(ce), len(files)))

print("  数据集   中位峰值   超膝点样本%   贴上限样本%   条数")
for name, p, k, c, n in rows:
    print(f"  {name:7s} {p:8.3f}   {k:10.3f}   {c:10.4f}   {n}")
print(f"\n  膝点 {KNEE:.3f}（−4.28 dBFS），上限 {CEIL:.3f}（−2.00 dBFS）")
