"""检查模型增强输出的电平是否正常（诊断用）。"""
import os
import wave

import numpy as np

MODEL_DIR = r"_asr_sim\spatial_dir\model392"
SRC_DIRS = [r"_asr_sim\spatial_dir\m5_2cm196", r"_asr_sim\spatial_dir\v6_new196"]


def stats(path):
    with wave.open(path, "rb") as w:
        n = w.getnframes()
        x = np.frombuffer(w.readframes(n), dtype="<i2").astype(np.float64) / 32768.0
    return 20 * np.log10(np.sqrt(np.mean(x ** 2)) + 1e-12), float(np.max(np.abs(x))), n


files = sorted(f for f in os.listdir(MODEL_DIR) if f.endswith("_L.wav"))[:5]
src = {}
for d in SRC_DIRS:
    if os.path.isdir(d):
        for f in os.listdir(d):
            if f.endswith("_L.wav"):
                src[f] = os.path.join(d, f)

print("  文件                         模型RMS   峰值    |  原始RMS   峰值")
for f in files:
    r, p, n = stats(os.path.join(MODEL_DIR, f))
    if f in src:
        r0, p0, _ = stats(src[f])
        print(f"  {f[:26]:28s} {r:7.1f} {p:5.2f}  |  {r0:7.1f} {p0:5.2f}")
    else:
        print(f"  {f[:26]:28s} {r:7.1f} {p:5.2f}  |  (无源文件)")
