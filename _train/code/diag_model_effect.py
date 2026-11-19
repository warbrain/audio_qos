"""诊断：模型输出是否只是"左通道 × alpha"（即只学了调音量）。

若 alpha 明显 < 1 且与左通道波形相关接近 1，说明模型没有真正做谱处理，
CER 变差可能只是电平变化导致的。
"""
import os
import wave

import numpy as np

SRC = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir/m5_2cm196"
ENH = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir/model392_f"


def rd(p):
    with wave.open(p, "rb") as w:
        return np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64) / 32768.0


ids = [i for i in sorted({f[: -len("_L.wav")] for f in os.listdir(ENH) if f.endswith("_L.wav")})
       if os.path.exists(os.path.join(SRC, i + "_L.wav"))][:40]
alphas, corrs, rin, rout = [], [], [], []
for cid in ids:
    l = rd(os.path.join(SRC, cid + "_L.wav"))
    m = rd(os.path.join(ENH, cid + "_L.wav"))
    n = min(len(l), len(m))
    l, m = l[:n], m[:n]
    a = float(np.dot(m, l) / (np.dot(l, l) + 1e-12))
    alphas.append(a)
    lm, mm = l - l.mean(), m - m.mean()
    corrs.append(float(np.dot(mm, lm) / ((np.linalg.norm(mm) * np.linalg.norm(lm)) + 1e-12)))
    rin.append(float(np.sqrt(np.mean(l ** 2))))
    rout.append(float(np.sqrt(np.mean(m ** 2))))

print(f"  样本 {len(ids)} 条")
print(f"  输出 ≈ alpha × 左通道: alpha 中位 {np.median(alphas):.3f}  四分位 "
      f"{np.percentile(alphas, 25):.3f}~{np.percentile(alphas, 75):.3f}")
print(f"  与左通道波形相关: 中位 {np.median(corrs):.4f}  最低 {min(corrs):.4f}")
print(f"  电平: 输入 {20*np.log10(np.median(rin)+1e-12):.1f} dBFS -> "
      f"输出 {20*np.log10(np.median(rout)+1e-12):.1f} dBFS")
print(f"  能量变化: {10*np.log10(np.median(np.array(rout)**2)/(np.median(np.array(rin)**2)+1e-20)):+.1f} dB")
