"""在对齐后的同块配对数据上测真实语音期间 SNR（残差法）。

这次 clean 与 noisy 来自同一次录音、并已互相关对齐（r>=0.5），
残差 = noisy − clean 可以真正代表语音期间的噪声。

输出：带内 SNR 分布 + 按相关分档的对比。
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import FS, estimate_lag, read_wav  # noqa: E402

D = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir/train600_paired"
IDS = "/mnt/d/App/cppApp/audio_qos/_asr_sim/train504_ids.json"


def band_power(x: np.ndarray) -> float:
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1.0 / FS)
    X[(f < 300) | (f > 3400)] = 0.0
    y = np.fft.irfft(X, len(x))
    return float(np.mean(y ** 2)) + 1e-20


def main() -> int:
    keep = json.load(open(IDS, encoding="utf-8"))
    print(f"  使用过滤后的 {len(keep)} 对")

    rows = []
    for item in keep:
        cid = item["id"]
        cp = os.path.join(D, cid + "_clean_L.wav")
        npth = os.path.join(D, cid + "_L.wav")
        if not (os.path.exists(cp) and os.path.exists(npth)):
            continue
        c = read_wav(cp).astype(np.float64)
        n = read_wav(npth).astype(np.float64)
        lag = estimate_lag(n, c, max_lag=64, step=1)
        if lag:
            n = n[lag:] if lag > 0 else n[:lag]
        m = min(len(c), len(n))
        c, n = c[:m], n[:m]

        # 最小二乘尺度，消除电平差
        scale = float(np.dot(n, c) / (np.dot(c, c) + 1e-12))
        resid = n - scale * c
        sp = band_power(scale * c)
        npow = band_power(resid)
        rows.append({
            "id": cid,
            "r": item["r"],
            "snr": 10 * np.log10(sp / npow),
            "noise_dbfs": 10 * np.log10(npow),
            "speech_dbfs": 10 * np.log10(sp),
            "scale": scale,
        })

    snr = np.array([x["snr"] for x in rows])
    print(f"  样本 {len(rows)} 对")
    print(f"  >>> 真实带内 SNR: 中位 {np.median(snr):+.2f} dB   "
          f"四分位 {np.percentile(snr,25):+.2f} ~ {np.percentile(snr,75):+.2f}")
    print(f"      噪声@麦 中位 {np.median([x['noise_dbfs'] for x in rows]):.1f} dBFS")
    print(f"      语音@麦 中位 {np.median([x['speech_dbfs'] for x in rows]):.1f} dBFS")
    print(f"      尺度因子 中位 {np.median([x['scale'] for x in rows]):.3f}")

    hi = [x["snr"] for x in rows if x["r"] >= 0.7]
    lo = [x["snr"] for x in rows if x["r"] < 0.7]
    if hi:
        print(f"      高相关(r>=0.7, {len(hi)} 对) 中位 SNR {np.median(hi):+.2f} dB")
    if lo:
        print(f"      低相关(r<0.7,  {len(lo)} 对) 中位 SNR {np.median(lo):+.2f} dB")
    print(f"\n  参照：前导段 5 分位法曾估得 +19.64 dB（旧跨会话数据）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
