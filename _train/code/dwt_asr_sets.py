"""生成 DWT 降噪的 ASR 评测集：干净语音 + 已知粉噪，再分别用软阈值/BayesShrink 降噪。

输出目录：<out>/snr{N}_noisy | _soft | _bayes
文件名沿用 <id>_L.wav，可直接被 _asr_sim/bench/score.py 评分。
"""
from __future__ import annotations

import argparse
import os
import sys
import wave

import numpy as np
import pywt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import FS, read_wav  # noqa: E402
from dwt_controlled import denoise_bayes, denoise_soft, pink_noise, snr  # noqa: E402

QUIET = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir/quiet392"


def write_wav(path: str, x: np.ndarray) -> None:
    y = np.clip(np.round(x * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(y.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir/dwt_asr")
    ap.add_argument("--clips", type=int, default=40)
    ap.add_argument("--snr", default="10,0")
    args = ap.parse_args()

    files = sorted(f for f in os.listdir(QUIET) if f.endswith("_L.wav"))[: args.clips]
    print(f"  干净样本 {len(files)} 条")

    for target in [float(s) for s in args.snr.split(",")]:
        tag = f"snr{int(target):+d}"
        for kind in ("noisy", "soft", "bayes"):
            os.makedirs(os.path.join(args.out, f"{tag}_{kind}"), exist_ok=True)
        gains = {"noisy": [], "soft": [], "bayes": []}
        for i, f in enumerate(files):
            clean = read_wav(os.path.join(QUIET, f)).astype(np.float64)
            noise = pink_noise(len(clean), seed=i) * np.sqrt(
                np.mean(clean ** 2) / (10 ** (target / 10)))
            noisy = clean + noise
            soft = denoise_soft(noisy)
            bayes = denoise_bayes(noisy)
            cid = f[: -len("_L.wav")]
            for kind, sig in (("noisy", noisy), ("soft", soft), ("bayes", bayes)):
                write_wav(os.path.join(args.out, f"{tag}_{kind}", cid + "_L.wav"), sig)
                gains[kind].append(snr(clean, sig))
        print(f"  {tag}: " + "  ".join(f"{k}={np.mean(v):+.2f}dB" for k, v in gains.items()))
    print(f"  输出 -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
