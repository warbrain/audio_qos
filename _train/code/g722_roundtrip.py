"""G.722 回环测试：合成含噪信号经 G.722 编解码后再评分。

目的：区分"加性噪声本身"与"有损编码在噪声下的失真"对 CER 的贡献。
用 ffmpeg 的 g722 编码器（16 kHz / 64 kbps，与固件同规格）。

用法（WSL）：
    python g722_roundtrip.py
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import wave

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import FS, read_wav  # noqa: E402
from dwt_controlled import pink_noise  # noqa: E402

QUIET = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir/quiet392"
OUT = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir/g722_rt"


def write_wav(path: str, x: np.ndarray) -> None:
    y = np.clip(np.round(x * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(y.tobytes())


def g722_roundtrip(src: str, dst: str, tmp: str) -> bool:
    g7 = os.path.join(tmp, "a.g722")
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-i", src,
           "-ar", str(FS), "-ac", "1", "-c:a", "g722", "-b:a", "64k", g7]
    r = subprocess.run(cmd, capture_output=True)
    if r.returncode != 0:
        return False
    cmd2 = ["ffmpeg", "-y", "-loglevel", "error", "-i", g7,
            "-ar", str(FS), "-ac", "1", "-c:a", "pcm_s16le", dst]
    r2 = subprocess.run(cmd2, capture_output=True)
    return r2.returncode == 0


def main() -> int:
    if not shutil.which("ffmpeg"):
        print("  ffmpeg 不在 PATH 中")
        return 1
    files = sorted(f for f in os.listdir(QUIET) if f.endswith("_L.wav"))[:40]
    print(f"  样本 {len(files)} 条")

    tmp = tempfile.mkdtemp()
    for target in (10.0, 0.0):
        tag = f"snr{int(target):+d}"
        for kind in ("plain", "g722"):
            os.makedirs(os.path.join(OUT, f"{tag}_{kind}"), exist_ok=True)
        ok = 0
        for i, f in enumerate(files):
            clean = read_wav(os.path.join(QUIET, f)).astype(np.float64)
            noise = pink_noise(len(clean), seed=i) * np.sqrt(
                np.mean(clean ** 2) / (10 ** (target / 10)))
            noisy = clean + noise
            cid = f[: -len("_L.wav")]
            plain = os.path.join(OUT, f"{tag}_plain", cid + "_L.wav")
            write_wav(plain, noisy)
            dst = os.path.join(OUT, f"{tag}_g722", cid + "_L.wav")
            if g722_roundtrip(plain, dst, tmp):
                ok += 1
        print(f"  {tag}: G.722 回环成功 {ok}/{len(files)}")
    shutil.rmtree(tmp, ignore_errors=True)
    print(f"  输出 -> {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
