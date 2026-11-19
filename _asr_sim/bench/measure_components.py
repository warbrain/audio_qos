"""5 cm 工位分量法标定（不依赖 pyroomacoustics / simcore）。

读 WAV 用标准库 wave，避免导入被 Application Control 拦截的 libroom。
"""
from __future__ import annotations

import os
import sys
import wave

import numpy as np


def load_wav(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as w:
        n = w.getnframes()
        ch = w.getnchannels()
        sr = w.getframerate()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    if ch > 1:
        x = x[::ch]
    return x, sr


def band_power(x: np.ndarray, fs: int, lo: float = 300.0, hi: float = 3400.0) -> float:
    if len(x) < 16:
        return 1e-20
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    X[(f < lo) | (f > hi)] = 0.0
    y = np.fft.irfft(X, len(x))
    return float(np.mean(y ** 2)) + 1e-20


def block_level(path: str) -> float | None:
    x, fs = load_wav(path)
    if len(x) == 0:
        return None
    n = int(0.02 * fs)
    k = len(x) // n
    if k < 40:
        return None
    e = np.array([band_power(x[i * n:(i + 1) * n], fs) for i in range(k)])
    floor = float(np.percentile(e, 10))
    thr = floor * 4.0
    st = 0
    for i in range(k - 2):
        if e[i] > thr and e[i + 1] > thr and e[i + 2] > thr:
            st = i
            break
    begin = st + int(0.7 / 0.02)
    if begin >= k - 5:
        return None
    return 10.0 * np.log10(float(np.mean(e[begin:])))


def main() -> int:
    base = sys.argv[1] if len(sys.argv) > 1 else r"D:\App\cppApp\audio_qos\_asr_sim\cal5cm"
    out = {}
    for tag, play in (("speech", -40.0), ("noise", -14.0)):
        d = os.path.join(base, tag)
        if not os.path.isdir(d):
            print(f"  {tag}: 目录不存在")
            continue
        fs_ = [f for f in os.listdir(d) if f.endswith("_L.wav")]
        if not fs_:
            print(f"  {tag}: 无 wav")
            continue
        lv = block_level(os.path.join(d, fs_[0]))
        if lv is None:
            print(f"  {tag}: 无法定位块")
            continue
        out[tag] = lv
        print(f"  {tag:8s} 播放 {play:6.1f} dBFS -> 麦克风带内 {lv:7.1f} dBFS   耦合 {lv - play:+.1f} dB")
    if len(out) == 2:
        snr = out["speech"] - out["noise"]
        print()
        print(f"  → 当前设置 SNR = {snr:+.1f} dB")
        print(f"  → 若要在 5 cm 下做 SNR 0 dB：语音播放 = {(-40.0) + (0 - snr):.1f} dBFS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
