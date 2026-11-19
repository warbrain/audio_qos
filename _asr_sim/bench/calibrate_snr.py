"""当前工位声学标定：分别测语音与噪声到麦克风的耦合。

用途：隔离箱门缝开着、仅 Out3 播放噪声、嘴距 10 cm 的新配置下，
先量出两条通路的耦合，才能把 --play-dbfs / --noise-dbfs 设到目标 SNR。

做法：分两次播放
  1) 只在人工嘴放语音（Out2）
  2) 只在左声道放噪声（Out3）
各录一次，报告带内（300-3400 Hz）RMS 与耦合。
"""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "bench"))

import bootstrap  # noqa: F401
import numpy as np
import sounddevice as sd

import simcore as S

PLAY_FS = 48000
REC_FS = 16000
REC_CH = 2
SPEECH_DBFS = -30.0
NOISE_DBFS = -12.0


def pick_in() -> int:
    """复用 box_capture 的选择逻辑：设备名归一化后匹配 "whisprompt"（注意只有一个 p）。"""
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import box_capture

    dev, _info = box_capture.pick_device("WhisPrompt", 0, 2, 16000)
    if dev is None:
        raise SystemExit("WhisPrompt input not found")
    return int(dev)


def pick_out() -> int:
    best = None
    for i, d in enumerate(sd.query_devices()):
        n = "".join(c for c in d["name"].lower() if c.isalnum())
        if "us4x4" in n and d["max_output_channels"] >= 4:
            api = sd.query_hostapis(d["hostapi"])["name"].lower()
            if "wasapi" in api:
                return i
            best = best if best is not None else i
    if best is None:
        raise SystemExit("US-4x4 output not found")
    return best


def band_rms(x: np.ndarray, fs: int, lo: float = 300.0, hi: float = 3400.0) -> float:
    if len(x) == 0:
        return float("nan")
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    X[(f < lo) | (f > hi)] = 0.0
    y = np.fft.irfft(X, len(x))
    return float(np.sqrt(np.mean(y ** 2)))


def play_record(block: np.ndarray, in_dev: int, out_dev: int, seconds: float) -> np.ndarray:
    frames = int((seconds + 0.6) * REC_FS)
    buf = np.zeros((frames, REC_CH), dtype=np.float32)
    pos = {"n": 0}

    def cb(indata, n, t, s):  # noqa: ARG001
        k = min(n, frames - pos["n"])
        if k > 0:
            buf[pos["n"]:pos["n"] + k] = indata[:k]
            pos["n"] += k

    st = sd.InputStream(device=in_dev, samplerate=REC_FS, channels=REC_CH,
                        dtype="float32", callback=cb)
    st.start()
    try:
        sd.play(block, PLAY_FS, device=out_dev, blocking=True)
        time.sleep(0.4)
    finally:
        st.stop()
        st.close()
    return buf[:max(pos["n"], 1)].astype(np.float64)


def main() -> int:
    in_dev, out_dev = pick_in(), pick_out()
    print(f"  in [{in_dev}] out [{out_dev}]")

    items = S.load_manifest(1)
    wav, fs = S.load_wav_mono(items[0]["audio"])
    wav = wav.astype(np.float64)
    rms = np.sqrt(np.mean(wav ** 2)) + 1e-12
    speech = (wav / rms) * (10 ** (SPEECH_DBFS / 20.0))
    speech = np.concatenate([np.zeros(int(0.6 * fs)), speech, np.zeros(int(0.8 * fs))])

    rng = np.random.default_rng(7)
    n = len(speech)
    noise = rng.standard_normal(n)
    spec = np.fft.rfft(noise)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    spec[1:] /= np.sqrt(np.maximum(f[1:], 1.0))
    spec[0] = 0
    noise = np.fft.irfft(spec, n)
    noise = noise / (np.sqrt(np.mean(noise ** 2)) + 1e-12) * (10 ** (NOISE_DBFS / 20.0))
    if np.max(np.abs(noise)) > 0.95:
        noise *= 0.95 / np.max(np.abs(noise))

    def resample(x: np.ndarray, fs_in: int, fs_out: int) -> np.ndarray:
        n_out = int(len(x) * fs_out / fs_in)
        return np.interp(np.linspace(0, len(x) - 1, n_out), np.arange(len(x)), x)

    sp48 = resample(speech, fs, PLAY_FS)
    nz48 = resample(noise, fs, PLAY_FS)

    def run(sig: np.ndarray, ch: int, label: str) -> tuple[float, float]:
        block = np.zeros((len(sig), 4), dtype=np.float32)
        block[:, ch] = sig.astype(np.float32)
        rec = play_record(block, in_dev, out_dev, len(sig) / PLAY_FS)
        speech_win = rec[int(0.9 * REC_FS):int(0.9 * REC_FS + 2.0 * REC_FS), 0]
        lv = 20 * np.log10(band_rms(speech_win, REC_FS) + 1e-12)
        print(f"  {label:10s} 带内 RMS {lv:7.1f} dBFS")
        return lv, 0.0

    sp_lv, _ = run(sp48, 1, "speech")
    nz_lv, _ = run(nz48, 2, "noise")
    print()
    print(f"  语音耦合: {sp_lv - SPEECH_DBFS:+.1f} dB   (播放 {SPEECH_DBFS:.0f} dBFS)")
    print(f"  噪声耦合: {nz_lv - NOISE_DBFS:+.1f} dB   (播放 {NOISE_DBFS:.0f} dBFS, 仅 Out3)")
    print(f"  → 当前设置下的带内 SNR = {sp_lv - nz_lv:+.1f} dB")
    print()
    print("  目标 SNR 对应的播放电平（语音固定 -30 dBFS，调噪声）：")
    for snr in (10, 5, 0, -5, -10):
        need = sp_lv - snr
        print(f"    SNR {snr:+3d} dB -> 噪声需 {need:6.1f} dBFS  (即 --noise-dbfs {need:6.1f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
