"""测量同一段采集里「前导噪声 / 语音期间 / 尾部噪声」的电平，验证噪声是否恒定。"""
from __future__ import annotations

import os
import sys
import time
import wave

import numpy as np
import sounddevice as sd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from paired_capture import FS_CAP, FS_PLAY, pink, read_wav_any, resample  # noqa: E402

TEST_WAV = r"D:\App\cppApp\audio_qos\qwen3-asr-strict-v6\audio\train\S0038\S0038_M-0038-2_000844-001316.wav"
OUT = r"D:\App\cppApp\audio_qos\_asr_sim\spatial_dir\agc_test"
PRE, POST = 1.0, 1.0


def band_rms(x, fs=FS_CAP):
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    X[(f < 300) | (f > 3400)] = 0.0
    y = np.fft.irfft(X, len(x))
    return float(np.sqrt(np.mean(y ** 2)) + 1e-12)


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    do = di = None
    for d in sd.query_devices():
        n = "".join(c for c in d["name"].lower() if c.isalnum())
        if di is None and "whisprompt" in n and d["max_input_channels"] >= 2:
            di = d["index"]
        if do is None and d["max_output_channels"] == 4 and "us4x4" in n:
            do = d["index"]

    sp, sr = read_wav_any(TEST_WAV)
    sp = resample(sp, sr, FS_PLAY)
    rms0 = float(np.sqrt(np.mean(sp.astype(np.float64) ** 2))) + 1e-12
    sp = (sp * (10 ** (-46.0 / 20.0) / rms0)).astype(np.float32)

    mouth = np.concatenate([np.zeros(int(PRE * FS_PLAY), np.float32), sp,
                            np.zeros(int(POST * FS_PLAY), np.float32)])
    audio = np.zeros((len(mouth), 4), np.float32)
    audio[:, 1] = mouth
    audio[:, 2] = pink(len(mouth), seed=3) * (10 ** (-14.0 / 20.0))

    rec = []
    def cb(indata, frames, t, status):
        rec.append(indata[:, :2].copy())
    st = sd.InputStream(device=di, channels=2, samplerate=FS_CAP, blocksize=2048, callback=cb)
    st.start(); time.sleep(0.15)
    sd.play(audio, FS_PLAY, device=do, blocking=True)
    time.sleep(0.5); st.stop(); st.close()
    cap = (np.concatenate(rec) if rec else np.zeros((0, 2)))[:, 0]

    y = np.clip(np.round(cap * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(os.path.join(OUT, "full_block.wav"), "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(FS_CAP); w.writeframes(y.tobytes())

    fl = int(0.02 * FS_CAP)
    n = len(cap) // fl
    e = np.array([np.mean(cap[i * fl:(i + 1) * fl] ** 2) for i in range(n)])
    floor = np.percentile(e, 5) + 1e-14
    idx = [i for i in range(n) if e[i] > floor * 25]
    on = idx[0] * fl if idx else 0

    sp_start = on + int(PRE * FS_CAP)
    sp_end = sp_start + int(len(sp) / FS_PLAY * FS_CAP)
    pre = cap[on + int(0.05 * FS_CAP): on + int(0.95 * FS_CAP)]
    mid = cap[sp_start: sp_end]
    post = cap[sp_end + int(0.05 * FS_CAP): min(sp_end + int(0.95 * FS_CAP), len(cap))]

    print(f"  采集 {len(cap)/FS_CAP:.2f}s  块起点 {on/FS_CAP:.2f}s")
    print(f"  前导（仅噪声）  {20*np.log10(band_rms(pre)):7.2f} dBFS")
    print(f"  语音期间        {20*np.log10(band_rms(mid)):7.2f} dBFS")
    print(f"  尾部（仅噪声）  {20*np.log10(band_rms(post)):7.2f} dBFS")
    print(f"  尾部 − 前导 = {20*np.log10(band_rms(post)) - 20*np.log10(band_rms(pre)):+.2f} dB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
