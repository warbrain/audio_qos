"""录制纯噪声（Out3 播放粉噪），用于构建"精确配对"的训练数据。

为什么要这样做：同块配对采集的"干净段"与"含噪段"是两次播放，
存在无法消除的差异（时钟、房间状态），模型会退化成恒等映射。
改用「干净录音 + 真实噪声数字混合」，目标就是干净录音本身，差异为零。
"""
from __future__ import annotations

import sys
import time
import wave

import numpy as np
import sounddevice as sd

FS_CAP = 16000
FS_PLAY = 48000


def pink(n: int, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    w = rng.standard_normal(n)
    spec = np.fft.rfft(w)
    f = np.fft.rfftfreq(n, 1.0 / FS_PLAY)
    spec[1:] /= np.sqrt(np.maximum(f[1:], 1.0))
    spec[0] = 0.0
    x = np.fft.irfft(spec, n).astype(np.float32)
    return x / (np.sqrt(np.mean(x ** 2)) + 1e-12)


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    out = sys.argv[2] if len(sys.argv) > 2 else r"_asr_sim\spatial_dir\noise_only.wav"

    dev_out = dev_in = None
    for d in sd.query_devices():
        n = "".join(c for c in d["name"].lower() if c.isalnum())
        if dev_in is None and "whisprompt" in n and d["max_input_channels"] >= 2:
            dev_in = d["index"]
        if dev_out is None and d["max_output_channels"] == 4 and "us4x4" in n:
            dev_out = d["index"]
    print(f"  in={dev_in} out={dev_out}  {seconds:.0f}s")

    n_play = int(seconds * FS_PLAY)
    noise = pink(n_play) * (10 ** (-14.0 / 20.0))
    audio = np.zeros((n_play, 4), dtype=np.float32)
    audio[:, 2] = noise          # Out3（实测 index2）

    rec = []
    def cb(indata, frames, t, status):
        rec.append(indata[:, :2].copy())

    stream = sd.InputStream(device=dev_in, channels=2, samplerate=FS_CAP,
                            blocksize=2048, callback=cb)
    stream.start()
    time.sleep(0.2)
    sd.play(audio, FS_PLAY, device=dev_out, blocking=True)
    time.sleep(0.5)
    stream.stop()
    stream.close()

    cap = np.concatenate(rec) if rec else np.zeros((0, 2))
    print(f"  采集 {len(cap)/FS_CAP:.1f}s")
    y = np.clip(np.round(cap * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(out, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(FS_CAP)
        w.writeframes(y.tobytes())
    lvl = 20 * np.log10(np.sqrt(np.mean(cap[:, 0] ** 2)) + 1e-12)
    print(f"  输出 {out}  带内电平 {lvl:.1f} dBFS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
