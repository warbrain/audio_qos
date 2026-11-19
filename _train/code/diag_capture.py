"""诊断：完整保存一次同块采集，打印能量包络，确认两段语音的真实位置。"""
import json
import os
import sys
import time
import wave

import numpy as np
import sounddevice as sd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from paired_capture import FS_CAP, FS_PLAY, GAP, POST, PRE, pink, read_wav_any, resample, write_wav

MANIFEST = r"D:\App\cppApp\audio_qos\_asr_sim\v6_392.json"
OUT = r"D:\App\cppApp\audio_qos\_asr_sim\spatial_dir\paired_diag"


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    items = json.load(open(MANIFEST, encoding="utf-8"))[:1]
    it = items[0]

    dev_out = dev_in = None
    for d in sd.query_devices():
        n = "".join(c for c in d["name"].lower() if c.isalnum())
        if dev_in is None and "whisprompt" in n and d["max_input_channels"] >= 2:
            dev_in = d["index"]
        if dev_out is None and d["max_output_channels"] == 4 and "us4x4" in n:
            dev_out = d["index"]
    print(f"  in={dev_in} out={dev_out}")

    sp, sr = read_wav_any(it["audio"])
    sp = resample(sp, sr, FS_PLAY)
    rms = float(np.sqrt(np.mean(sp.astype(np.float64) ** 2))) + 1e-12
    sp = (sp * (10 ** (-40.0 / 20.0) / rms)).astype(np.float32)
    n_sp = len(sp)

    pre = np.zeros(int(PRE * FS_PLAY), dtype=np.float32)
    gap = np.zeros(int(GAP * FS_PLAY), dtype=np.float32)
    post = np.zeros(int(POST * FS_PLAY), dtype=np.float32)
    mouth = np.concatenate([pre, sp, gap, sp, post])
    noise = np.zeros(len(mouth), dtype=np.float32)
    start2 = len(pre) + n_sp + len(gap)
    noise[start2:] = pink(len(mouth) - start2, seed=1) * (10 ** (-14.0 / 20.0))

    audio = np.zeros((len(mouth), 4), dtype=np.float32)
    audio[:, 1] = mouth   # 仅人工嘴通道（实测 Out2 才是嘴）
    audio[:, 2] = noise
    print(f"  块长 {len(mouth)/FS_PLAY:.2f}s   第二段起点 {start2/FS_PLAY:.2f}s")

    rec = []
    flags = {"over": 0, "under": 0, "n": 0}
    def cb(indata, frames, t, status):
        flags["n"] += 1
        if status.input_overflow: flags["over"] += 1
        if status.input_underflow: flags["under"] += 1
        rec.append(indata[:, :2].copy())
    stream = sd.InputStream(device=dev_in, channels=2, samplerate=FS_CAP,
                            blocksize=2048, callback=cb)
    stream.start()
    time.sleep(0.1)
    sd.play(audio, FS_PLAY, device=dev_out, blocking=False)
    time.sleep(len(mouth) / FS_PLAY + 1.5)
    sd.stop()
    stream.stop()
    stream.close()
    cap = np.concatenate(rec) if rec else np.zeros((0, 2))
    print(f"  采集 {len(cap)/FS_CAP:.2f}s  回调 {flags['n']} 次  溢出 {flags['over']}  欠载 {flags['under']}")

    write_wav(os.path.join(OUT, "full_L.wav"), cap[:, 0], FS_CAP)
    write_wav(os.path.join(OUT, "full_R.wav"), cap[:, 1], FS_CAP)

    # 包络：50 ms 帧
    fl = int(0.05 * FS_CAP)
    mono = cap.mean(axis=1)
    env = [20 * np.log10(np.sqrt(np.mean(mono[i:i+fl] ** 2)) + 1e-12)
           for i in range(0, len(mono) - fl, fl)]
    print("  包络(dBFS, 每 50ms):")
    for i in range(0, len(env), 20):
        row = " ".join(f"{v:6.1f}" for v in env[i:i+20])
        print(f"   {i*0.05:5.2f}s {row}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
