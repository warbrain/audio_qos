"""检测采集链路是否存在自动增益控制（AGC）或电平相关处理。

方法：
  1. 同一句语音按 −52 / −46 / −40 dBFS 三种电平各播放一次（不加噪声），
     测麦克风端 RMS。若比例与播放比例一致（各差 6 dB），说明无 AGC。
  2. 同一句在 −46 dBFS 下分别「加噪声 / 不加噪声」采集，比较**语音段的 RMS**。
     若带噪时语音被压低 >0.5 dB，说明增益随噪声改变。

注意：固件软限幅膝点为 −4.28 dBFS，因此电平必须足够低以避免削波干扰结论。
"""
from __future__ import annotations

import json
import os
import sys
import time
import wave

import numpy as np
import sounddevice as sd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from paired_capture import FS_CAP, FS_PLAY, pink, read_wav_any, resample, write_wav  # noqa: E402

MANIFEST = r"D:\App\cppApp\audio_qos\_asr_sim\v6_392.json"
OUT = r"D:\App\cppApp\audio_qos\_asr_sim\spatial_dir\agc_test"
PRE = 0.4
POST = 0.4


def pick_devices():
    do = di = None
    for d in sd.query_devices():
        n = "".join(c for c in d["name"].lower() if c.isalnum())
        if di is None and "whisprompt" in n and d["max_input_channels"] >= 2:
            di = d["index"]
        if do is None and d["max_output_channels"] == 4 and "us4x4" in n:
            do = d["index"]
    return do, di


def capture(audio, dev_out, dev_in):
    rec = []
    def cb(indata, frames, t, status):
        rec.append(indata[:, :2].copy())
    st = sd.InputStream(device=dev_in, channels=2, samplerate=FS_CAP, blocksize=2048, callback=cb)
    st.start()
    time.sleep(0.15)
    sd.play(audio, FS_PLAY, device=dev_out, blocking=True)
    time.sleep(0.4)
    st.stop(); st.close()
    return np.concatenate(rec) if rec else np.zeros((0, 2))


def band_rms(x, fs):
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    X[(f < 300) | (f > 3400)] = 0.0
    y = np.fft.irfft(X, len(x))
    return float(np.sqrt(np.mean(y ** 2)) + 1e-12)


def speech_region(cap, expect_s):
    """用能量找到播放起点，返回语音段（跳过前导）。"""
    fl = int(0.02 * FS_CAP)
    m = cap.mean(axis=1)
    n = len(m) // fl
    e = np.array([np.mean(m[i * fl:(i + 1) * fl] ** 2) for i in range(n)])
    floor = np.percentile(e, 10) + 1e-12
    st = 0
    for i in range(n - 2):
        if e[i] > floor * 25:
            st = i * fl
            break
    a = st + int(PRE * FS_CAP)
    b = min(a + int(expect_s * FS_CAP), len(m))
    return cap[a:b]


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    TEST_WAV = r'D:\App\cppApp\audio_qos\qwen3-asr-strict-v6\audio\train\S0038\S0038_M-0038-2_000844-001316.wav'
    sp, sr = read_wav_any(TEST_WAV)
    sp = resample(sp, sr, FS_PLAY)
    rms0 = float(np.sqrt(np.mean(sp.astype(np.float64) ** 2))) + 1e-12
    dur = len(sp) / FS_PLAY

    dev_out, dev_in = pick_devices()
    print(f"  in={dev_in} out={dev_out}  片段 {dur:.2f}s")

    rows = []
    for db in (-52.0, -46.0, -40.0):
        spx = (sp * (10 ** (db / 20.0) / rms0)).astype(np.float32)
        mouth = np.concatenate([np.zeros(int(PRE * FS_PLAY), np.float32), spx,
                                np.zeros(int(POST * FS_PLAY), np.float32)])
        audio = np.zeros((len(mouth), 4), np.float32)
        audio[:, 1] = mouth
        cap = capture(audio, dev_out, dev_in)
        seg = speech_region(cap, dur)[:, 0]
        lvl = 20 * np.log10(band_rms(seg, FS_CAP))
        pk = float(np.max(np.abs(seg)))
        rows.append((db, lvl, pk))
        print(f"  播放 {db:6.1f} dBFS -> 麦克风 {lvl:7.2f} dBFS  峰值 {pk:.3f}")

    print("\n  === 电平传递 ===（若无 AGC，相邻应差 6 dB）")
    for i in range(1, len(rows)):
        d_in = rows[i][0] - rows[i - 1][0]
        d_out = rows[i][1] - rows[i - 1][1]
        print(f"  {rows[i-1][0]:.0f}→{rows[i][0]:.0f} dBFS : 输入 {d_in:+.1f} 输出 {d_out:+.1f}"
              f"  {'⚠ 压缩/扩张' if abs(d_out - d_in) > 1.0 else '✅ 线性'}")

    # 有/无噪声对比
    print("\n  === 同电平下加噪声是否改变语音增益 ===")
    spx = (sp * (10 ** (-46.0 / 20.0) / rms0)).astype(np.float32)
    for tag, with_noise in (("无噪声", False), ("加噪声", True)):
        mouth = np.concatenate([np.zeros(int(PRE * FS_PLAY), np.float32), spx,
                                np.zeros(int(POST * FS_PLAY), np.float32)])
        audio = np.zeros((len(mouth), 4), np.float32)
        audio[:, 1] = mouth
        if with_noise:
            audio[:, 2] = pink(len(mouth), seed=7) * (10 ** (-14.0 / 20.0))
        cap = capture(audio, dev_out, dev_in)
        seg = speech_region(cap, dur)[:, 0]
        lvl = 20 * np.log10(band_rms(seg, FS_CAP))
        print(f"  {tag}: 语音段 {lvl:7.2f} dBFS   峰值 {np.max(np.abs(seg)):.3f}")
        write_wav(os.path.join(OUT, f"speech_{'noisy' if with_noise else 'clean'}.wav"), seg, FS_CAP)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
