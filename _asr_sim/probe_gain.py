"""Determine the dual-mic firmware's actual runtime fixed gain.

The compile default is CONFIG_MIC_OUTPUT_GAIN_X=8, but mic_audio_load_output_gain()
lets a persisted NVS value ("fixed_gain_x") override it, so the running value can
differ. Reading it back over HID is unreliable on this host (the HID class driver
returns the cached SET payload), so we measure instead: set a known gain, capture
one clip, and compare levels. A 3x change in the multiplier moves the level by
9.5 dB, which is unambiguous.

Usage: python probe_gain.py
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import wave

import numpy as np

ROOT = r"D:\App\cppApp\audio_qos\_asr_sim"
PY_FULL = r"D:\App\cppApp\audio_qos\_praenv\Scripts\python.exe"
OUT = os.path.join(ROOT, "gain_probe")
WAV = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5\test\audio\hfm\whisper\S0045\S0045_M-0045-2_003060-003504.wav"

sys.path.insert(0, ROOT)
from set_spatial import open_config, request, DIR_WRITE, DIR_READ   # noqa: E402

ITEM_GAIN_L, ITEM_GAIN_R = 2, 3
VERSION_CONFIG = 1


def set_gain(h, gain_x: int) -> None:
    for item in (ITEM_GAIN_L, ITEM_GAIN_R):
        got, err = request(h, VERSION_CONFIG, item, gain_x, DIR_WRITE)
        print(f"  set item {item} = {gain_x}  (reply: {got if got is not None else err})", flush=True)


def capture(tag: str) -> float | None:
    d = os.path.join(OUT, tag)
    os.makedirs(d, exist_ok=True)
    proc = subprocess.run([PY_FULL, os.path.join(ROOT, "box_capture.py"),
                           "--wav", WAV, "--play-dbfs", "-30", "--out", d],
                          capture_output=True, text=True, encoding="utf-8", errors="replace")
    rows = [json.loads(l) for l in proc.stdout.splitlines() if l.startswith("{")]
    if not rows:
        print(f"  capture {tag}: FAILED ({proc.stderr.strip()[:120]})", flush=True)
        return None
    p = os.path.join(d, f"{os.path.splitext(os.path.basename(WAV))[0]}_L.wav")
    with wave.open(p, "rb") as w:
        n, ch = w.getnframes(), w.getnchannels()
        x = np.frombuffer(w.readframes(n), dtype="<i2").astype(np.float64) / 32768
    if ch > 1:
        x = x.reshape(-1, ch)[:, 0]
    seg = x[int(1.5 * 16000):int(3.0 * 16000)]
    rms = 20 * np.log10(np.sqrt(np.mean(seg ** 2)) + 1e-12)
    peak = 20 * np.log10(np.abs(x).max() + 1e-12)
    print(f"  capture {tag}: 语音段 RMS {rms:6.1f} dBFS, 峰值 {peak:6.1f} dBFS", flush=True)
    return rms


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    h = open_config()
    print("=== 1) 当前增益下采集 ===", flush=True)
    base = capture("current")

    print("=== 2) 设为 1x ===", flush=True)
    set_gain(h, 1)
    one = capture("gain1")

    print("=== 3) 设为 8x ===", flush=True)
    set_gain(h, 8)
    eight = capture("gain8")

    print("=== 4) 恢复 8x（编译默认）===", flush=True)
    set_gain(h, 8)
    h.close()

    if base is None or one is None or eight is None:
        print("采集失败，无法判定")
        return 1
    d_conv = eight - one          # 8x 与 1x 的实测差，应约 18.06 dB
    print()
    print(f"实测 8x − 1x = {d_conv:+.2f} dB（理论 +18.06 dB）")
    print(f"当前增益相对 1x = {base - one:+.2f} dB")
    best = min((1, 2, 4, 8, 16, 32), key=lambda g: abs(20 * np.log10(g) - (base - one)))
    print(f"→ 当前运行增益最接近 **{best}x**（{20*np.log10(best):+.2f} dB）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
