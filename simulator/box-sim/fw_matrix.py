#!/usr/bin/env python3
r"""Exercise every codec and spatial mode on the box unit and measure the result.

For each configuration the script sets the DSP parameter through the Dongle HID
interface, plays one sentence into the artificial mouth, records the Dongle UAC
stream, and reports level, channel correlation and the encoder time observed in
the Scepter telemetry.

Usage:
    python fw_matrix.py --speech D:\path\to\sentence.wav --port COM6
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
import wave
from pathlib import Path

import numpy as np
import serial
import sounddevice as sd

SR = 48000
OSR = 16000
MOUTH = 1
NOISE_L, NOISE_R = 2, 3
CODECS = {4: "G722-64k", 3: "Opus-32k", 5: "Opus-62k"}
MODES = {0: "bypass", 1: "DAS", 2: "NLMS", 3: "NLMS-stereo", 4: "DAS+Wiener", 5: "Wiener-stereo"}
DSP = str(Path(__file__).resolve().with_name("dsp_param.py"))


def find(name: str, want_out: bool) -> int:
    for index, info in enumerate(sd.query_devices()):
        if name in info["name"]:
            if want_out and info["max_output_channels"] >= 4:
                return index
            if not want_out and info["max_input_channels"] >= 2:
                return index
    raise SystemExit(f"device {name} not found")


def load(path: str) -> np.ndarray:
    with wave.open(path, "rb") as handle:
        channels = handle.getnchannels()
        rate = handle.getframerate()
        raw = handle.readframes(handle.getnframes())
    x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if channels > 1:
        x = x.reshape(-1, channels).mean(axis=1)
    k = int(round(len(x) * SR / rate))
    return np.interp(np.linspace(0, len(x) - 1, k), np.arange(len(x)), x).astype(np.float32)


def resample16(x: np.ndarray) -> np.ndarray:
    k = int(round(len(x) * OSR / SR))
    return np.interp(np.linspace(0, len(x) - 1, k), np.arange(len(x)), x).astype(np.float32)


def dsp(args: list[str]) -> None:
    subprocess.run([sys.executable, DSP] + args,
                   capture_output=True, timeout=30)


def play_record(speech: np.ndarray, out_dev: int, in_dev: int) -> tuple[float, float, float]:
    n = int(len(speech) + 1.0 * SR)
    buf = np.zeros((n, 4), dtype=np.float32)
    start = int(0.3 * SR)
    buf[start:start + len(speech), MOUTH] = speech * 0.18
    chunks: list[np.ndarray] = []

    def cb(indata, frames, time_info, status):  # noqa: ANN001
        chunks.append(indata.copy())

    stream = sd.InputStream(device=in_dev, channels=2, samplerate=SR, dtype="float32", callback=cb)
    stream.start()
    time.sleep(0.1)
    try:
        sd.play(buf, samplerate=SR, device=out_dev, blocking=True)
        time.sleep(0.2)
    finally:
        stream.stop()
        stream.close()
    rec = np.concatenate(chunks) if chunks else np.zeros((1, 2), np.float32)
    left, right = resample16(rec[:, 0]), resample16(rec[:, 1])
    peak = float(max(np.max(np.abs(left)), np.max(np.abs(right))))
    rms = float(np.sqrt(np.mean(np.stack([left, right]) ** 2)))
    corr = float(np.corrcoef(left, right)[0, 1]) if left.size > 2 else 0.0
    return peak, rms, corr


def encode_us(port: str, seconds: float = 4.0) -> int | None:
    ser = serial.Serial(port, 115200, timeout=0.2)
    end = time.time() + seconds
    found = None
    while time.time() < end:
        line = ser.readline()
        if not line:
            continue
        text = line.decode("utf-8", "replace")
        match = re.search(r"codec_perf.*?mean_us\":(\d+)", text)
        if match:
            found = int(match.group(1))
    ser.close()
    return found


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speech", required=True)
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--codecs", default="4,3,5")
    parser.add_argument("--modes", default="0,1,2,3,4,5")
    args = parser.parse_args()

    speech = load(args.speech)
    out_dev, in_dev = find("US-4x4HR", True), find("Audio QoS", False)
    print(f"speech {len(speech) / SR:.2f}s  out={out_dev} in={in_dev}")

    dsp(["--codec", "4"])
    dsp(["--mode", "0"])

    print("\n=== codecs ===")
    for codec in [int(v) for v in args.codecs.split(",") if v]:
        dsp(["--codec", str(codec)])
        time.sleep(0.4)
        peak, rms, corr = play_record(speech, out_dev, in_dev)
        us = encode_us(args.port)
        print(f"{CODECS.get(codec, codec):<11} encode_mean={us if us else '-':>6} us  "
              f"peak={peak:.2f} rms={20 * np.log10(rms + 1e-12):.1f} dBFS corr={corr:.3f}")

    dsp(["--codec", "4"])
    print("\n=== spatial modes (G722) ===")
    for mode in [int(v) for v in args.modes.split(",") if v.strip()]:
        dsp(["--mode", str(mode)])
        time.sleep(0.4)
        peak, rms, corr = play_record(speech, out_dev, in_dev)
        applied = subprocess.run([sys.executable, DSP, "--get", "19"],
                                 capture_output=True, text=True, timeout=30).stdout.strip()
        print(f"mode {mode} {MODES.get(mode, '?'):<12} applied[{applied}]  "
              f"peak={peak:.2f} rms={20 * np.log10(rms + 1e-12):.1f} dBFS corr={corr:.3f}")

    dsp(["--mode", "0"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
