#!/usr/bin/env python3
"""Closed-box acoustic test harness for the Scepter + Dongle pair.

Routing (US-4x4HR, 4 channels @ 48 kHz, WASAPI):
    ch0 -> artificial mouth
    ch1 -> unused (silence)
    ch2 -> left noise speaker
    ch3 -> right noise speaker

The Dongle UAC microphone (16 kHz stereo) is recorded for every condition, so
each wav is exactly what the ASR pipeline receives.

Examples:
    python box_test.py --speech-dir wavs --out results --snr 10 --noise pink
    python box_test.py --speech-dir wavs --calibrate-only
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import time
import wave

import numpy as np
import sounddevice as sd

SR_OUT = 48000
SR_IN = 16000


def find_device(name_part: str, want_output: bool, prefer_rate: int = 0) -> int:
    """Resolve a device index by name; indices shift when USB devices are added."""
    needle = "".join(char for char in name_part.casefold() if char.isalnum())
    best = None
    for index, info in enumerate(sd.query_devices()):
        device_name = "".join(char for char in info["name"].casefold() if char.isalnum())
        if needle not in device_name:
            continue
        if want_output and info["max_output_channels"] < 4:
            continue
        if not want_output and info["max_input_channels"] < 2:
            continue
        if prefer_rate and int(info["default_samplerate"]) == prefer_rate:
            return index
        if best is None:
            best = index
    if best is None:
        raise SystemExit(f"device containing '{name_part}' not found")
    return best
MOUTH_CH, NOISE_L_CH, NOISE_R_CH = 1, 2, 3   # measured: card Out1 -> WAV ch2


def load_wav_mono(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as handle:
        channels, width, rate = handle.getnchannels(), handle.getsampwidth(), handle.getframerate()
        frames = handle.readframes(handle.getnframes())
    if width == 2:
        data = np.frombuffer(frames, dtype="<i2").astype(np.float32) / 32768.0
    elif width == 4:
        data = np.frombuffer(frames, dtype="<i4").astype(np.float32) / 2147483648.0
    elif width == 1:
        data = (np.frombuffer(frames, dtype="<u1").astype(np.float32) - 128.0) / 128.0
    else:
        raise SystemExit(f"unsupported sample width {width}: {path}")
    if channels > 1:
        data = data.reshape(-1, channels).mean(axis=1)
    return data, rate


def resample(x: np.ndarray, src: int, dst: int) -> np.ndarray:
    if src == dst or x.size == 0:
        return x
    n = int(round(x.size * dst / src))
    grid = np.linspace(0.0, x.size - 1.0, n, dtype=np.float64)
    return np.interp(grid, np.arange(x.size), x).astype(np.float32)


def noise_buffer(n: int, rng: np.random.Generator, kind: str) -> np.ndarray:
    if kind == "white":
        out = rng.standard_normal(n).astype(np.float32)
    else:
        white = rng.standard_normal(n)
        spectrum = np.fft.rfft(white)
        freqs = np.fft.rfftfreq(n, 1.0 / SR_OUT)
        spectrum[1:] /= np.sqrt(np.maximum(freqs[1:], 1.0))
        spectrum[0] = 0.0
        out = np.fft.irfft(spectrum, n).astype(np.float32)
    return out / (np.max(np.abs(out)) + 1e-9)


def rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x, dtype=np.float64)))) if x.size else 0.0


def dbfs(x: float) -> float:
    return 20.0 * math.log10(max(x, 1e-9))


def record_while(buf4: np.ndarray, out_device: int, in_device: int) -> np.ndarray:
    """Record the UAC microphone (48 kHz native) while playing the buffer."""
    chunks: list[np.ndarray] = []

    def callback(indata, frames, time_info, status):  # noqa: ANN001
        chunks.append(indata.copy())

    stream = sd.InputStream(device=in_device, channels=2, samplerate=SR_OUT,
                            dtype="float32", callback=callback)
    stream.start()
    time.sleep(0.15)
    try:
        sd.play(buf4, samplerate=SR_OUT, device=out_device, blocking=True)
        time.sleep(0.25)
    finally:
        stream.stop()
        stream.close()
    if not chunks:
        return np.zeros((0, 2), np.float32)
    raw = np.concatenate(chunks, axis=0)
    return np.stack([resample(raw[:, 0], SR_OUT, SR_IN),
                     resample(raw[:, 1], SR_OUT, SR_IN)], axis=1)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-device", type=int, default=-1)
    parser.add_argument("--in-device", type=int, default=-1)
    parser.add_argument("--speech-dir", required=True)
    parser.add_argument("--out", default="box-results")
    parser.add_argument("--snr", type=float, default=10.0)
    parser.add_argument("--noise", choices=("pink", "white"), default="pink")
    parser.add_argument("--coherent", action="store_true")
    parser.add_argument("--pre", type=float, default=0.5)
    parser.add_argument("--post", type=float, default=1.0)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--calibrate-only", action="store_true")
    args = parser.parse_args()
    if not os.path.isdir(args.speech_dir):
        raise SystemExit("--speech-dir does not exist")
    files = sorted(f for f in os.listdir(args.speech_dir) if f.lower().endswith(".wav"))
    if not files:
        raise SystemExit("no wav files found")
    if args.limit:
        files = files[: args.limit]

    if args.out_device < 0:
        args.out_device = find_device("US-4x4HR", True, 48000)
    if args.in_device < 0:
        args.in_device = find_device("Audio QoS", False, 48000)
    print(f"out device {args.out_device}: {sd.query_devices(args.out_device)['name']}")
    print(f"in  device {args.in_device}: {sd.query_devices(args.in_device)['name']}")

    rng = np.random.default_rng(7)
    first, rate = load_wav_mono(os.path.join(args.speech_dir, files[0]))
    speech = resample(first, rate, SR_OUT)

    mouth_gain = 0.25 / (np.max(np.abs(speech)) + 1e-9)   # keep peaks clear of the device limiter
    buf = np.zeros((len(speech) + SR_OUT, 4), dtype=np.float32)
    buf[SR_OUT // 2:SR_OUT // 2 + len(speech), MOUTH_CH] = speech * mouth_gain
    rec = record_while(buf, args.out_device, args.in_device)
    speech_rms = rms(rec[:, 0])
    print(f"speech-only  rms={speech_rms:.5f} ({dbfs(speech_rms):.1f} dBFS)  "
          f"recorded {len(rec) / SR_IN:.2f} s")

    probe = noise_buffer(2 * SR_OUT, rng, args.noise)
    buf = np.zeros((2 * SR_OUT, 4), dtype=np.float32)
    buf[:, NOISE_L_CH] = probe * 0.05
    buf[:, NOISE_R_CH] = probe * 0.05
    # Auto-trim the mouth drive so the device receives a healthy level without
    # reaching its 20000/32768 soft limiter.
    target_speech = 0.0794  # -22 dBFS
    mouth_gain = float(np.clip(mouth_gain * target_speech / max(speech_rms, 1e-6), 0.01, 2.0))
    rec = record_while(buf, args.out_device, args.in_device)
    noise_rms = rms(rec[:, 0])
    print(f"noise@0.05   rms={noise_rms:.5f} ({dbfs(noise_rms):.1f} dBFS)")

    if speech_rms < 1e-5:
        raise SystemExit("calibration failed: mouth signal not captured")
    if noise_rms < 1e-6:
        raise SystemExit("calibration failed: noise not captured")
    wanted = speech_rms / (10.0 ** (args.snr / 20.0))
    noise_gain = float(np.clip(0.05 * wanted / noise_rms, 0.0, 1.5))
    print(f"target SNR={args.snr:g} dB -> mouth_gain={mouth_gain:.3f} noise_gain={noise_gain:.4f}")
    if args.calibrate_only:
        return 0

    os.makedirs(args.out, exist_ok=True)
    manifest = os.path.join(args.out, "manifest.csv")
    with open(manifest, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["file", "wav", "snr_db", "noise", "coherent", "speech_s", "record_s"])
        for index, name in enumerate(files):
            audio, rate = load_wav_mono(os.path.join(args.speech_dir, name))
            speech = resample(audio, rate, SR_OUT)
            n = int((len(speech) + args.pre * SR_OUT + args.post * SR_OUT))
            left = noise_buffer(n, rng, args.noise) * noise_gain
            right = left.copy() if args.coherent else noise_buffer(n, rng, args.noise) * noise_gain
            buf = np.zeros((n, 4), dtype=np.float32)
            start = int(args.pre * SR_OUT)
            buf[start:start + len(speech), MOUTH_CH] = speech * mouth_gain
            buf[:, NOISE_L_CH] = left
            buf[:, NOISE_R_CH] = right
            rec = record_while(buf, args.out_device, args.in_device)
            path = os.path.join(args.out, f"{index:03d}_{os.path.splitext(name)[0]}.wav")
            pcm = np.clip(rec * 32767.0, -32768, 32767).astype("<i2")
            with wave.open(path, "wb") as out:
                out.setnchannels(2)
                out.setsampwidth(2)
                out.setframerate(SR_IN)
                out.writeframes(pcm.tobytes())
            writer.writerow([name, path, args.snr, args.noise, int(args.coherent),
                             round(len(speech) / SR_OUT, 3), round(len(rec) / SR_IN, 3)])
            print(f"  [{index + 1}/{len(files)}] {name} -> {path}")
    print(f"manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
