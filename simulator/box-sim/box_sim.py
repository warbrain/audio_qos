#!/usr/bin/env python3
"""Simulate the 1 m^3 closed test box with pyroomacoustics.

Geometry (metres, 1 x 1 x 1 m shoebox):
    dual microphone pair   : 20 mm apart on the y axis, at (0.5, 0.5, 0.5)
    artificial mouth       : 5 cm in front of the pair, on the array axis
    noise speaker L / R    : two wide-band sources placed symmetrically

The simulator produces the same signal the Dongle UAC sends to the PC:
16 kHz stereo float, one channel per microphone.  Speech and noise are
propagated through the image-source RIRs, mixed at a target SNR, and written
next to a manifest that the existing ASR evaluation can consume.

Examples:
    python box_sim.py --speech-dir wavs --out sim --snr 10 --t60 0.3
    python box_sim.py --speech-dir wavs --out sim --report
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import wave

import numpy as np
import pyroomacoustics as pra
from scipy.signal import fftconvolve, resample_poly

FS = 48000
FS_OUT = 16000
MIC_SPACING = 0.020
MOUTH_DISTANCE = 0.05
ROOM = np.array([1.0, 1.0, 1.0])


def load_wav_mono(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as handle:
        channels, width, rate = handle.getnchannels(), handle.getsampwidth(), handle.getframerate()
        raw = handle.readframes(handle.getnframes())
    if width != 2:
        raise SystemExit(f"only 16-bit wav is supported: {path}")
    data = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if channels > 1:
        data = data.reshape(-1, channels).mean(axis=1)
    return data, rate


def resample(x: np.ndarray, src: int, dst: int) -> np.ndarray:
    if src == dst:
        return x
    from math import gcd
    g = gcd(src, dst)
    return resample_poly(x, dst // g, src // g).astype(np.float32)


def pink(n: int, rng: np.random.Generator, fs: int = FS) -> np.ndarray:
    white = rng.standard_normal(n)
    spectrum = np.fft.rfft(white)
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    spectrum[1:] /= np.sqrt(np.maximum(freqs[1:], 1.0))
    spectrum[0] = 0.0
    out = np.fft.irfft(spectrum, n)
    return (out / (np.max(np.abs(out)) + 1e-9)).astype(np.float32)


def build_room(t60: float) -> tuple[pra.Room, np.ndarray]:
    """Return the room and the 2x3 microphone position matrix."""
    absorption, max_order = pra.inverse_sabine(t60, ROOM)
    room = pra.ShoeBox(ROOM, fs=FS, materials=pra.Material(absorption),
                       max_order=min(max_order, 40), ray_tracing=False,
                       air_absorption=True)
    mic_center = np.array([0.5, 0.5, 0.5])
    mics = np.array([
        mic_center + np.array([0.0, -MIC_SPACING / 2, 0.0]),
        mic_center + np.array([0.0, +MIC_SPACING / 2, 0.0]),
    ]).T  # shape (3, 2)
    room.add_microphone_array(pra.MicrophoneArray(mics, fs=FS))
    mouth = mic_center + np.array([0.0, MIC_SPACING / 2 + MOUTH_DISTANCE, 0.0])
    room.add_source(mouth)
    # Two noise speakers, symmetric, ~40 cm from the array and off axis.
    room.add_source(np.array([0.15, 0.75, 0.5]))
    room.add_source(np.array([0.85, 0.75, 0.5]))
    room.compute_rir()
    return room, mics


def whisperize(x: np.ndarray, fs: int = FS) -> np.ndarray:
    """Approximate a whispered voice: remove the low band and lift presence."""
    spectrum = np.fft.rfft(x)
    freqs = np.fft.rfftfreq(len(x), 1.0 / fs)
    gain = np.ones_like(freqs)
    gain[freqs < 300.0] = np.clip(freqs[freqs < 300.0] / 300.0, 0.0, 1.0) ** 2
    band = (freqs >= 1500.0) & (freqs <= 4000.0)
    gain[band] = 1.8
    out = np.fft.irfft(spectrum * gain, len(x))
    return (out / (np.max(np.abs(out)) + 1e-9)).astype(np.float32)


def drr_db(rir: np.ndarray, fs: int = FS) -> float:
    """Approximate direct-to-reverberant ratio from the RIR envelope."""
    energy = np.square(rir.astype(np.float64))
    peak = int(np.argmax(energy))
    direct = energy[max(0, peak - 48):peak + 48].sum()
    total = energy.sum()
    reverb = max(total - direct, 1e-12)
    return 10.0 * math.log10(max(direct, 1e-12) / reverb)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speech-dir", default="")
    parser.add_argument("--out", default="sim-box")
    parser.add_argument("--snr", type=float, default=10.0)
    parser.add_argument("--t60", type=float, default=0.30)
    parser.add_argument("--noise", choices=("pink", "white"), default="pink")
    parser.add_argument("--coherent", action="store_true",
                        help="same noise signal on both speakers")
    parser.add_argument("--speech-level-dbfs", type=float, default=-20.0,
                        help="normalise the speech component at the microphones to this RMS level")
    parser.add_argument("--whisper", action="store_true",
                        help="apply a whisper-like spectral tilt (low cut + presence boost)")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--report", action="store_true", help="print geometry/RIR report and exit")
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    room, mics = build_room(args.t60)
    rir = room.rir  # rir[mic][source]
    if args.report:
        print(f"room {ROOM} m, t60={args.t60:g} s, fs={FS}")
        print(f"mic spacing {MIC_SPACING * 1000:.0f} mm, mouth distance {MOUTH_DISTANCE * 100:.0f} cm")
        for mic in range(2):
            for src, name in enumerate(("mouth", "speaker L", "speaker R")):
                r = np.asarray(rir[mic][src], dtype=np.float32)
                print(f"  mic{mic} <- {name:9s}: {len(r):6d} taps "
                      f"({len(r) / FS * 1000:.0f} ms), DRR={drr_db(r):6.1f} dB")
        return 0

    if not args.speech_dir or not os.path.isdir(args.speech_dir):
        raise SystemExit("--speech-dir is required unless --report is used")
    files = sorted(f for f in os.listdir(args.speech_dir) if f.lower().endswith(".wav"))
    if not files:
        raise SystemExit("no wav files found")
    if args.limit:
        files = files[: args.limit]

    os.makedirs(args.out, exist_ok=True)
    manifest = os.path.join(args.out, "manifest.csv")
    with open(manifest, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["file", "wav", "snr_db", "t60", "noise", "coherent", "speech_s"])
        for index, name in enumerate(files):
            speech, rate = load_wav_mono(os.path.join(args.speech_dir, name))
            speech = resample(speech, rate, FS)
            speech = speech / (np.max(np.abs(speech)) + 1e-9) * 0.5
            if args.whisper:
                speech = whisperize(speech, FS)
            noise_len = len(speech) + FS // 2
            left = pink(noise_len, rng) if args.noise == "pink" else \
                rng.standard_normal(noise_len).astype(np.float32) * 0.3
            right = left.copy() if args.coherent else (
                pink(noise_len, rng) if args.noise == "pink" else
                rng.standard_normal(noise_len).astype(np.float32) * 0.3)

            def pad_to(signal: np.ndarray, length: int) -> np.ndarray:
                if len(signal) >= length:
                    return np.asarray(signal[:length], dtype=np.float32)
                return np.pad(signal, (0, length - len(signal))).astype(np.float32)

            speech_sig = []
            noise_sig = []
            for mic in range(2):
                speech_rir = np.asarray(rir[mic][0], dtype=np.float32)
                speech_sig.append(pad_to(fftconvolve(speech, speech_rir), noise_len))
                acc = np.zeros(noise_len, dtype=np.float32)
                acc += pad_to(fftconvolve(left, np.asarray(rir[mic][1], dtype=np.float32)), noise_len)
                acc += pad_to(fftconvolve(right, np.asarray(rir[mic][2], dtype=np.float32)), noise_len)
                noise_sig.append(acc)

            target_rms = 10.0 ** (args.speech_level_dbfs / 20.0)
            for index in (0, 1):
                current = float(np.sqrt(np.mean(np.square(speech_sig[index], dtype=np.float64))))
                if current > 0.0:
                    speech_sig[index] = speech_sig[index] * (target_rms / current)
            speech_rms = target_rms
            noise_rms = float(np.sqrt(np.mean(np.square(noise_sig[0], dtype=np.float64))))
            gain = speech_rms / (10.0 ** (args.snr / 20.0)) / max(noise_rms, 1e-12)
            mix = np.stack([speech_sig[0] + noise_sig[0] * gain,
                            speech_sig[1] + noise_sig[1] * gain], axis=1)
            peak = float(np.max(np.abs(mix)))
            if peak > 0.95:
                mix *= 0.95 / peak
            stereo = np.stack([resample(mix[:, 0], FS, FS_OUT),
                               resample(mix[:, 1], FS, FS_OUT)], axis=1)
            path = os.path.join(args.out, f"{index:03d}_{os.path.splitext(name)[0]}.wav")
            pcm = np.clip(stereo * 32767.0, -32768, 32767).astype("<i2")
            with wave.open(path, "wb") as out:
                out.setnchannels(2)
                out.setsampwidth(2)
                out.setframerate(FS_OUT)
                out.writeframes(pcm.tobytes())
            writer.writerow([name, path, args.snr, args.t60, args.noise,
                             int(args.coherent), round(len(speech) / FS, 3)])
            print(f"  [{index + 1}/{len(files)}] {name} -> {path} "
                  f"(speech {speech_rms:.4f}, noise gain {gain:.2f})")
    print(f"manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
