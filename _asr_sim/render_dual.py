"""Simulate dual-microphone captures of the 196-clip test set for the audio_qos
firmware's spatial modes.

Geometry matches the physical box and the earlier single-mic runs:
    1 x 1 x 1 m shoebox, two microphones 20 mm apart on the y axis,
    artificial mouth 5 cm in front of the array centre, cardioid directivity
    (p=0.5) aimed at the array, and the two noise speakers at
    (0.15, 0.75, 0.5) / (0.85, 0.75, 0.5).

Output is 16 kHz interleaved stereo PCM16 - exactly what
tools/asr_eval/host/win_spatial_wav.exe consumes (channel 0 = mic A,
channel 1 = mic B).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import wave

import numpy as np
import pyroomacoustics as pra
from pyroomacoustics.directivities import CardioidFamily, DirectionVector
from scipy.signal import resample_poly

FS = 48000
FS_OUT = 16000
ROOM = np.array([1.0, 1.0, 1.0])
CENTER = np.array([0.5, 0.5, 0.5])
MIC_SPACING = 0.020
MOUTH_DISTANCE = 0.05
SPK = (np.array([0.15, 0.75, 0.5]), np.array([0.85, 0.75, 0.5]))
TARGET_RMS_DBFS = -26.0
GAIN_CAP_DB = 55.0
DATASET = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5"


def build_room(t60: float, cap: int = 60, mouth_p: float | None = 0.5):
    absorption, order = pra.inverse_sabine(t60, ROOM)
    room = pra.ShoeBox(ROOM, fs=FS, materials=pra.Material(absorption),
                       max_order=min(order, cap), air_absorption=True)
    mics = np.array([CENTER + np.array([0.0, -MIC_SPACING / 2, 0.0]),
                     CENTER + np.array([0.0, +MIC_SPACING / 2, 0.0])]).T
    room.add_microphone_array(pra.MicrophoneArray(mics, fs=FS))
    mouth = CENTER + np.array([0.0, MOUTH_DISTANCE, 0.0])
    if mouth_p is None:
        room.add_source(mouth)
    else:
        aim = DirectionVector(azimuth=-90.0, colatitude=90.0, degrees=True)
        room.add_source(mouth, directivity=CardioidFamily(orientation=aim, p=mouth_p))
    room.add_source(SPK[0])
    room.add_source(SPK[1])
    room.compute_rir()
    return room, absorption, min(order, cap)


def read_mono48(path: str) -> np.ndarray:
    with wave.open(path, "rb") as w:
        rate, ch, n, width = w.getframerate(), w.getnchannels(), w.getnframes(), w.getsampwidth()
        raw = w.readframes(n)
    if width != 2:
        raise RuntimeError(f"expect 16-bit PCM: {path}")
    x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    if rate != FS:
        g = math.gcd(int(rate), FS)
        x = resample_poly(x, FS // g, int(rate) // g).astype(np.float32)
    return x


def pink(n: int, rng) -> np.ndarray:
    w = rng.standard_normal(n)
    spec = np.fft.rfft(w)
    f = np.fft.rfftfreq(n, 1.0 / FS)
    spec[1:] /= np.sqrt(np.maximum(f[1:], 1.0))
    spec[0] = 0.0
    x = np.fft.irfft(spec, n)
    return (x / (np.max(np.abs(x)) + 1e-9)).astype(np.float32)


def normalize(x: np.ndarray) -> np.ndarray:
    rms = float(np.sqrt(np.mean(x.astype(np.float64) ** 2)))
    if rms <= 0:
        return x
    gain_db = min(TARGET_RMS_DBFS - 20.0 * math.log10(rms), GAIN_CAP_DB)
    y = x * (10.0 ** (gain_db / 20.0))
    peak = float(np.max(np.abs(y)))
    if peak > 0.95:
        y = y * (0.95 / peak)
    return y.astype(np.float32)


def render(dry: np.ndarray, room, snr_db: float | None, seed: int) -> np.ndarray:
    n_len = len(dry) + FS // 2

    def fit(x):
        x = np.asarray(x, dtype=np.float32)
        return x[:n_len] if len(x) >= n_len else np.pad(x, (0, n_len - len(x)))

    chans = []
    for mic in range(2):
        speech = fit(np.convolve(dry, room.rir[mic][0], mode="full"))
        speech = normalize(speech)
        if snr_db is None:
            mixed = speech
        else:
            rng = np.random.default_rng(seed * 31 + mic)
            nz = fit(np.convolve(pink(n_len, rng), room.rir[mic][1], mode="full"))
            nz = nz + fit(np.convolve(pink(n_len, rng), room.rir[mic][2], mode="full"))
            srms = float(np.sqrt(np.mean(speech.astype(np.float64) ** 2)))
            nrms = float(np.sqrt(np.mean(nz.astype(np.float64) ** 2))) + 1e-12
            nz = nz * (srms / (10.0 ** (snr_db / 20.0)) / nrms)
            mixed = speech + nz
            peak = float(np.max(np.abs(mixed)))
            if peak > 0.95:
                mixed = mixed * (0.95 / peak)
        chans.append(resample_poly(mixed.astype(np.float32), FS_OUT, FS).astype(np.float32))
    n = min(len(chans[0]), len(chans[1]))
    stereo = np.stack([chans[0][:n], chans[1][:n]], axis=1)
    return stereo


CONDITIONS = [
    ("t030_snr10", 0.30, 10.0),
    ("t060_snr00", 0.60, 0.0),
    ("quiet", 0.30, None),
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=r"D:\App\cppApp\audio_qos\_asr_sim\dual_mic")
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    mf = json.load(open(os.path.join(DATASET, "heldout-manifest.strict-v5.json"), encoding="utf-8"))
    clips = mf["recordings"]
    if args.limit:
        clips = clips[: args.limit]
    os.makedirs(args.out, exist_ok=True)

    rooms = {}
    for name, t60, snr in CONDITIONS:
        if t60 in rooms:
            continue
        room, ab, order = build_room(t60)
        rooms[t60] = room
        print(f"# room T60={t60}: absorption={ab:.4f} order={order}", flush=True)

    manifest = []
    for cond, t60, snr in CONDITIONS:
        d = os.path.join(args.out, cond)
        os.makedirs(d, exist_ok=True)
        print(f"# rendering {cond}", flush=True)
        for i, c in enumerate(clips, 1):
            p = os.path.join(DATASET, c["audio"].replace("/", os.sep))
            dry = read_mono48(p)
            stereo = render(dry, rooms[t60], snr, seed=1000 + i)
            cid = os.path.splitext(os.path.basename(p))[0]
            out = os.path.join(d, f"{cid}.wav")
            with wave.open(out, "wb") as w:
                w.setnchannels(2)
                w.setsampwidth(2)
                w.setframerate(FS_OUT)
                w.writeframes(np.clip(stereo * 32767, -32768, 32767).astype("<i2").tobytes())
            if cond == CONDITIONS[0][0]:
                manifest.append({"id": cid, "speaker": c.get("speaker", ""),
                                 "reference": c["reference"], "audio": out})
            if i % 40 == 0:
                print(f"    {cond}: {i}/{len(clips)}", flush=True)
    with open(os.path.join(args.out, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)
    print(f"# wrote {len(manifest)} entries")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
