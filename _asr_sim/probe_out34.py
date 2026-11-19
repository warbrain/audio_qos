"""Probe the Out3/Out4 box speakers and how the dual-mic array hears them.

For each case (silence, Out3 only, Out4 only, both) it plays pink noise through
the US-4x4HR outputs while recording the WhisPrompt UAC stream, then reports the
captured level per microphone channel and the inter-channel delay. A working,
spatially separated pair shows a different level/arrival per speaker; identical
values on both speakers would mean the array cannot tell them apart.

Usage: python probe_out34.py
"""
from __future__ import annotations

import sys
import time

import numpy as np
import sounddevice as sd

RATE_PLAY = 48000
RATE_REC = 16000
DURATION_S = 3.0
LEVEL_DBFS = -30.0
OUT_CHANNELS = 4          # Out1=mouth, Out2=?, Out3=index2, Out4=index3
MOUTH_INDEX = 1
NOISE_INDICES = (2, 3)    # Out3, Out4


def pick(name: str, need_out: int, need_in: int, rate: int):
    best = None
    for i, d in enumerate(sd.query_devices()):
        if name.lower() not in d["name"].lower():
            continue
        if need_out and d["max_output_channels"] < need_out:
            continue
        if need_in and d["max_input_channels"] < need_in:
            continue
        score = 0 if int(d["default_samplerate"]) == rate else 1
        if best is None or score < best[0]:
            best = (score, i, d)
    return (best[1], best[2]) if best else (None, None)


def pink(n: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    white = rng.standard_normal(n)
    spec = np.fft.rfft(white)
    f = np.fft.rfftfreq(n, 1.0 / RATE_PLAY)
    f[0] = f[1] if len(f) > 1 else 1.0
    spec /= np.sqrt(f)
    x = np.fft.irfft(spec, n)
    return x / (np.sqrt(np.mean(x ** 2)) + 1e-12)


def tdoa(a: np.ndarray, b: np.ndarray, fs: int, max_lag: int = 8) -> float:
    a = a - a.mean()
    b = b - b.mean()
    if np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return float("nan")
    up = 16
    A = np.fft.irfft(np.fft.rfft(a), len(a) * up) * up
    B = np.fft.irfft(np.fft.rfft(b), len(b) * up) * up
    best, best_lag = -1e18, 0
    span = max_lag * up
    for lag in range(-span, span + 1):
        if lag >= 0:
            x, y = A[lag:], B
        else:
            x, y = A, B[-lag:]
        m = min(len(x), len(y))
        if m < 512:
            continue
        c = float(np.dot(x[:m], y[:m]))
        if c > best:
            best, best_lag = c, lag
    return best_lag / up


def run_case(label: str, active: tuple[int, ...]) -> dict:
    n_play = int(DURATION_S * RATE_PLAY)
    n_rec = int(DURATION_S * RATE_REC)
    buf = np.zeros((n_play, OUT_CHANNELS), dtype=np.float32)
    if active:
        noise = pink(n_play, 11) * (10 ** (LEVEL_DBFS / 20.0))
        for ch in active:
            buf[:, ch] = noise.astype(np.float32)
    rec = sd.rec(n_rec, samplerate=RATE_REC, channels=2, device=IN_DEV, dtype="int16")
    sd.play(buf, RATE_PLAY, device=OUT_DEV, blocking=False)
    sd.wait()
    time.sleep(0.2)
    x = rec[:, 0].astype(np.float64) / 32768.0
    y = rec[:, 1].astype(np.float64) / 32768.0
    skip = int(0.4 * RATE_REC)
    x, y = x[skip:], y[skip:]
    lv = 20 * np.log10(float(np.sqrt(np.mean(x ** 2))) + 1e-12)
    rv = 20 * np.log10(float(np.sqrt(np.mean(y ** 2))) + 1e-12)
    lag = tdoa(x, y, RATE_REC)
    return {"case": label, "L_dbfs": round(lv, 1), "R_dbfs": round(rv, 1),
            "LR_delta_db": round(lv - rv, 2), "tdoa_samples": round(lag, 3),
            "tdoa_ms": round(lag / RATE_REC * 1000, 3)}


def main() -> int:
    global OUT_DEV, IN_DEV
    OUT_DEV, _ = pick("US-4x4HR", OUT_CHANNELS, 0, RATE_PLAY)
    IN_DEV, _ = pick("WhisPrompt", 0, 2, RATE_REC)
    print(f"  out_device={OUT_DEV} in_device={IN_DEV}", flush=True)
    if OUT_DEV is None or IN_DEV is None:
        print("  device missing", file=sys.stderr)
        return 2

    cases = [
        ("silence", ()),
        ("out3_only", (NOISE_INDICES[0],)),
        ("out4_only", (NOISE_INDICES[1],)),
        ("both", NOISE_INDICES),
        ("mouth_ref", (MOUTH_INDEX,)),
    ]
    rows = []
    for label, active in cases:
        rows.append(run_case(label, active))
        r = rows[-1]
        print(f"  {r['case']:12s} L={r['L_dbfs']:6.1f} R={r['R_dbfs']:6.1f} "
              f"L-R={r['LR_delta_db']:+6.2f} dB  tdoa={r['tdoa_samples']:+7.3f} 样本", flush=True)
        time.sleep(0.6)

    base = rows[0]
    print("\n  === 相对静音的提升 ===")
    for r in rows[1:]:
        print(f"    {r['case']:12s} L {r['L_dbfs'] - base['L_dbfs']:+6.1f} dB   "
              f"R {r['R_dbfs'] - base['R_dbfs']:+6.1f} dB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
