"""Field verification for the acoustic bench: is the noise field usable?

The spatial filter can only work if the two microphones see different noise.
This records the WhisPrompt capture stream and reports the numbers that decide
it:

  * per-channel level and the L/R imbalance (a leaky door shows up here),
  * the inter-channel correlation of the noise floor (0.6..0.75 is usable,
    0.98 means both cabinets play the same signal and no delay information
    exists),
  * the fractional-delay TDOA, which the 20 mm array cannot express with an
    integer lag at 16 kHz.

Run it with the rig playing noise only. The capture path goes through the
firmware's spatial mode, so treat the numbers as an upper bound on coherence:
the DSP can only remove differences, never create them.
"""

from __future__ import annotations

import argparse
import json
import time

import numpy as np
import sounddevice as sd

FS = 16000
VIRTUAL = ("primarysounddriver", "microsoftsoundmapper", "nahimic")


def _norm(s: str) -> str:
    return "".join(ch for ch in s.lower() if ch.isalnum())


def pick_capture(name: str) -> tuple[int, int]:
    """Prefer WASAPI at the device's native rate.

    The UAC runs at 16 kHz, but opening the MME/DirectSound entries of the same
    device returns digital silence (1 LSB), which made a dead stream look like a
    perfectly diffuse field. WASAPI enumerates at 48 kHz and does deliver audio,
    so pick it and stream at the rate the device reports.
    """
    aliases = [_norm(a) for a in name.split(",") if a.strip()]
    apis = {i: a["name"] for i, a in enumerate(sd.query_hostapis())}
    cands = []
    for i, d in enumerate(sd.query_devices()):
        if d["max_input_channels"] < 2:
            continue
        if not any(a in _norm(d["name"]) for a in aliases):
            continue
        api = apis[d["hostapi"]]
        score = 0 if "WASAPI" in api else (1 if "DirectSound" in api else 2)
        cands.append((score, i, api, int(d["default_samplerate"])))
    if not cands:
        raise SystemExit(f"capture device matching {name!r} not found")
    cands.sort()
    _, idx, api, rate = cands[0]
    print(f"  candidate {idx} {api} @{rate}")
    return idx, rate

def fractional_tdoa(a: np.ndarray, b: np.ndarray, rate: int) -> tuple[float, float]:
    """Sub-sample GCC-PHAT delay in microseconds and its peak sharpness."""
    seg = min(len(a), rate)
    a0 = a[:seg] - a[:seg].mean()
    b0 = b[:seg] - b[:seg].mean()
    n = 1 << int(np.ceil(np.log2(2 * seg)))
    A, B = np.fft.rfft(a0, n), np.fft.rfft(b0, n)
    R = A * np.conj(B)
    R /= np.maximum(np.abs(R), 1e-12)
    cc = np.fft.irfft(R, n)
    k = int(np.argmax(cc[: seg // 2]))
    y0, y1, y2 = cc[max(k - 1, 0)], cc[k], cc[min(k + 1, n - 1)]
    den = y0 - 2 * y1 + y2
    delta = 0.5 * (y0 - y2) / den if abs(den) > 1e-20 else 0.0
    peak = float(y1 / max(np.std(cc), 1e-20))
    return (k + float(np.clip(delta, -0.5, 0.5))) * 1e6 / rate, peak


def coherence(a: np.ndarray, b: np.ndarray, nbins: int = 8) -> list[float]:
    """Magnitude-squared coherence per frequency band."""
    win, hop = 512, 256
    frames = np.lib.stride_tricks.sliding_window_view
    def spec(x: np.ndarray) -> np.ndarray:
        f = frames(x, win)[::hop] * np.hanning(win)
        return np.fft.rfft(f, axis=1)
    A, B = spec(a), spec(b)
    cross = np.mean(np.conj(A) * B, axis=0)
    pa = np.mean(np.abs(A) ** 2, axis=0)
    pb = np.mean(np.abs(B) ** 2, axis=0)
    coh = np.abs(cross) ** 2 / np.maximum(pa * pb, 1e-20)
    edges = np.linspace(0, len(coh), nbins + 1).astype(int)
    return [round(float(np.mean(coh[edges[i]:edges[i + 1]])), 3) for i in range(nbins)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="WhisPrompt")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--json-out", default="")
    args = ap.parse_args()

    idx, rate = pick_capture(args.device)

    frames = int(args.seconds * rate)
    rec = sd.rec(frames, samplerate=rate, channels=2, dtype="float32", device=idx)
    sd.wait()
    x = np.asarray(rec, dtype=np.float64)
    l, r = x[:, 0], x[:, 1]

    lvl = lambda v: 20 * np.log10(np.sqrt(np.mean(v ** 2)) + 1e-20)
    ll, lr = lvl(l), lvl(r)
    corr = float(np.corrcoef(l, r)[0, 1])
    tdoa, peak = fractional_tdoa(l, r, rate)
    bands = coherence(l, r)

    # Silence makes the two channels trivially uncorrelated, so the level gate
    # has to run first or a dead link looks like a perfect diffuse field.
    if max(ll, lr) < -60.0:
        verdict = "invalid: no signal (Scepter offline, link down, or noise stopped)"
    elif corr < 0.80:
        verdict = "usable (independent sources)"
    else:
        verdict = "correlated: both cabinets likely share one signal"
    report = {
        "device": sd.query_devices(idx)["name"],
        "seconds": args.seconds,
        "level_l_dbfs": round(float(ll), 2),
        "level_r_dbfs": round(float(lr), 2),
        "lr_delta_db": round(float(abs(ll - lr)), 2),
        "correlation": round(corr, 4),
        "tdoa_us": round(float(tdoa), 2),
        "gcc_peak": round(float(peak), 2),
        "coherence_bands": bands,
        "verdict": verdict,
    }
    print(f"  level  L={ll:>7.2f} dBFS  R={lr:>7.2f} dBFS  delta={abs(ll - lr):.2f} dB")
    print(f"  corr   {corr:.4f}   TDOA {tdoa:>7.2f} us   peak {peak:.1f}")
    print(f"  coh     {' '.join(f'{v:.2f}' for v in bands)}")
    print(f"  verdict {verdict}")
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
