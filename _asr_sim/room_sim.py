"""Box-rig simulator: RT60 calibration, fractional-delay TDOA, distance sweep.

Answers three questions with one tool:
  * RT60 sweep  - which reverberation matches the measured noise correlation
                  (0.984) of the real cabinet, so the simulation is calibrated;
  * fractional TDOA - the 20 mm array spans less than one 16 kHz sample, so an
                  integer-lag peak cannot resolve direction; interpolate instead;
  * distance sweep - how the mouth-to-array distance moves SNR and the ideal
                  spatial ceiling, so a distance can be chosen on paper.
"""

from __future__ import annotations

import argparse
import json
import os

import numpy as np
import pyroomacoustics as pra

FS = 16000
MIC_SPACING_M = 0.020


def build_room(rt60: float, dims: tuple[float, float, float]) -> pra.ShoeBox:
    # inverse_sabine rejects very low RT60 in a small box because the required
    # absorption passes 1.0; clamp it and keep a shallow reflection order.
    try:
        e_absorption, max_order = pra.inverse_sabine(rt60, dims)
        if not (0.0 < e_absorption < 0.999):
            raise ValueError
    except ValueError:
        e_absorption, max_order = 0.97, 4
    return pra.ShoeBox(dims, fs=FS, materials=pra.Material(e_absorption), max_order=max_order)


def render(args: argparse.Namespace, rt60: float, distance: float,
           with_mouth: bool) -> tuple[np.ndarray, np.ndarray]:
    """Return (mic signals, mouth signal) for one geometry."""
    dims = (args.length, args.width, args.height)
    room = build_room(rt60, dims)
    center = np.array([args.length / 2.0, args.width / 2.0, args.mic_height])
    mics = np.array([
        [center[0] - MIC_SPACING_M / 2.0, center[1], center[2]],
        [center[0] + MIC_SPACING_M / 2.0, center[1], center[2]],
    ]).T
    room.add_microphone_array(pra.MicrophoneArray(mics, room.fs))

    n = int(3.0 * FS)
    speech = load_speech(args, int(2.0 * FS))
    mouth_pos = center + np.array([0.0, distance, 0.0])
    if with_mouth:
        pad = np.zeros(n)
        pad[: len(speech)] = speech
        room.add_source(mouth_pos, signal=pad)

    rng = np.random.default_rng(args.seed)
    shared = rng.standard_normal(n) * 0.05
    for pos in speaker_positions(args):
        # --noise-correlated feeds both cabinets the same signal, which is what
        # the bench looks like: a symmetric coherent field gives the 0.984
        # inter-channel correlation that independent noise cannot reach.
        sig = shared if args.noise_correlated else rng.standard_normal(n) * 0.05
        room.add_source(pos, signal=sig)

    room.simulate()
    sig = room.mic_array.signals
    if sig.shape[1] < n:
        sig = np.pad(sig, ((0, 0), (0, n - sig.shape[1])))
    return sig, speech


def speaker_positions(args: argparse.Namespace) -> list[np.ndarray]:
    """Noise sources. The bench photo shows both cabinets on the floor rails,
    left and right of the stand, roughly 0.4 m out and facing inwards, so the
    default layout is lateral rather than the earlier corner pair."""
    cx, cy = args.length / 2.0, args.width / 2.0
    z = args.speaker_height
    off = args.speaker_offset
    if args.speaker_layout == "lateral":
        return [np.array([cx - off, cy, z]), np.array([cx + off, cy, z])]
    return [np.array([cx - off, cy - off, z]), np.array([cx + off, cy + off, z])]


def load_speech(args: argparse.Namespace, n: int) -> np.ndarray:
    import wave

    with wave.open(args.speech, "rb") as fh:
        rate, ch = fh.getframerate(), fh.getnchannels()
        data = np.frombuffer(fh.readframes(fh.getnframes()), dtype=np.int16)
    data = data.reshape(-1, ch).mean(axis=1).astype(np.float64) / 32768.0
    if rate != FS:
        idx = np.linspace(0, len(data) - 1, int(len(data) * FS / rate))
        data = np.interp(idx, np.arange(len(data)), data)
    if len(data) < n:
        data = np.tile(data, int(np.ceil(n / len(data))))
    return data[:n]


def gcc_phat_fractional(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """Fractional-delay TDOA in microseconds plus peak sharpness.

    The 20 mm array spans 0.93 samples at 16 kHz, so the integer argmax alone
    always lands on lag 0. Parabolic interpolation recovers the sub-sample peak.
    """
    seg = min(len(a), FS)
    a0 = a[:seg] - a[:seg].mean()
    b0 = b[:seg] - b[:seg].mean()
    n = 1 << int(np.ceil(np.log2(2 * seg)))
    A, B = np.fft.rfft(a0, n), np.fft.rfft(b0, n)
    R = A * np.conj(B)
    R /= np.maximum(np.abs(R), 1e-12)
    cc = np.fft.irfft(R, n)
    k = int(np.argmax(cc[: seg // 2]))
    y0, y1, y2 = cc[max(k - 1, 0)], cc[k], cc[min(k + 1, n - 1)]
    denom = y0 - 2 * y1 + y2
    delta = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-20 else 0.0
    peak = float(y1 / max(np.std(cc), 1e-20))
    return (k + float(np.clip(delta, -0.5, 0.5))) * 1e6 / FS, peak


def oracle_stats(mix: np.ndarray, noise: np.ndarray, speech: np.ndarray) -> tuple[float, float, float]:
    """Ideal binary mask ceiling, expressed as noise suppression.

    The old helper reported the fraction of speech an oracle keeps, which is
    ~1.0 for any sensible mask and therefore says nothing about separation.
    This reports the pair that matters: how much of the speech survives and how
    much of the noise is removed, both at a 0 dB per-bin threshold.
    """
    win, hop = 512, 128

    def spec(x: np.ndarray) -> np.ndarray:
        frames = np.lib.stride_tricks.sliding_window_view(x, win)[::hop] * np.hanning(win)
        return np.abs(np.fft.rfft(frames, axis=1)) ** 2

    sp = np.zeros_like(mix)
    sp[: min(len(speech), len(mix))] = speech[: min(len(speech), len(mix))]
    S, N = spec(sp), spec(noise)
    mask = S > N
    speech_keep = float((S * mask).sum() / max(S.sum(), 1e-20))
    noise_keep = float((N * mask).sum() / max(N.sum(), 1e-20))
    noise_supp = float(-10.0 * np.log10(max(noise_keep, 1e-12)))
    return speech_keep, noise_keep, noise_supp

def measure(args: argparse.Namespace, rt60: float, distance: float) -> dict:
    mix, speech = render(args, rt60, distance, True)
    noise, _ = render(args, rt60, distance, False)
    sp_only, _ = render(args, rt60, distance, True)
    # Speech-only render still contains the noise sources; subtract to isolate it.
    sp = sp_only - noise
    lvl = lambda x: 20 * np.log10(np.sqrt(np.mean(x ** 2)) + 1e-20)
    snr_l = lvl(sp[0]) - lvl(noise[0])
    snr_r = lvl(sp[1]) - lvl(noise[1])
    corr = float(np.corrcoef(noise[0], noise[1])[0, 1])
    t, peak = gcc_phat_fractional(noise[0], noise[1])
    speech_keep, noise_keep, noise_supp = oracle_stats(mix[0], noise[0], sp[0])
    return {
        "rt60": round(rt60, 3),
        "distance_cm": round(distance * 100, 1),
        "snr_db": round(float((snr_l + snr_r) / 2), 2),
        "snr_delta_db": round(float(abs(snr_l - snr_r)), 2),
        "noise_corr": round(corr, 4),
        "tdoa_us": round(float(t), 2),
        "gcc_peak": round(peak, 3),
        "oracle_speech_keep": round(speech_keep, 3),
        "oracle_noise_supp_db": round(noise_supp, 2),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, default=1.2)
    ap.add_argument("--width", type=float, default=1.0)
    ap.add_argument("--height", type=float, default=0.8)
    ap.add_argument("--mic-height", type=float, default=0.40)
    ap.add_argument("--speaker-height", type=float, default=0.20)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--speech", required=True)
    ap.add_argument("--mode", choices=["rt60", "distance"], default="rt60")
    ap.add_argument("--rt60-sweep", default="0.02,0.05,0.08,0.12,0.20")
    ap.add_argument("--distance-sweep", default="0.02,0.05,0.10,0.20")
    ap.add_argument("--fixed-rt60", type=float, default=0.05)
    ap.add_argument("--speaker-layout", choices=["lateral", "diagonal"], default="lateral")
    ap.add_argument("--speaker-offset", type=float, default=0.40)
    ap.add_argument("--noise-correlated", action="store_true")
    ap.add_argument("--json-out", default="")
    args = ap.parse_args()
    if not os.path.exists(args.speech):
        raise SystemExit("--speech must be a readable 16 kHz wav")

    rows: list[dict] = []
    if args.mode == "rt60":
        for v in [float(x) for x in args.rt60_sweep.split(",") if x.strip()]:
            rows.append(measure(args, v, 0.02))
            r = rows[-1]
            print(f"  RT60={r['rt60']:.3f}s | noise corr={r['noise_corr']:.4f} | "
                  f"TDOA={r['tdoa_us']:>6.2f} us | peak={r['gcc_peak']:.2f} | SNR={r['snr_db']:.1f} dB")
    else:
        for v in [float(x) for x in args.distance_sweep.split(",") if x.strip()]:
            rows.append(measure(args, args.fixed_rt60, v))
            r = rows[-1]
            print(f"  dist={r['distance_cm']:>5.1f} cm | SNR={r['snr_db']:>6.2f} dB | "
                  f"corr={r['noise_corr']:.3f} | TDOA={r['tdoa_us']:>6.2f} us | keep={r['oracle_speech_keep']:.2f} supp={r['oracle_noise_supp_db']:.1f} dB")

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(rows, fh, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
