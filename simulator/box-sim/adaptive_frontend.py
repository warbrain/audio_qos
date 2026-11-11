#!/usr/bin/env python3
r"""Adaptive front-end that tunes the ASR input to the current device state.

The controller is deliberately small and interpretable: it estimates the live
noise floor, speech level, SNR and inter-channel correlation from the device's
stereo capture, then decides

    * a per-utterance gain that lands speech near a target level while keeping
      peak headroom below the firmware soft limiter (20000/32768 = 0.61 FS),
    * whether channel fusion can help (low correlation) or is pointless,
    * whether the current state is too noisy for the ASR to be reliable.

It adapts across utterances with an exponential moving average of the noise
floor and a closed-loop correction of the measured output level, so the same
code tracks a quiet box, a noisy room or a hot microphone without retuning.

Usage:
    python adaptive_frontend.py --wav D:\path\file.wav [--state]
    python adaptive_frontend.py --dir .\scenes
"""
from __future__ import annotations

import argparse
import glob
import os

import numpy as np
import soundfile as sf

TARGET_SPEECH_DBFS = -22.0      # comfortable ASR level with limiter headroom
LIMITER_KNEE = 20000.0 / 32768.0
CLIP_GUARD_DB = 6.0             # keep peaks this far below the knee
GAIN_MIN_DB, GAIN_MAX_DB = -6.0, 24.0
EMA_ALPHA = 0.3
FRAME_S = 0.02


def features(signal: np.ndarray, fs: int) -> dict:
    """Estimate level, noise floor, SNR and channel correlation."""
    frames = max(int(FRAME_S * fs), 1)
    count = len(signal) // frames
    block = signal[: count * frames].reshape(count, frames, -1)
    rms = np.sqrt(np.mean(block.astype(np.float64) ** 2, axis=(1, 2)))
    active = rms[rms > 0]
    if active.size == 0:
        return {"speech": 0.0, "noise": 0.0, "snr": 0.0, "corr": 0.0, "peak": 0.0}
    speech = float(np.percentile(active, 90))
    noise = float(np.percentile(active, 10))
    corr = 0.0
    if signal.ndim == 2 and signal.shape[1] > 1:
        left, right = signal[:, 0], signal[:, 1]
        if left.std() > 1e-9 and right.std() > 1e-9:
            corr = float(np.corrcoef(left, right)[0, 1])
    return {
        "speech": speech,
        "noise": noise,
        "snr": 20.0 * np.log10(max(speech, 1e-9) / max(noise, 1e-9)),
        "corr": corr,
        "peak": float(np.max(np.abs(signal))),
    }


class AdaptiveFrontEnd:
    """Stateful controller; one instance per device session."""

    def __init__(self) -> None:
        self.noise_ema: float | None = None
        self.level_correction_db = 0.0
        self.utterances = 0

    def decide(self, feat: dict) -> dict:
        speech_db = 20.0 * np.log10(max(feat["speech"], 1e-9))
        noise_db = 20.0 * np.log10(max(feat["noise"], 1e-9))
        self.noise_ema = noise_db if self.noise_ema is None else (
            (1.0 - EMA_ALPHA) * self.noise_ema + EMA_ALPHA * noise_db)
        gain_db = TARGET_SPEECH_DBFS - speech_db + self.level_correction_db
        gain_db = float(np.clip(gain_db, GAIN_MIN_DB, GAIN_MAX_DB))

        peak_db = 20.0 * np.log10(max(feat["peak"], 1e-9)) + gain_db
        headroom_db = 20.0 * np.log10(LIMITER_KNEE) - peak_db
        if headroom_db < CLIP_GUARD_DB:
            gain_db -= CLIP_GUARD_DB - headroom_db

        fusion = "mean" if feat["corr"] < 0.95 else "keep-stereo"
        noisy = feat["snr"] < 8.0

        # Closed loop: remember how far this utterance landed from the target.
        achieved = speech_db + gain_db
        self.level_correction_db = float(np.clip(
            self.level_correction_db + 0.5 * (TARGET_SPEECH_DBFS - achieved),
            -6.0, 6.0))
        self.utterances += 1
        return {
            "gain_db": round(gain_db, 1),
            "fusion": fusion,
            "noisy": noisy,
            "headroom_db": round(headroom_db, 1),
            "noise_ema_db": round(self.noise_ema, 1),
            "correction_db": round(self.level_correction_db, 1),
        }


def run_file(path: str, controller: AdaptiveFrontEnd) -> None:
    signal, fs = sf.read(path, always_2d=True)
    feat = features(signal, fs)
    decision = controller.decide(feat)
    print(f"{os.path.basename(path):<40} speech={20 * np.log10(max(feat['speech'], 1e-9)):6.1f} dBFS "
          f"snr={feat['snr']:5.1f} dB corr={feat['corr']:.3f} "
          f"-> gain={decision['gain_db']:+5.1f} dB fusion={decision['fusion']:<11} "
          f"{'NOISY' if decision['noisy'] else ''}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wav")
    parser.add_argument("--dir")
    args = parser.parse_args()
    controller = AdaptiveFrontEnd()
    if args.wav:
        run_file(args.wav, controller)
        return 0
    if not args.dir:
        raise SystemExit("--wav or --dir is required")
    for path in sorted(glob.glob(os.path.join(args.dir, "**", "*.wav"), recursive=True)):
        run_file(path, controller)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
