#!/usr/bin/env python3
"""Play a noise bed through the two box speakers for live HMI testing.

The artificial mouth stays silent: the operator speaks into it (or close to the
device) while HMI records, so the recognition result includes the real acoustic
path, the firmware microphone chain and the real noise field.

Routing (US-4x4HR, 4 channels @ 48 kHz):
    ch2 -> left noise speaker
    ch3 -> right noise speaker

Examples:
    python noise_play.py --level 0.05 --seconds 30
    python noise_play.py --level 0.15 --seconds 30 --coherent
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import sounddevice as sd

SR = 48000
NOISE_L_CH, NOISE_R_CH = 2, 3


def find_output(name_part: str = "US-4x4HR") -> int:
    for index, info in enumerate(sd.query_devices()):
        if name_part.lower() in info["name"].lower() and info["max_output_channels"] >= 4:
            return index
    raise SystemExit(f"output device containing '{name_part}' not found")


def pink(n: int, rng: np.random.Generator) -> np.ndarray:
    white = rng.standard_normal(n)
    spectrum = np.fft.rfft(white)
    freqs = np.fft.rfftfreq(n, 1.0 / SR)
    spectrum[1:] /= np.sqrt(np.maximum(freqs[1:], 1.0))
    spectrum[0] = 0.0
    out = np.fft.irfft(spectrum, n)
    return (out / (np.max(np.abs(out)) + 1e-9)).astype(np.float32)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--level", type=float, default=0.05, help="noise amplitude (0..1)")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--coherent", action="store_true")
    parser.add_argument("--device", type=int, default=-1)
    args = parser.parse_args()

    device = args.device if args.device >= 0 else find_output()
    rng = np.random.default_rng(7)
    total = int(args.seconds * SR)
    left = pink(total, rng) * args.level
    right = left.copy() if args.coherent else pink(total, rng) * args.level
    buffer = np.zeros((total, 4), dtype=np.float32)
    buffer[:, NOISE_L_CH] = left
    buffer[:, NOISE_R_CH] = right

    print(f"device {device}: {sd.query_devices(device)['name']}")
    print(f"playing {args.seconds:g}s noise level={args.level:g} "
          f"({'coherent' if args.coherent else 'independent'}) - Ctrl+C to stop")
    try:
        sd.play(buffer, samplerate=SR, device=device, blocking=True)
    except KeyboardInterrupt:
        sd.stop()
    print("noise stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
