"""Measure how ASR accuracy depends on the input level, to judge whether the
HMI's AGC target of -23 dBFS RMS sits on the best plateau.

Each 8x-gain capture is rescaled so its speech-region RMS hits a fixed target,
then transcribed and scored against the verified reference text.

Usage: python run_level_sweep.py [--clips 24] [--targets -32,-26,-23,-20,-14]
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import unicodedata
import wave

import numpy as np

ROOT = r"D:\App\cppApp\audio_qos\_asr_sim"
SRC = os.path.join(ROOT, "gain_sweep", "g8")
OUT = os.path.join(ROOT, "level_sweep")
RATE = 16000
SPEECH_START_S = 1.5
SPEECH_END_S = 3.0

sys.path.insert(0, ROOT)
import asr_harness as AH  # noqa: E402


def norm(s: str) -> str:
    s = unicodedata.normalize("NFKC", s or "").lower()
    return "".join(c for c in s if unicodedata.category(c)[0] not in ("P", "S", "Z", "C"))


def edit_distance(a: str, b: str) -> int:
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def load(path: str) -> np.ndarray:
    with wave.open(path, "rb") as w:
        n, ch = w.getnframes(), w.getnchannels()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch)[:, 0]
    return x


def speech_rms(x: np.ndarray) -> float:
    a, b = int(SPEECH_START_S * RATE), int(SPEECH_END_S * RATE)
    seg = x[a:min(b, len(x))]
    return float(np.sqrt(np.mean(seg ** 2))) if len(seg) else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--targets", default="-32,-26,-23,-20,-14")
    ap.add_argument("--backend", default="vulkan")
    args = ap.parse_args()
    targets = [float(t) for t in args.targets.split(",")]

    rows = [json.loads(l) for l in open(os.path.join(SRC, "asr.jsonl"), encoding="utf-8") if l.strip()]
    rows = rows[: args.clips]
    os.makedirs(OUT, exist_ok=True)

    eng = AH.AsrEngine(backend=args.backend)
    results = []
    for target in targets:
        dists, refs_len, perfect, peaks = 0, 0, 0, []
        for r in rows:
            p = os.path.join(SRC, f"{r['id']}_L.wav")
            if not os.path.exists(p):
                continue
            x = load(p)
            rms = speech_rms(x)
            if rms < 1e-7:
                continue
            gain = (10 ** (target / 20.0)) / rms
            y = x * gain
            pk = float(np.max(np.abs(y)))
            if pk > 0.999:                     # hard clip like a limiter would
                y = np.clip(y, -0.999, 0.999)
            peaks.append(20 * np.log10(pk + 1e-12))
            hyp = eng.transcribe(y.astype(np.float32), RATE)
            rr, hh = norm(r["reference"]), norm(hyp)
            d = edit_distance(rr, hh) if rr else 0
            dists += d
            refs_len += len(rr)
            perfect += 1 if d == 0 else 0
        entry = {
            "target_rms_dbfs": target,
            "clips": len(peaks),
            "cer": round(100 * dists / refs_len, 3) if refs_len else None,
            "perfect": perfect,
            "peak_median_dbfs": round(statistics.median(peaks), 2) if peaks else None,
            "peak_max_dbfs": round(max(peaks), 2) if peaks else None,
            "clipped": sum(1 for p in peaks if p > -0.01),
        }
        results.append(entry)
        print(f"  target {target:5.1f} dBFS -> CER {entry['cer']:6.2f}%  perfect {perfect}/{entry['clips']}  "
              f"peak med/max {entry['peak_median_dbfs']}/{entry['peak_max_dbfs']}  clipped {entry['clipped']}",
              flush=True)

    eng.close()
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"source": SRC, "clips": args.clips, "results": results}, f, ensure_ascii=False, indent=2)
    print("\n=== 电平 - CER 关系 ===")
    print(f"{'目标RMS':>9s} {'条数':>5s} {'CER':>8s} {'全对':>8s} {'峰值最大':>9s}")
    for e in results:
        print(f"{e['target_rms_dbfs']:8.1f} {e['clips']:5d} {e['cer']:7.2f}% "
              f"{e['perfect']:4d}/{e['clips']:<3d} {e['peak_max_dbfs']:8.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
