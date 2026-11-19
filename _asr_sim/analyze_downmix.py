"""Measure what the HMI's (L+R)/2 downmix does to recognition.

For each captured pair it:
  1. measures the L/R cross-correlation lag (a delay means comb filtering),
  2. builds the exact HMI downmix (i16 arithmetic mean),
  3. builds a level-aligned downmix for comparison,
  4. transcribes L, R, plain mean and aligned mean, and reports CER.

Usage: python analyze_downmix.py --dir <capture dir> --clips <clips.json>
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import unicodedata
import wave

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import asr_harness as AH


def read_i16(path: str) -> np.ndarray:
    with wave.open(path, "rb") as w:
        n, ch = w.getnframes(), w.getnchannels()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2")
    return x.reshape(-1, ch)[:, 0] if ch > 1 else x


def norm(s: str) -> str:
    s = unicodedata.normalize("NFKC", s or "").lower()
    return "".join(c for c in s if unicodedata.category(c)[0] not in ("P", "S", "Z", "C"))


def cer(ref: str, hyp: str) -> float:
    r, h = norm(ref), norm(hyp)
    if not r:
        return 0.0 if not h else 1.0
    prev = list(range(len(h) + 1))
    for i, ca in enumerate(r, 1):
        cur = [i]
        for j, cb in enumerate(h, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1] / len(r)


def to_f32(x: np.ndarray) -> np.ndarray:
    return x.astype(np.float32) / 32768.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--clips", required=True)
    args = ap.parse_args()

    refs = {e["id"]: e.get("reference", "")
            for e in json.load(open(args.clips, encoding="utf-8"))}
    ids = [k for k in refs if os.path.exists(os.path.join(args.dir, f"{k}_L.wav"))]
    if not ids:
        print(f"no captured pairs in {args.dir}")
        return 1

    eng = AH.AsrEngine()
    rows = []
    for cid in ids:
        L = read_i16(os.path.join(args.dir, f"{cid}_L.wav"))
        R = read_i16(os.path.join(args.dir, f"{cid}_R.wav"))
        n = min(len(L), len(R))
        L, R = L[:n].astype(np.int32), R[:n].astype(np.int32)

        # 1) cross-correlation lag
        a = to_f32(L.astype(np.int16)).astype(np.float64)
        b = to_f32(R.astype(np.int16)).astype(np.float64)
        seg = min(16000 * 3, len(a))
        c = np.correlate(a[:seg] - a[:seg].mean(), b[:seg] - b[:seg].mean(), mode="full")
        lag = int(np.argmax(c) - (seg - 1))
        denom = (np.linalg.norm(a[:seg]) * np.linalg.norm(b[:seg])) + 1e-12
        xc = float(c.max() / denom)

        # 2) HMI downmix: i16 arithmetic mean
        mean = ((L + R) // 2).astype(np.int16)
        # 3) level-aligned downmix: match L to R RMS first
        r_rms = float(np.sqrt(np.mean(R.astype(np.float64) ** 2))) + 1e-9
        l_rms = float(np.sqrt(np.mean(L.astype(np.float64) ** 2))) + 1e-9
        g = r_rms / l_rms
        aligned = np.clip((L * g + R) / 2.0, -32768, 32767).astype(np.int16)

        ref = refs[cid]
        out = {"id": cid}
        for tag, sig in (("L", L.astype(np.int16)), ("R", R.astype(np.int16)),
                         ("mean", mean), ("aligned", aligned)):
            out[tag] = cer(ref, eng.transcribe(to_f32(sig), 16000))
        rows.append((out, lag, xc, g))
    eng.close()

    print(f"{'id':22s} {'lag':>6s} {'xcorr':>6s} {'L':>8s} {'R':>8s} {'mean':>8s} {'aligned':>8s}")
    print("-" * 78)
    for o, lag, xc, g in rows:
        print(f"{o['id'][:22]:22s} {lag:6d} {xc:6.2f} {o['L']*100:7.2f}% {o['R']*100:7.2f}% "
              f"{o['mean']*100:7.2f}% {o['aligned']*100:7.2f}%")
    print()
    for tag in ("L", "R", "mean", "aligned"):
        print(f"{tag:8s} 平均 CER {100*np.mean([o[tag] for o, *_ in rows]):.2f}%  "
              f"全对 {sum(1 for o, *_ in rows if o[tag] == 0)}/{len(rows)}")
    lags = [abs(l) for _, l, _, _ in rows]
    print(f"\nL/R 延迟: 绝对值 中位 {np.median(lags):.0f} 样本, 最大 {max(lags)} 样本 "
          f"({max(lags)/16:.2f} ms @16k)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
