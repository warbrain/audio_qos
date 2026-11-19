"""Score both USB channels of a physical capture against the reference text.

Usage:
    python score_capture.py --dir <capture dir> [--clips <clips.json>]
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import unicodedata
import wave

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import asr_harness as AH


def norm(s: str) -> str:
    s = unicodedata.normalize("NFKC", s or "").lower()
    return "".join(ch for ch in s if unicodedata.category(ch)[0] not in ("P", "S", "Z", "C"))


def edit_distance(a: str, b: str) -> int:
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def read_wav(path: str):
    import numpy as np
    with wave.open(path, "rb") as w:
        rate, ch, n = w.getframerate(), w.getnchannels(), w.getnframes()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    return x, rate


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--clips", default="")
    ap.add_argument("--backend", default="cuda")
    args = ap.parse_args()

    refs = {}
    clips_path = args.clips or os.path.join(os.path.dirname(args.dir.rstrip("\\/")), "clips.json")
    if os.path.exists(clips_path):
        for e in json.load(open(clips_path, encoding="utf-8")):
            refs[e["id"]] = e.get("reference", "")

    caps = []
    for line in open(os.path.join(args.dir, "capture.json"), encoding="utf-8"):
        if line.strip():
            caps.append(json.loads(line))
    seen, uniq = set(), []
    for c in caps:
        if c["id"] in seen:
            continue
        seen.add(c["id"])
        uniq.append(c)

    eng = AH.AsrEngine(backend=args.backend)
    rows = []
    for c in uniq:
        ref = refs.get(c["id"], c.get("reference", ""))
        out = {"id": c["id"], "reference": ref}
        for tag in ("L", "R"):
            p = os.path.join(args.dir, f"{c['id']}_{tag}.wav")
            if not os.path.exists(p):
                out[tag] = None
                continue
            pcm, rate = read_wav(p)
            hyp = eng.transcribe(pcm, rate)
            r, h = norm(ref), norm(hyp)
            d = edit_distance(r, h) if r else (0 if not h else 1)
            out[tag] = {"hyp": hyp, "dist": d, "ref_len": len(r),
                        "cer": (d / len(r)) if r else 0.0}
        rows.append(out)
    eng.close()

    print(f"{'id':22s} {'L':>9s} {'R':>9s}   参考")
    print("-" * 100)
    for r in rows:
        l = r["L"]["cer"] * 100 if r["L"] else float("nan")
        rr = r["R"]["cer"] * 100 if r["R"] else float("nan")
        print(f"{r['id'][:22]:22s} {l:8.2f}% {rr:8.2f}%   {r['reference'][:26]}")
    print()
    for tag in ("L", "R"):
        d = sum(r[tag]["dist"] for r in rows if r[tag])
        n = sum(r[tag]["ref_len"] for r in rows if r[tag])
        perfect = sum(1 for r in rows if r[tag] and r[tag]["dist"] == 0)
        print(f"{tag}: CER {100*d/n:.2f}%  全对 {perfect}/{len(rows)}")
    print()
    for r in rows:
        for tag, nm in (("L", "L"), ("R", "R")):
            if r[tag] and r[tag]["dist"]:
                print(f"  [{nm}] {r['id'][:20]}  CER {r[tag]['cer']*100:.1f}%")
                print(f"      ref: {r['reference']}")
                print(f"      hyp: {r[tag]['hyp']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
