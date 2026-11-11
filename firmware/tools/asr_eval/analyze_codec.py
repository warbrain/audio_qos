"""Paired codec comparison against the uncompressed baseline.

Reads results_codec/per_utterance.csv and reports, per codec, the CER and the
paired delta (codec - PCM) with a bootstrap confidence interval.
"""
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

import numpy as np

CSV = Path(__file__).with_name("results_codec") / "per_utterance.csv"
ITERS = 4000
SEED = 13
LABELS = ["m0_g1_cpcm", "m0_g1_cg722", "m0_g1_co32", "m0_g1_co62"]
NAMES = {
    "m0_g1_cpcm": "PCM (no codec)",
    "m0_g1_cg722": "G.722 64k",
    "m0_g1_co32": "Opus 32k",
    "m0_g1_co62": "Opus 62k",
}


def main() -> int:
    rows = list(csv.DictReader(CSV.open(encoding="utf-8-sig")))
    by: dict[str, dict[str, float]] = defaultdict(dict)
    for r in rows:
        by[r["condition"]][r["id"]] = float(r["cer"])
    base = by[LABELS[0]]
    ids = sorted(base)
    base_vec = np.array([base[i] for i in ids])
    rng = np.random.default_rng(SEED)

    print(f"{'codec':>16} {'n':>3} {'CER%':>7} {'delta':>8} {'95% CI':>18} {'P(better)':>10}")
    for label in LABELS:
        vec = np.array([by[label][i] for i in ids])
        d = vec - base_vec
        boots = np.array([d[rng.integers(0, len(d), len(d))].mean() for _ in range(ITERS)])
        ci = (np.percentile(boots, 2.5) * 100, np.percentile(boots, 97.5) * 100)
        p = float(np.mean(boots < 0) + 0.5 * np.mean(boots == 0))
        delta = "—" if label == LABELS[0] else f"{d.mean()*100:+.2f}"
        ci_text = "—" if label == LABELS[0] else f"[{ci[0]:+.2f},{ci[1]:+.2f}]"
        p_text = "—" if label == LABELS[0] else f"{p:.3f}"
        print(f"{NAMES[label]:>16} {len(ids):>3} {vec.mean()*100:>7.2f} {delta:>8} "
              f"{ci_text:>18} {p_text:>10}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
