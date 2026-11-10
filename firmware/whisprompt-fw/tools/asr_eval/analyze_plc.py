"""Compare the two concealment strategies at each simulated drop rate.

Reads results_plc/per_utterance.csv and reports, per drop rate:
  CER(zero), CER(repeat), and the paired delta (repeat - zero) with a
  bootstrap confidence interval. Negative delta means repeat is better.
"""
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

import numpy as np

CSV = Path(__file__).with_name("results_plc") / "per_utterance.csv"
ITERS = 4000
SEED = 11


def load() -> dict:
    rows = list(csv.DictReader(CSV.open(encoding="utf-8-sig")))
    by: dict[tuple[float, str], dict[str, float]] = defaultdict(dict)
    for r in rows:
        by[(float(r["drop"]), r["conceal"])][r["id"]] = float(r["cer"])
    return by


def main() -> int:
    by = load()
    rates = sorted({k[0] for k in by})
    rng = np.random.default_rng(SEED)
    print(f"{'drop':>6} {'n':>3} {'zero%':>7} {'repeat%':>8} {'delta':>8} "
          f"{'95% CI':>18} {'P(repeat better)':>18}")
    for rate in rates:
        if rate == 0.0:
            key = (0.0, "repeat")
            ids = sorted(by[key])
            cer = np.array([by[key][i] for i in ids])
            print(f"{rate:>6.2f} {len(ids):>3} {cer.mean()*100:>7.2f} {'-':>8} "
                  f"{'-':>8} {'-':>18} {'baseline':>18}")
            continue
        z, r = by[(rate, "zero")], by[(rate, "repeat")]
        ids = sorted(set(z) & set(r))
        zv = np.array([z[i] for i in ids])
        rv = np.array([r[i] for i in ids])
        d = rv - zv
        boots = np.array([d[rng.integers(0, len(d), len(d))].mean() for _ in range(ITERS)])
        ci = (np.percentile(boots, 2.5) * 100, np.percentile(boots, 97.5) * 100)
        p = float(np.mean(boots < 0) + 0.5 * np.mean(boots == 0))
        print(f"{rate:>6.2f} {len(ids):>3} {zv.mean()*100:>7.2f} {rv.mean()*100:>8.2f} "
              f"{d.mean()*100:>+8.2f} [{ci[0]:+.2f},{ci[1]:+.2f}] {p:>18.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
