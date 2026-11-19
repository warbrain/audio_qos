"""配对比较两个采集目录（同一批语料、不同条件）。

用法：
    python compare_dirs.py --a ../spatial_dir/m0_5cm --b ../spatial_dir/m5_5cm
    python compare_dirs.py --a ../snr_threshold/snr+0_m0 --b ../spatial_dir/m0_5cm --label "10cm vs 5cm (bypass)"
"""
from __future__ import annotations

import argparse
import json
import os
from math import comb

import numpy as np


def load(d: str) -> dict:
    path = os.path.join(d, "asr.jsonl")
    return {r["id"]: r for r in (json.loads(l) for l in open(path, encoding="utf-8") if l.strip())}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", required=True)
    ap.add_argument("--b", required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--limit", type=int, default=0, help="只比较前 N 条（0=全部）")
    args = ap.parse_args()

    a, b = load(args.a), load(args.b)
    ids = sorted(set(a) & set(b))
    if args.limit:
        ids = ids[: args.limit]
    if not ids:
        print("  没有共同样本")
        return 1

    def pooled(d: dict, sub: list[str]) -> float:
        dist = sum(d[i]["dist"] for i in sub)
        total = sum(d[i]["ref_len"] for i in sub)
        return 100.0 * dist / total

    ca, cb = pooled(a, ids), pooled(b, ids)
    wins = sum(1 for i in ids if b[i]["dist"] < a[i]["dist"])
    losses = sum(1 for i in ids if b[i]["dist"] > a[i]["dist"])

    rng = np.random.default_rng(0)
    diffs = []
    for _ in range(5000):
        idx = rng.integers(0, len(ids), len(ids))
        sub = [ids[k] for k in idx]
        d1 = sum(b[i]["dist"] for i in sub)
        d0 = sum(a[i]["dist"] for i in sub)
        t = sum(a[i]["ref_len"] for i in sub)
        diffs.append(100.0 * (d1 - d0) / t)
    lo, hi = np.percentile(diffs, [2.5, 97.5])

    n = wins + losses
    k = min(wins, losses)
    p_sign = 2 * sum(comb(n, i) for i in range(0, k + 1)) / (2 ** n) if n else 1.0

    label = args.label or f"{os.path.basename(args.a)} -> {os.path.basename(args.b)}"
    print(f"  {label}")
    print(f"    样本 {len(ids)} 条")
    print(f"    A {ca:6.2f}%   全对 {sum(1 for i in ids if a[i]['dist'] == 0)}")
    print(f"    B {cb:6.2f}%   全对 {sum(1 for i in ids if b[i]['dist'] == 0)}")
    print(f"    Δ {cb - ca:+.2f} pp   Bootstrap 95%CI [{lo:+.2f}, {hi:+.2f}]   "
          f"{'✅ 显著' if (lo > 0 or hi < 0) else '不显著'}")
    print(f"    配对: {wins} 胜 / {losses} 负 / {len(ids) - wins - losses} 平   "
          f"符号检验 p={p_sign:.5f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
