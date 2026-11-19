"""配对比较某个 SNR 条件下 直通(0) vs 双路 Wiener(5) 的差异。

默认读取 SNR 0 dB 的 196 条采集；可用 --dir-tag 指定其它条件目录前缀。
数据来自 snr_threshold_sweep.py 的输出目录。
"""
from __future__ import annotations

import argparse
import json
import os
from math import comb

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.join(ROOT, "snr_threshold")


def load(mode: int, tag: str, suffix: str = "") -> dict:
    path = os.path.join(BASE, f"{tag}_m{mode}{suffix}", "asr.jsonl")
    return {r["id"]: r for r in (json.loads(l) for l in open(path, encoding="utf-8") if l.strip())}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir-tag", default="snr+0", help="输出目录前缀，如 snr+0 / snr+5")
    ap.add_argument("--suffix", default="", help="目录后缀，如 _cal2")
    args = ap.parse_args()

    d = {m: load(m, args.dir_tag, args.suffix) for m in (0, 5)}
    ids = sorted(set(d[0]) & set(d[5]))

    def pooled(mode: int, sub: list[str]) -> float:
        dist = sum(d[mode][i]["dist"] for i in sub)
        total = sum(d[mode][i]["ref_len"] for i in sub)
        return 100.0 * dist / total

    c0, c5 = pooled(0, ids), pooled(5, ids)
    perfect0 = sum(1 for i in ids if d[0][i]["dist"] == 0)
    perfect5 = sum(1 for i in ids if d[5][i]["dist"] == 0)
    wins = sum(1 for i in ids if d[5][i]["dist"] < d[0][i]["dist"])
    losses = sum(1 for i in ids if d[5][i]["dist"] > d[0][i]["dist"])

    rng = np.random.default_rng(0)
    diffs = []
    for _ in range(5000):
        idx = rng.integers(0, len(ids), len(ids))
        sub = [ids[k] for k in idx]
        d1 = sum(d[5][i]["dist"] for i in sub)
        d0 = sum(d[0][i]["dist"] for i in sub)
        t = sum(d[0][i]["ref_len"] for i in sub)
        diffs.append(100.0 * (d1 - d0) / t)
    lo, hi = np.percentile(diffs, [2.5, 97.5])

    n = wins + losses
    k = min(wins, losses)
    p_sign = 2 * sum(comb(n, i) for i in range(0, k + 1)) / (2 ** n) if n else 1.0

    print(f"  条件 {args.dir_tag}{args.suffix}   样本 {len(ids)} 条")
    print(f"  直通        {c0:6.2f}%   全对 {perfect0}")
    print(f"  双路Wiener  {c5:6.2f}%   全对 {perfect5}")
    print(f"  Δ {c5 - c0:+.2f} pp   Bootstrap 95%CI [{lo:+.2f}, {hi:+.2f}]   "
          f"{'✅ 显著' if (lo > 0 or hi < 0) else '不显著'}")
    print(f"  配对: {wins} 胜 / {losses} 负 / {len(ids) - wins - losses} 平   "
          f"符号检验 p={p_sign:.5f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
