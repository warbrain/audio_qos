"""Summarise the T60 x SNR grid produced by run_testset.py --t60s --snrs.

Reads <out>/results.jsonl and prints:
  * a CER matrix (T60 rows x SNR columns) with per-cell clip counts
  * marginal effects (mean CER by T60, by SNR)
  * perfect-transcript counts
  * the worst cells' typical failure modes
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import statistics


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=r"D:\App\cppApp\audio_qos\_asr_sim\grid")
    args = ap.parse_args()
    path = os.path.join(args.dir, "results.jsonl")
    rows = [json.loads(l) for l in open(path, encoding="utf-8") if l.strip()]
    print(f"# {len(rows)} records from {path}\n")

    by = collections.defaultdict(list)
    for r in rows:
        by[r["condition"]].append(r)

    t60s = sorted({r["t60"] for r in rows if r["t60"] is not None})
    snrs = sorted({r["snr_db"] for r in rows if r["snr_db"] is not None}, reverse=True)

    def cell(cond):
        rs = by.get(cond, [])
        if not rs:
            return None
        d = sum(r["dist"] for r in rs)
        n = sum(r["ref_len"] for r in rs)
        perfect = sum(1 for r in rs if r["dist"] == 0)
        return {"n": len(rs), "cer": 100.0 * d / n if n else 0.0,
                "perfect": perfect, "dist": d, "ref_len": n}

    # ---- matrix -------------------------------------------------------
    head = f"{'T60':>6s} |" + "".join(f" {int(s):+4d} dB " for s in snrs) + "|  mean"
    print("=== CER 矩阵 (%) ===")
    print(head)
    print("-" * len(head))
    for t in t60s:
        cells = []
        for s in snrs:
            c = cell(f"t{int(round(t*100)):03d}_snr{int(s):+03d}")
            cells.append(f" {c['cer']:6.2f} " if c else "   --   ")
        vals = [cell(f"t{int(round(t*100)):03d}_snr{int(s):+03d}") for s in snrs]
        vals = [v["cer"] for v in vals if v]
        print(f"{t:6.2f} |" + "".join(cells) + "|" + (f" {statistics.mean(vals):6.2f}" if vals else ""))
    print()

    # ---- perfect counts ----------------------------------------------
    print("=== 全对条数 (/196) ===")
    print(head)
    print("-" * len(head))
    for t in t60s:
        cells = []
        for s in snrs:
            c = cell(f"t{int(round(t*100)):03d}_snr{int(s):+03d}")
            cells.append(f" {c['perfect']:6d} " if c else "   --   ")
        print(f"{t:6.2f} |" + "".join(cells) + "|")
    print()

    # ---- marginals ----------------------------------------------------
    clean = cell("clean")
    print(f"=== 参考基线 clean: CER {clean['cer']:.2f}%  全对 {clean['perfect']}/{clean['n']}")
    print("\n=== 边际效应 ===")
    print("按 T60 (各 SNR 平均):")
    for t in t60s:
        vals = [cell(f"t{int(round(t*100)):03d}_snr{int(s):+03d}") for s in snrs]
        vals = [v for v in vals if v]
        td = sum(v["dist"] for v in vals); tn = sum(v["ref_len"] for v in vals)
        print(f"  T60 {t:.2f} s : CER {100*td/tn:6.2f}%")
    print("按 SNR (各 T60 平均):")
    for s in snrs:
        vals = [cell(f"t{int(round(t*100)):03d}_snr{int(s):+03d}") for t in t60s]
        vals = [v for v in vals if v]
        td = sum(v["dist"] for v in vals); tn = sum(v["ref_len"] for v in vals)
        print(f"  SNR {int(s):+4d} dB : CER {100*td/tn:6.2f}%")
    print()

    # ---- failure modes ------------------------------------------------
    print("=== 最差格子里的失败形态 ===")
    worst = sorted(((c, cell(c)) for c in by if cell(c)), key=lambda kv: -kv[1]["cer"])[:3]
    for cond, c in worst:
        rs = by[cond]
        empty = [r for r in rs if not r["hypothesis"].strip()]
        greedy = [r for r in rs if r["cer"] > 1.0]
        print(f"  {cond}: CER {c['cer']:.2f}%  空输出 {len(empty)} 条  CER>100% {len(greedy)} 条")
        for r in sorted(rs, key=lambda x: -x["cer"])[:2]:
            print(f"     {r['cer']*100:6.1f}%  ref={r['reference'][:16]}")
            print(f"              hyp={r['hypothesis'][:26]!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
