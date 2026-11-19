"""对已渲染的目录做 ASR 评分。

用法：
  python score_dir.py --dir _asr_sim/sim_snr0_side
  python score_dir.py --dir _asr_sim/sim_snr0_side --channel R
  python score_dir.py --dir a b c            # 多个目录一起对比

输出：目录内 asr.jsonl（逐条）+ score.json（汇总），并在屏幕打印表格。
退化样本（单条 CER>300%，通常是 ASR 循环输出）自动剔除并单独标注。
"""
from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
import score as SC
import simcore as S


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", nargs="+", required=True)
    ap.add_argument("--channel", default="L", choices=["L", "R"])
    ap.add_argument("--clips", type=int, default=24, help="用清单前 N 条做参考文本")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--backend", default="vulkan")
    args = ap.parse_args()

    items = S.load_manifest(args.clips, args.start)
    refs = {it["id"]: it["reference"] for it in items}
    eng = SC.open_asr(args.backend)
    results = []
    try:
        for d in args.dir:
            res = SC.score_dir(d, refs, eng, channel=args.channel)
            SC.write_report(d, res)
            results.append(res)
    finally:
        eng.close()

    print(f"\n{'目录':<42s} {'条数':>4s} {'CER':>8s} {'稳健CER':>9s} {'全对':>7s}  退化")
    print("-" * 84)
    for r in results:
        deg = ",".join(x[:14] for x in r["degenerate"]) or "-"
        print(f"{os.path.basename(r['dir']):<42s} {r['clips']:4d} "
              f"{r['cer']:7.2f}% {r['cer_robust']:8.2f}% {r['perfect']:3d}/{r['clips']:<3d}  {deg}")

    with open(os.path.join(args.dir[0], "..", "score_summary.json"), "w", encoding="utf-8") as f:
        json.dump([{k: v for k, v in r.items() if k != "rows"} for r in results], f,
                  ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
