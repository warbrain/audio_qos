"""ASR 解码参数实验：在既有采集上比较 language / max_tokens / chunk 的影响。

只读已采集的 wav，不需要重采。重点验证两件事：
  1. `max_tokens` 能否压掉"ASR 循环输出"的退化样本（我们数据里出现过 300+ 字插入）；
  2. `language=zh` 是否改变结果（模型自带语言声明）。

用法：
  python asr_tune.py --dir _asr_sim/spatial_eval/m0_g722 --clips 196
  python asr_tune.py --dir ... --max-tokens 64 --language zh
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
import score as SC
import simcore as S


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--channel", default="L", choices=["L", "R"])
    ap.add_argument("--max-tokens", type=int, default=0, help="0 = 模型默认")
    ap.add_argument("--language", default="", help="空 = 不指定")
    ap.add_argument("--backend", default="vulkan")
    args = ap.parse_args()

    items = S.load_manifest(args.clips, args.start)
    refs = {it["id"]: it["reference"] for it in items}

    import asr_harness as AH
    eng = AH.AsrEngine(backend=args.backend,
                       language=(args.language or None),
                       max_tokens=args.max_tokens)
    try:
        res = SC.score_dir(args.dir, refs, eng, channel=args.channel)
    finally:
        eng.close()

    per = [r["cer"] * 100 for r in res["rows"]]
    tag = f"max_tokens={args.max_tokens or '默认'} language={args.language or '默认'}"
    print(f"\n=== {tag} ===")
    print(f"  条数 {res['clips']}  合并CER {res['cer']:.2f}%  稳健CER {res['cer_robust']:.2f}%")
    print(f"  整句全对 {res['perfect']}/{res['clips']}  逐条中位 {statistics.median(per):.1f}%")
    print(f"  退化样本(>300%) {len(res['degenerate'])}: {', '.join(x[:20] for x in res['degenerate']) or '无'}")
    out = os.path.join(args.dir, f"tune_mt{args.max_tokens}_{args.language or 'def'}.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump({k: v for k, v in res.items() if k != "rows"}, f, ensure_ascii=False, indent=2)
    print(f"  写入 {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
