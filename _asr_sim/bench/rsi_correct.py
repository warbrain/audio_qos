"""RSI 式递归自我提升：从错误里挖修正规则，逐轮应用到识别结果上。

严格防止数据泄漏
----------------
按**说话人**切分：一半说话人做 dev（挖规则），另一半做 test（评估）。
规则只在 dev 上挖，只在 test 上评，所以提升是真的泛化效果，不是背答案。

每轮做什么
----------
1. 把 dev 的参考文本与识别结果做字符级对齐，统计替换对 (识别字 → 参考字)；
2. 只保留"方向一致"的高频对：该识别字在多数出现位置都被替换成同一个参考字；
3. 把规则应用到 test 的识别结果；
4. 重算 test CER，进入下一轮（用修正后的结果继续挖）。

用法：
  python rsi_correct.py --dir _asr_sim/spatial_eval/m0_g722 --rounds 4
"""
from __future__ import annotations

import argparse
import json
import os
import re
import statistics
import sys
from collections import Counter, defaultdict

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
import score as SC


def speaker_of(cid: str) -> str:
    m = re.match(r"(S\d+)", cid)
    return m.group(1) if m else cid[:5]


def align_ops(ref: str, hyp: str) -> list[tuple[str, str, str]]:
    """返回编辑操作序列 [(op, ref_char, hyp_char)]，op ∈ {eq, sub, del, ins}。"""
    n, m = len(ref), len(hyp)
    dp = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        dp[i][0] = i
    for j in range(m + 1):
        dp[0][j] = j
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            cost = 0 if ref[i - 1] == hyp[j - 1] else 1
            dp[i][j] = min(dp[i - 1][j] + 1, dp[i][j - 1] + 1, dp[i - 1][j - 1] + cost)
    ops = []
    i, j = n, m
    while i > 0 or j > 0:
        if i > 0 and j > 0 and ref[i - 1] == hyp[j - 1] and dp[i][j] == dp[i - 1][j - 1]:
            ops.append(("eq", ref[i - 1], hyp[j - 1])); i -= 1; j -= 1
        elif i > 0 and j > 0 and dp[i][j] == dp[i - 1][j - 1] + 1:
            ops.append(("sub", ref[i - 1], hyp[j - 1])); i -= 1; j -= 1
        elif i > 0 and dp[i][j] == dp[i - 1][j] + 1:
            ops.append(("del", ref[i - 1], "")); i -= 1
        else:
            ops.append(("ins", "", hyp[j - 1])); j -= 1
    ops.reverse()
    return ops


def mine(pairs: list[tuple[str, str]], min_count: int = 2,
         min_ratio: float = 0.6) -> dict[str, str]:
    """从 (ref, hyp) 对里挖字符级替换规则 hyp_char -> ref_char。"""
    forward = Counter()
    total = Counter()
    for ref, hyp in pairs:
        for op, rc, hc in align_ops(ref, hyp):
            if op != "sub":
                continue
            if not ("\u4e00" <= rc <= "\u9fff" and "\u4e00" <= hc <= "\u9fff"):
                continue
            forward[(hc, rc)] += 1
            total[hc] += 1
    rules = {}
    for (hc, rc), cnt in forward.items():
        if cnt >= min_count and total[hc] > 0 and cnt / total[hc] >= min_ratio:
            rules[hc] = rc
    return rules


def apply_rules(text: str, rules: dict[str, str]) -> str:
    return "".join(rules.get(c, c) for c in text)


def cer_of(rows: list[dict]) -> float:
    tot = sum(r["ref_len"] for r in rows)
    dist = sum(SC.edit_distance(SC.norm(r["reference"]), SC.norm(r["hyp"])) for r in rows)
    return 100 * dist / tot if tot else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="含 asr.jsonl 的采集目录")
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--min-count", type=int, default=2)
    ap.add_argument("--min-ratio", type=float, default=0.6)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    path = os.path.join(args.dir, "asr.jsonl")
    rows = [json.loads(l) for l in open(path, encoding="utf-8") if l.strip()]
    for r in rows:
        r.setdefault("speaker", speaker_of(r["id"]))
        r["ref_len"] = len(SC.norm(r["reference"]))

    speakers = sorted({r["speaker"] for r in rows})
    dev_spk = set(speakers[::2])
    dev = [r for r in rows if r["speaker"] in dev_spk]
    test = [r for r in rows if r["speaker"] not in dev_spk]
    print(f"  总 {len(rows)} 条 / {len(speakers)} 说话人")
    print(f"  dev  {len(dev):3d} 条（{len(dev_spk)} 说话人）")
    print(f"  test {len(test):3d} 条（{len(speakers)-len(dev_spk)} 说话人）")
    print(f"  基线: dev {cer_of(dev):6.2f}%   test {cer_of(test):6.2f}%\n")

    for r in rows:
        r["hyp_cur"] = r["hyp"]

    history = []
    all_rules: dict[str, str] = {}
    print(f"  {'轮次':>4s} {'新增规则':>8s} {'规则总数':>8s} {'dev CER':>9s} {'test CER':>9s} {'test 变化':>10s}")
    print("  " + "-" * 62)
    base_test = cer_of(test)
    for rnd in range(1, args.rounds + 1):
        dev_pairs = [(SC.norm(r["reference"]), SC.norm(r["hyp_cur"])) for r in dev]
        rules = mine(dev_pairs, args.min_count, args.min_ratio)
        new_rules = {k: v for k, v in rules.items() if all_rules.get(k) != v}
        if not new_rules:
            print(f"  {rnd:4d} {0:8d} {len(all_rules):8d}   （无新规则，收敛）")
            break
        all_rules.update(new_rules)
        for r in rows:
            r["hyp_cur"] = apply_rules(r["hyp_cur"], new_rules)
        dev_cer = cer_of(dev); test_cer = cer_of(test)
        history.append({"round": rnd, "new_rules": len(new_rules), "total_rules": len(all_rules),
                        "dev_cer": round(dev_cer, 2), "test_cer": round(test_cer, 2)})
        print(f"  {rnd:4d} {len(new_rules):8d} {len(all_rules):8d} {dev_cer:8.2f}% "
              f"{test_cer:8.2f}% {test_cer-base_test:+9.2f} pp")

    print(f"\n  最终：test CER {base_test:.2f}% → {cer_of(test):.2f}%")
    top = sorted(all_rules.items(), key=lambda kv: kv[0])[:20]
    print(f"  规则示例（共 {len(all_rules)} 条）: " + " ".join(f"{k}→{v}" for k, v in top))

    out = os.path.join(args.dir, "rsi_report.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"dev_speakers": sorted(dev_spk), "baseline_test": round(base_test, 2),
                   "history": history, "rules": all_rules}, f, ensure_ascii=False, indent=2)
    print(f"  写入 {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
