"""Full evaluation report for the 196-clip single-mic simulation.

Reads <dir>/results.jsonl (written by run_testset.py) and reports:
  * CER / perfect-count matrix over T60 x SNR
  * marginal effects of reverberation and noise
  * error decomposition (substitutions / insertions / deletions)
  * ASR performance (RTF, latency) alongside accuracy
  * failure taxonomy and worst utterances
  * per-speaker breakdown
Also writes report.md next to the results.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import statistics

ORDER = ["clean", "t010_snr+20", "t010_snr+10", "t010_snr+00",
         "t030_snr+20", "t030_snr+10", "t030_snr+00",
         "t060_snr+20", "t060_snr+10", "t060_snr+00"]

LABEL = {"clean": "无箱体/无噪声 (clean)",
         "t010_snr+20": "T60 0.10s / SNR +20dB", "t010_snr+10": "T60 0.10s / SNR +10dB",
         "t010_snr+00": "T60 0.10s / SNR   0dB",
         "t030_snr+20": "T60 0.30s / SNR +20dB", "t030_snr+10": "T60 0.30s / SNR +10dB",
         "t030_snr+00": "T60 0.30s / SNR   0dB",
         "t060_snr+20": "T60 0.60s / SNR +20dB", "t060_snr+10": "T60 0.60s / SNR +10dB",
         "t060_snr+00": "T60 0.60s / SNR   0dB"}


def norm(s: str) -> str:
    """Same normalisation the CER uses: NFKC, drop punctuation/symbols/space."""
    import unicodedata
    s = unicodedata.normalize("NFKC", s or "").lower()
    return "".join(ch for ch in s if unicodedata.category(ch)[0] not in ("P", "S", "Z", "C"))


def align_counts(ref: str, hyp: str):
    """Return (substitutions, insertions, deletions) via Levenshtein backtrace."""
    ref, hyp = norm(ref), norm(hyp)
    n, m = len(ref), len(hyp)
    d = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        d[i][0] = i
    for j in range(m + 1):
        d[0][j] = j
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            d[i][j] = min(d[i - 1][j] + 1, d[i][j - 1] + 1,
                          d[i - 1][j - 1] + (ref[i - 1] != hyp[j - 1]))
    i, j, s, ins, dele = n, m, 0, 0, 0
    while i > 0 or j > 0:
        if i > 0 and j > 0 and d[i][j] == d[i - 1][j - 1] + (ref[i - 1] != hyp[j - 1]):
            if ref[i - 1] != hyp[j - 1]:
                s += 1
            i -= 1
            j -= 1
        elif i > 0 and d[i][j] == d[i - 1][j] + 1:
            dele += 1
            i -= 1
        else:
            ins += 1
            j -= 1
    return s, ins, dele


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=r"D:\App\cppApp\audio_qos\_asr_sim\grid_final")
    args = ap.parse_args()

    rows = [json.loads(l) for l in open(os.path.join(args.dir, "results.jsonl"),
                                        encoding="utf-8") if l.strip()]
    by = collections.defaultdict(list)
    for r in rows:
        by[r["condition"]].append(r)
    conds = [c for c in ORDER if c in by]
    n_clips = len({r["id"] for r in rows})
    out = []
    w = out.append

    w(f"# 单麦 1 m³ 箱体仿真评测报告\n")
    w(f"- 测试集：`qwen3-asr-strict-v5` heldout 196 条 scored 片段（28 位说话人，全为耳语）")
    w(f"- 仿真：pyroomacoustics，1×1×1 m 混响箱体，人工嘴距麦克风 **10 cm**，心形指向性 p=0.5")
    w(f"- 语料电平：源 RMS 中位 −61.3 dBFS，渲染后统一归一化到 −26 dBFS（+55 dB 上限）")
    w(f"- 识别：Qwen3-ASR（`native.dll` ABI 0.5.0，CUDA）")
    w(f"- 规模：{n_clips} 条 × {len(conds)} 条件 = {len(rows)} 次推理\n")

    # ---- accuracy matrix -------------------------------------------------
    w("## 1. 识别精度（CER）\n")
    w("| 条件 | 条数 | CER | 全对 | 中位 CER | P90 |")
    w("|---|---:|---:|---:|---:|---:|")
    stats = {}
    for c in conds:
        rs = by[c]
        d = sum(r["dist"] for r in rs)
        n = sum(r["ref_len"] for r in rs)
        cers = sorted(r["cer"] for r in rs)
        perfect = sum(1 for r in rs if r["dist"] == 0)
        med = 100 * statistics.median(cers)
        p90 = 100 * cers[int(0.9 * (len(cers) - 1))]
        stats[c] = dict(cer=100 * d / n, perfect=perfect, n=len(rs), med=med, p90=p90)
        w(f"| {LABEL[c]} | {len(rs)} | **{100*d/n:.2f}%** | {perfect}/{len(rs)} | {med:.2f}% | {p90:.2f}% |")
    w("")

    # ---- marginals -------------------------------------------------------
    w("## 2. 混响与噪声各自的代价\n")
    t60s = [0.10, 0.30, 0.60]
    snrs = [20, 10, 0]
    w("**固定 T60，平均三个 SNR：**\n")
    w("| T60 | CER |")
    w("|---|---:|")
    for t in t60s:
        key = f"t{int(round(t*100)):03d}"
        rs = [r for c in conds if c.startswith(key + "_") for r in by[c]]
        d = sum(r["dist"] for r in rs)
        n = sum(r["ref_len"] for r in rs)
        w(f"| {t:.2f} s | **{100*d/n:.2f}%** |")
    w("")
    w("**固定 SNR，平均三个 T60：**\n")
    w("| SNR | CER |")
    w("|---|---:|")
    for s in snrs:
        rs = [r for c in conds if c.endswith(f"snr{s:+03d}") for r in by[c]]
        d = sum(r["dist"] for r in rs)
        n = sum(r["ref_len"] for r in rs)
        w(f"| {s:+d} dB | **{100*d/n:.2f}%** |")
    w("")

    # ---- error decomposition --------------------------------------------
    w("## 3. 错误构成（替换 / 插入 / 删除）\n")
    w("| 条件 | 替换 | 插入 | 删除 | 总错字 |")
    w("|---|---:|---:|---:|---:|")
    for c in conds:
        S = I = D = 0
        for r in by[c]:
            s, i, d = align_counts(r["reference"], r["hypothesis"])
            S += s
            I += i
            D += d
        w(f"| {LABEL[c]} | {S} | {I} | {D} | {S+I+D} |")
    w("")

    # ---- ASR performance -------------------------------------------------
    w("## 4. ASR 性能（与精度并看）\n")
    w("| 条件 | 平均 RTF | 平均耗时 | 音频总时长 | 累计 GPU 时间 |")
    w("|---|---:|---:|---:|---:|")
    for c in conds:
        rs = by[c]
        rtf = statistics.mean(r["rtf"] for r in rs)
        ms = statistics.mean(r["asr_ms"] for r in rs)
        audio = sum(r["audio_s"] for r in rs)
        total = sum(r["asr_ms"] for r in rs) / 1000.0
        w(f"| {LABEL[c]} | {rtf:.3f} | {ms:.0f} ms | {audio:.0f} s | {total:.0f} s |")
    all_rtf = [r["rtf"] for r in rows]
    w(f"\n全体 RTF：均值 {statistics.mean(all_rtf):.3f}，中位 {statistics.median(all_rtf):.3f}，"
      f"最大 {max(all_rtf):.3f}\n")

    # ---- failure taxonomy ------------------------------------------------
    w("## 5. 失败形态\n")
    w("| 条件 | 空输出 | CER>100% | 50–100% | 10–50% | 0–10% |")
    w("|---|---:|---:|---:|---:|---:|")
    for c in conds:
        rs = by[c]
        e = sum(1 for r in rs if not r["hypothesis"].strip())
        g = sum(1 for r in rs if r["cer"] > 1.0)
        h = sum(1 for r in rs if 0.5 < r["cer"] <= 1.0)
        m = sum(1 for r in rs if 0.1 < r["cer"] <= 0.5)
        l = sum(1 for r in rs if 0 < r["cer"] <= 0.1)
        w(f"| {LABEL[c]} | {e} | {g} | {h} | {m} | {l} |")
    w("")

    # ---- per speaker -----------------------------------------------------
    w("## 6. 说话人维度（T60 0.60s / SNR 0dB 最差档）\n")
    sp = collections.defaultdict(lambda: [0, 0, 0])
    for r in by.get("t060_snr+00", []):
        a = sp[r["speaker"]]
        a[0] += r["dist"]
        a[1] += r["ref_len"]
        a[2] += 1
    if sp:
        ranked = sorted(sp.items(), key=lambda kv: kv[1][0] / kv[1][1])
        w(f"共 {len(sp)} 位说话人。最好 3 位与最差 3 位：\n")
        w("| 说话人 | 条数 | CER |")
        w("|---|---:|---:|")
        for s, (d, n, c) in ranked[:3]:
            w(f"| {s} | {c} | {100*d/n:.1f}% |")
        w("| … | | |")
        for s, (d, n, c) in ranked[-3:]:
            w(f"| {s} | {c} | {100*d/n:.1f}% |")
        w("")

    # ---- worst utts ------------------------------------------------------
    w("## 7. 最差样本（T60 0.60s / SNR 0dB）\n")
    worst = sorted(by.get("t060_snr+00", []), key=lambda r: -r["cer"])[:5]
    for r in worst:
        w(f"- `{r['id']}`（{r['speaker']}，CER {r['cer']*100:.1f}%）")
        w(f"  - 参考：{r['reference']}")
        w(f"  - 识别：{r['hypothesis'] or '（空）'}")
    w("")

    text = "\n".join(out)
    path = os.path.join(args.dir, "report.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(text)
    print(f"\n# 已写出 {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
