"""ASR 评分（不需要声卡，只需要 native.dll + 一个输出目录）。

native.dll 的部分副本被 Smart App Control 拦截，`asr_harness.resolve_dll()`
会自动挑能加载的那个；CUDA 后端在该构建里未注册，故固定用 vulkan。
"""
from __future__ import annotations

import json
import os
import sys
import unicodedata

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
from simcore import load_wav_mono

ROOT = os.path.dirname(_HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)   # asr_harness.py 在 _asr_sim 下


def norm(s: str) -> str:
    s = unicodedata.normalize("NFKC", s or "").lower()
    return "".join(c for c in s if unicodedata.category(c)[0] not in ("P", "S", "Z", "C"))


def edit_distance(a: str, b: str) -> int:
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def open_asr(backend: str = "vulkan"):
    import asr_harness as AH

    return AH.AsrEngine(backend=backend)


def score_dir(d: str, references: dict[str, str], eng, channel: str = "L") -> dict:
    """对目录里的 `<id>_<channel>.wav` 逐条识别并打分。"""
    rows = []
    for f in sorted(os.listdir(d)):
        if not f.endswith(f"_{channel}.wav"):
            continue
        cid = f[: -(len(channel) + 5)]
        x, fs = load_wav_mono(os.path.join(d, f))
        hyp = eng.transcribe(x.astype("float32"), fs)
        ref = references.get(cid, "")
        rr, hh = norm(ref), norm(hyp)
        dist = edit_distance(rr, hh) if rr else 0
        rows.append({"id": cid, "reference": ref, "hyp": hyp, "dist": dist,
                     "ref_len": len(rr), "cer": (dist / len(rr)) if rr else 0.0})
    tot = sum(r["ref_len"] for r in rows)
    dist = sum(r["dist"] for r in rows)
    degenerate = [r["id"] for r in rows if r["cer"] > 3.0]
    robust = [r for r in rows if r["cer"] <= 3.0]
    rtot = sum(r["ref_len"] for r in robust)
    rdist = sum(r["dist"] for r in robust)
    return {
        "dir": d,
        "clips": len(rows),
        "cer": round(100 * dist / tot, 2) if tot else None,
        "perfect": sum(1 for r in rows if r["dist"] == 0),
        "cer_robust": round(100 * rdist / rtot, 2) if rtot else None,
        "perfect_robust": sum(1 for r in robust if r["dist"] == 0),
        "robust_clips": len(robust),
        "degenerate": degenerate,
        "rows": rows,
    }


def write_report(d: str, res: dict) -> None:
    with open(os.path.join(d, "asr.jsonl"), "w", encoding="utf-8") as f:
        for r in res["rows"]:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    with open(os.path.join(d, "score.json"), "w", encoding="utf-8") as f:
        json.dump({k: v for k, v in res.items() if k != "rows"}, f,
                  ensure_ascii=False, indent=2)
