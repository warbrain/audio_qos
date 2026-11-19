"""构建配对数据集：同一批语料在不同信噪比下的箱体采集。

训练降噪/前端模型需要**成对**数据，且必须是同域（同箱体、同麦克风、同编码链）。
本脚本按条件批量采集，并写出训练用的清单。

条件设计（噪声走人工嘴，与历史基线口径一致）
  quiet   无噪声                → 干净的"域内参考"（含箱体混响，比干声更贴近推理分布）
  snr+6   语音 -24 / 噪声 -30   → 偏易
  snr0    语音 -30 / 噪声 -30   → 主基线（与 196 条历史数据同口径）
  snr-6   语音 -36 / 噪声 -30   → 偏难（小声说话）

输出：
  <out>/<condition>/<id>_L.wav、_R.wav     实际采集
  <out>/pairs.jsonl                        训练清单：{id, reference, condition, clean, noisy}
  <out>/summary.json                       各条件的电平与 SNR 统计

用法：
  python build_dataset.py --clips 196 --conditions quiet,snr0 --out _asr_sim/dataset_v1
  python build_dataset.py --clips 24  --conditions quiet,snr+6,snr0,snr-6 --out _asr_sim/dataset_smoke
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import wave

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

import bootstrap  # noqa: F401
import common as C
import simcore as S

FULL_PY = sys.executable
BOX = os.path.join(os.path.dirname(_HERE), "box_capture.py")

# condition -> (play_dbfs, noise_dbfs or None)
# 电平已按**实测**信噪比标定（标称电平不等于到达麦克风的信噪比）：
#   语音 -30 播放 -> 语音窗 -22.7 dBFS，噪声 -30 播放 -> 尾部 -37.9 dBFS，实测 +15 dB
# 因此每降低 6 dB 语音，实测 SNR 约降 6 dB。
CONDITIONS = {
    "quiet": (-30.0, None),        # 实测 ~+80 dB，作为域内干净参考
    "easy": (-30.0, -30.0),        # 实测 ~+15 dB
    "snr0": (-44.0, -30.0),        # 目标 ~0 dB
    "snr-6": (-50.0, -30.0),       # 目标 ~-6 dB
}


def capture_condition(cond: str, out_dir: str, items: list[dict], extra: list[str]) -> int:
    play, noise = CONDITIONS[cond]
    os.makedirs(out_dir, exist_ok=True)
    manifest = os.path.join(out_dir, "_manifest.json")
    json.dump(items, open(manifest, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    cmd = [FULL_PY, "-X", "utf8", BOX, "--manifest", manifest,
           "--play-dbfs", str(play), "--out", out_dir, "--noise-mouth"]
    if noise is not None:
        cmd += ["--noise-dbfs", str(noise)]
    cmd += extra
    print(f"  [{cond}] 采集 {len(items)} 条 -> {out_dir}", flush=True)
    r = subprocess.run(cmd, check=False, env=bootstrap.child_env(),
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        print(f"  [{cond}] 失败: {r.stderr[-300:]}", file=sys.stderr)
    return r.returncode


def verify(out_dir: str, expect: int) -> dict:
    files = [f for f in os.listdir(out_dir) if f.endswith("_L.wav")]
    silent, levels = 0, []
    for f in files:
        x, _ = S.load_wav_mono(os.path.join(out_dir, f))
        if len(x) == 0 or float(np.max(np.abs(x))) == 0:
            silent += 1
        else:
            levels.append(20 * np.log10(float(np.sqrt(np.mean(x ** 2))) + 1e-12))
    return {"files": len(files), "expected": expect, "silent": silent,
            "rms_median": round(float(np.median(levels)), 1) if levels else None}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--conditions", default="quiet,snr0")
    ap.add_argument("--out", required=True)
    ap.add_argument("--extra", nargs="*", default=[], help="传给 box_capture 的额外参数")
    args = ap.parse_args()

    conds = [c.strip() for c in args.conditions.split(",") if c.strip()]
    bad = [c for c in conds if c not in CONDITIONS]
    if bad:
        print(f"未知条件: {bad}，可选 {list(CONDITIONS)}", file=sys.stderr)
        return 2

    items = S.load_manifest(args.clips, args.start)
    os.makedirs(args.out, exist_ok=True)

    summary = {}
    for cond in conds:
        d = os.path.join(args.out, cond)
        capture_condition(cond, d, items, args.extra)
        info = verify(d, len(items))
        summary[cond] = info
        flag = "⚠ 全零" if info["silent"] else "ok"
        print(f"  [{cond}] {info['files']}/{info['expected']} 文件  "
              f"RMS 中位 {info['rms_median']} dBFS  {flag}", flush=True)
        if info["silent"] > 0:
            print(f"  [{cond}] 检出全零音频：链路假死，请断电重启 Scepter 后重跑", flush=True)

    # 训练清单
    clean_dir = os.path.join(args.out, "quiet")
    pairs = []
    if os.path.isdir(clean_dir):
        for it in items:
            cid = it["id"]
            clean = os.path.join(clean_dir, f"{cid}_L.wav")
            if not os.path.exists(clean):
                continue
            for cond in conds:
                if cond == "quiet":
                    continue
                noisy = os.path.join(args.out, cond, f"{cid}_L.wav")
                if os.path.exists(noisy):
                    pairs.append({"id": cid, "reference": it["reference"],
                                  "condition": cond, "clean": clean, "noisy": noisy})
    with open(os.path.join(args.out, "pairs.jsonl"), "w", encoding="utf-8") as f:
        for p in pairs:
            f.write(json.dumps(p, ensure_ascii=False) + "\n")
    with open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"clips": len(items), "conditions": summary, "pairs": len(pairs)},
                  f, ensure_ascii=False, indent=2)

    print(f"\n  配对样本 {len(pairs)} 条 -> {os.path.join(args.out, 'pairs.jsonl')}")
    print(f"  总时长约 {len(items) * 6 / 60:.0f} 分钟/条件")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
