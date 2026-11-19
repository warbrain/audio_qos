"""对比 HMI 出口的 4 种双声道→单声道融合策略（离线，用已有 L/R 采集）。

对应 audio_capture.cpp:286 的硬编码 `0.5*(L+R)`，以及 hpp 里已声明但未使用的
Fusion{Mean, Best, Left, Right} 枚举。

支持多个采集目录（合并成一个大集合），参考文本取自各目录的 capture.json。

用法：
    python fusion_compare.py --src dirA,dirB --backend cuda
    python fusion_compare.py --src dirA --clips 24
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import tempfile
import wave

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

import numpy as np

import score as SC

HOLD_DB = 1.5
BLOCK_S = 0.05


def read_wav(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as w:
        n, ch, sr = w.getnframes(), w.getnchannels(), w.getframerate()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch)[:, 0]
    return x, sr


def write_wav(path: str, x: np.ndarray, fs: int) -> None:
    y = np.clip(np.round(x * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(fs)
        w.writeframes(y.tobytes())


def fuse(left: np.ndarray, right: np.ndarray, mode: str, fs: int) -> np.ndarray:
    if mode == "mean":
        return 0.5 * (left + right)
    if mode == "left":
        return left.copy()
    if mode == "right":
        return right.copy()
    if mode == "best":
        n = min(len(left), len(right))
        out = np.zeros(n)
        blk = max(1, int(BLOCK_S * fs))
        pick_left = True
        for a in range(0, n, blk):
            b = min(a + blk, n)
            el = float(np.mean(left[a:b] ** 2)) + 1e-20
            er = float(np.mean(right[a:b] ** 2)) + 1e-20
            dl, dr = 10 * np.log10(el), 10 * np.log10(er)
            if pick_left and dr > dl + HOLD_DB:
                pick_left = False
            elif (not pick_left) and dl > dr + HOLD_DB:
                pick_left = True
            out[a:b] = left[a:b] if pick_left else right[a:b]
        return out
    raise SystemExit(f"unknown mode {mode}")


def collect(dirs: list[str], limit: int) -> tuple[list[tuple[str, str, str]], dict]:
    refs: dict[str, str] = {}
    files: list[tuple[str, str, str]] = []
    for d in dirs:
        cap = os.path.join(d, "capture.json")
        if os.path.exists(cap):
            for line in open(cap, encoding="utf-8"):
                if line.strip():
                    r = json.loads(line)
                    refs[r["id"]] = r["reference"]
        for f in sorted(os.listdir(d)):
            if not f.endswith("_L.wav"):
                continue
            cid = f[: -len("_L.wav")]
            rp = os.path.join(d, cid + "_R.wav")
            if os.path.exists(rp):
                files.append((cid, os.path.join(d, f), rp))
    if limit:
        files = files[:limit]
    return files, refs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="逗号分隔的采集目录")
    ap.add_argument("--clips", type=int, default=0, help="只取前 N 条（0=全部）")
    ap.add_argument("--backend", default="cuda")
    ap.add_argument("--modes", default="mean,left,right,best")
    ap.add_argument("--save-json", default="", help="把逐条结果写入该 json")
    args = ap.parse_args()

    dirs = [d.strip() for d in args.src.split(",") if d.strip()]
    files, refs = collect(dirs, args.clips)
    files = [(c, l, r) for c, l, r in files if c in refs]
    if not files:
        print("  没有可用的成对 L/R 文件")
        return 1
    print(f"  样本 {len(files)} 条   来源 {len(dirs)} 个目录   模式 {args.modes}", flush=True)

    eng = SC.open_asr(args.backend)
    rows = {}
    try:
        for mode in [m.strip() for m in args.modes.split(",") if m.strip()]:
            tmp = tempfile.mkdtemp(prefix=f"fusion_{mode}_")
            try:
                for cid, lp, rp in files:
                    l, fs = read_wav(lp)
                    r, _ = read_wav(rp)
                    write_wav(os.path.join(tmp, cid + "_L.wav"), fuse(l, r, mode, fs), fs)
                res = SC.score_dir(tmp, refs, eng)
                rows[mode] = res
                print(f"  {mode:6s} CER {res['cer']:6.2f}%   全对 {res['perfect']}/{res['clips']}", flush=True)
                if args.save_json:
                    with open(args.save_json.replace('.json', f'_{mode}.jsonl'), 'w', encoding='utf-8') as fh:
                        for item in res.get('rows', []) or []:
                            fh.write(json.dumps(item, ensure_ascii=False) + "\n")
            finally:
                shutil.rmtree(tmp, ignore_errors=True)
    finally:
        eng.close()

    print("\n  === 融合策略对比 ===")
    base = rows.get("mean")
    for mode, res in rows.items():
        delta = f"{res['cer'] - base['cer']:+.2f} pp" if base and mode != "mean" else "基线"
        print(f"  {mode:6s} {res['cer']:6.2f}%   全对 {res['perfect']:3d}/{res['clips']}   {delta}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
