"""Sweep the dual-mic firmware's fixed gain and measure ASR impact.

For each gain it sets both channels over HID (config item 2/3, version 1), captures
the same clip set through the box, and scores the enhanced... (no: plain) channel 0.

Usage: python run_gain_sweep.py [--gains 1,2,4,8] [--clips 24]
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
import unicodedata
import wave

import numpy as np

ROOT = r"D:\App\cppApp\audio_qos\_asr_sim"
OUT = os.path.join(ROOT, "gain_sweep")
PY_FULL = r"D:\App\cppApp\audio_qos\_praenv\Scripts\python.exe"
DATASET = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5"
RATE = 16000

sys.path.insert(0, ROOT)
from set_spatial import open_config, request, DIR_WRITE, DIR_READ  # noqa: E402

ITEM_GAIN_L, ITEM_GAIN_R = 2, 3
VERSION_CONFIG = 1
MODE_BYPASS = 0
ITEM_SPATIAL_REQUESTED = 18
VERSION_SPATIAL = 3


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


def build_manifest(n: int) -> str:
    os.makedirs(OUT, exist_ok=True)
    mf = json.load(open(os.path.join(DATASET, "heldout-manifest.strict-v5.json"), encoding="utf-8"))
    items = []
    for c in mf["recordings"][:n]:
        p = os.path.join(DATASET, c["audio"].replace("/", os.sep))
        items.append({"id": os.path.splitext(os.path.basename(p))[0], "audio": p,
                      "speaker": c.get("speaker", ""), "reference": c["reference"]})
    path = os.path.join(OUT, f"clips{n}.json")
    json.dump(items, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    return path


def set_gain(h, gain_x: int) -> None:
    for item in (ITEM_GAIN_L, ITEM_GAIN_R):
        got, err = request(h, VERSION_CONFIG, item, gain_x, DIR_WRITE)
        print(f"  gain {gain_x}x: item {item} -> {got if got is not None else err}", flush=True)


def capture(tag: str, clips_path: str) -> list[dict]:
    d = os.path.join(OUT, tag)
    os.makedirs(d, exist_ok=True)
    proc = subprocess.Popen([PY_FULL, os.path.join(ROOT, "box_capture.py"),
                             "--manifest", clips_path, "--play-dbfs", "-30",
                             "--spatial-mode", str(MODE_BYPASS),
                             "--noise-dbfs", "-30", "--noise-mouth", "--out", d],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, encoding="utf-8", errors="replace", bufsize=1)
    rows = []
    for line in proc.stdout:
        if line.startswith("{"):
            rows.append(json.loads(line))
    proc.wait()
    with open(os.path.join(d, "capture.json"), "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gains", default="1,2,4,8")
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--backend", default="vulkan")
    args = ap.parse_args()
    gains = [int(x) for x in args.gains.split(",") if x.strip()]

    import asr_harness as AH

    clips_path = build_manifest(args.clips)
    clips = {e["id"]: e for e in json.load(open(clips_path, encoding="utf-8"))}
    h = open_config()

    # keep the spatial mode fixed at bypass so the gain effect is isolated
    got, err = request(h, VERSION_SPATIAL, ITEM_SPATIAL_REQUESTED, MODE_BYPASS, DIR_WRITE)
    print(f"spatial mode -> {MODE_BYPASS} (reply: {got if got is not None else err})", flush=True)

    eng = AH.AsrEngine(backend=args.backend)
    summary = []
    for g in gains:
        set_gain(h, g)
        tag = f"g{g}"
        rows = capture(tag, clips_path)
        d = os.path.join(OUT, tag)
        print(f"  captured {len(rows)} clips at {g}x", flush=True)

        results = []
        for r in rows:
            p = os.path.join(d, f"{r['id']}_L.wav")
            if not os.path.exists(p):
                continue
            with wave.open(p, "rb") as w:
                n, ch = w.getnframes(), w.getnchannels()
                raw = w.readframes(n)
            x = np.frombuffer(raw, dtype="<i2")
            pcm = (x.reshape(-1, ch)[:, 0] if ch > 1 else x).astype(np.float32) / 32768.0
            hyp = eng.transcribe(pcm, RATE)
            ref = clips[r["id"]]["reference"]
            rr, hh = norm(ref), norm(hyp)
            dist = edit_distance(rr, hh) if rr else 0
            results.append({"id": r["id"], "reference": ref, "hyp": hyp, "dist": dist,
                            "ref_len": len(rr), "cer": (dist / len(rr)) if rr else 0.0,
                            "L": r.get("L", r.get("L_denoised")), "R": r.get("R", r.get("R_raw"))})
        with open(os.path.join(d, "asr.jsonl"), "w", encoding="utf-8") as f:
            for x in results:
                f.write(json.dumps(x, ensure_ascii=False) + "\n")

        dist = sum(x["dist"] for x in results)
        tot = sum(x["ref_len"] for x in results)
        perfect = sum(1 for x in results if x["dist"] == 0)
        peaks = [x["L"]["peak_dbfs"] for x in results if x.get("L")]
        rms = [x["L"]["rms_dbfs"] for x in results if x.get("L")]
        entry = {"gain_x": g, "clips": len(results),
                 "cer": round(100 * dist / tot, 3) if tot else 0.0,
                 "perfect": perfect,
                 "peak_median_dbfs": round(statistics.median(peaks), 2) if peaks else None,
                 "peak_max_dbfs": round(max(peaks), 2) if peaks else None,
                 "rms_median_dbfs": round(statistics.median(rms), 2) if rms else None}
        summary.append(entry)
        print(f"  {g}x: CER {entry['cer']:.2f}%  perfect {perfect}/{len(results)}  "
              f"peak(med/max) {entry['peak_median_dbfs']}/{entry['peak_max_dbfs']} dBFS", flush=True)

    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"clips": args.clips, "results": summary}, f, ensure_ascii=False, indent=2)
    h.close()
    eng.close()

    print("\n=== SUMMARY ===")
    print(f"{'增益':>4s} {'条数':>5s} {'CER':>8s} {'全对':>8s} {'峰值中位':>9s} {'峰值最大':>9s}")
    for e in summary:
        print(f"{e['gain_x']:3d}x {e['clips']:5d} {e['cer']:7.2f}% {e['perfect']:4d}/{e['clips']:<3d} "
              f"{e['peak_median_dbfs']:8.1f} {e['peak_max_dbfs']:8.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
