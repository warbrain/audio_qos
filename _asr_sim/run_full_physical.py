"""Full physical evaluation: 196 test clips through the real box, captured from
the WhisPrompt UAC, scored on three channel paths.

Channel paths evaluated per clip:
    L      denoise-chip output (UAC channel 0)
    R      raw microphone     (UAC channel 1)
    mean   what the scepter_client HMI actually feeds the ASR: (L + R) // 2

Conditions:
    quiet  no injected noise (measures the device's own noise floor)
    snr00  pink noise mixed into the mouth channel at the same RMS as speech

Everything is kept on disk: per-clip WAVs, per-clip capture levels, per-clip
transcripts and CERs, a machine-readable summary, and a markdown report.
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import unicodedata
import wave

import numpy as np

ROOT = r"D:\App\cppApp\audio_qos\_asr_sim"
OUT = os.path.join(ROOT, "physical_full")
PY = r"D:\App\cppApp\audio_qos\_praenv\Scripts\python.exe"
DATASET = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5"
N_CLIPS = 196
RATE = 16000

CONDITIONS = [("quiet", []), ("snr00", ["--noise-dbfs", "-30", "--noise-mouth"])]


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


def build_manifest() -> str:
    os.makedirs(OUT, exist_ok=True)
    mf = json.load(open(os.path.join(DATASET, "heldout-manifest.strict-v5.json"), encoding="utf-8"))
    items = []
    for c in mf["recordings"][:N_CLIPS]:
        p = os.path.join(DATASET, c["audio"].replace("/", os.sep))
        items.append({"id": os.path.splitext(os.path.basename(p))[0], "audio": p,
                      "speaker": c.get("speaker", ""), "reference": c["reference"]})
    path = os.path.join(OUT, f"clips{N_CLIPS}.json")
    json.dump(items, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"manifest: {len(items)} clips", flush=True)
    return path


def capture(cond: str, extra: list[str], clips_path: str) -> list[dict]:
    d = os.path.join(OUT, cond)
    os.makedirs(d, exist_ok=True)
    args = [PY, os.path.join(ROOT, "box_capture.py"), "--manifest", clips_path,
            "--play-dbfs", "-30", "--out", d] + extra
    rows: list[dict] = []
    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, encoding="utf-8", errors="replace", bufsize=1)
    assert proc.stdout is not None
    for line in proc.stdout:
        if line.startswith("{"):
            rows.append(json.loads(line))
            if len(rows) % 20 == 0:
                print(f"    {cond}: {len(rows)}/{N_CLIPS}", flush=True)
    proc.wait()
    with open(os.path.join(d, "capture.json"), "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    print(f"    {cond}: captured {len(rows)}", flush=True)
    return rows


def read_i16(path: str) -> np.ndarray:
    with wave.open(path, "rb") as w:
        n, ch = w.getnframes(), w.getnchannels()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2")
    return x.reshape(-1, ch)[:, 0] if ch > 1 else x


def main() -> int:
    sys.path.insert(0, ROOT)
    import asr_harness as AH

    clips_path = build_manifest()
    clips = {e["id"]: e for e in json.load(open(clips_path, encoding="utf-8"))}
    eng = AH.AsrEngine()

    summary = {"clips": N_CLIPS, "conditions": {}}
    for cond, extra in CONDITIONS:
        print(f"=== {cond}: capture ===", flush=True)
        caps = capture(cond, extra, clips_path)
        cap_by_id = {c["id"]: c for c in caps}
        d = os.path.join(OUT, cond)

        print(f"=== {cond}: scoring three channel paths ===", flush=True)
        results = []
        silent = 0
        clip_frac = []
        for i, cid in enumerate([c["id"] for c in caps], 1):
            lp = os.path.join(d, f"{cid}_L.wav")
            rp = os.path.join(d, f"{cid}_R.wav")
            if not (os.path.exists(lp) and os.path.exists(rp)):
                continue
            L = read_i16(lp).astype(np.int32)
            R = read_i16(rp).astype(np.int32)
            n = min(len(L), len(R))
            L, R = L[:n], R[:n]
            mean = ((L + R) // 2).astype(np.int16)
            if int(np.max(np.abs(L))) == 0 and int(np.max(np.abs(R))) == 0:
                silent += 1
            clip_frac.append(float(np.mean((np.abs(R) >= 32760))) if n else 0.0)

            ref = clips[cid]["reference"]
            r = norm(ref)
            row = {"id": cid, "speaker": clips[cid].get("speaker", ""), "reference": ref,
                   "levels": cap_by_id.get(cid, {}), "ref_len": len(r)}
            for tag, sig in (("L", L.astype(np.int16)), ("R", R.astype(np.int16)), ("mean", mean)):
                pcm = sig.astype(np.float32) / 32768.0
                hyp = eng.transcribe(pcm, RATE)
                dst = edit_distance(r, norm(hyp)) if r else 0
                row[tag] = {"hyp": hyp, "dist": dst,
                            "cer": (dst / len(r)) if r else 0.0}
            results.append(row)
            if i % 20 == 0:
                print(f"    {cond}: scored {i}/{len(caps)}", flush=True)

        with open(os.path.join(d, "asr.jsonl"), "w", encoding="utf-8") as f:
            for r in results:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")

        csum = {}
        for tag in ("L", "R", "mean"):
            dist = sum(r[tag]["dist"] for r in results)
            tot = sum(r["ref_len"] for r in results)
            csum[tag] = {
                "cer": round(100.0 * dist / tot, 3) if tot else 0.0,
                "perfect": sum(1 for r in results if r[tag]["dist"] == 0),
                "dist": dist, "ref_len": tot,
            }
        csum["n"] = len(results)
        csum["all_silent_clips"] = silent
        csum["mean_clip_fraction_R"] = round(float(np.mean(clip_frac)), 6) if clip_frac else 0.0
        summary["conditions"][cond] = csum
        print(f"    {cond}: " + "  ".join(f"{k} {v['cer']:.2f}%" for k, v in csum.items()
                                          if isinstance(v, dict)), flush=True)

    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print("\n=== SUMMARY ===", flush=True)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
