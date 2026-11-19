"""Whisper + noise matrix: hold the noise source fixed and lower the speech
level, which is what actually happens when someone whispers in a noisy room.

Conditions (play_dbfs, noise_dbfs):
  A  -30 / -30  SNR   0 dB  baseline (matches the 8x baseline run)
  B  -36 / -30  SNR  -6 dB  quiet voice
  C  -42 / -30  SNR -12 dB  whisper in noise
  D  -42 / -42  SNR   0 dB  control: same SNR, 12 dB lower level

C vs A isolates the SNR penalty; D vs A isolates the level penalty.

Usage: python run_whisper_noise.py [--clips 24]
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
OUT = os.path.join(ROOT, "whisper_noise")
PY_FULL = r"D:\App\cppApp\audio_qos\_praenv\Scripts\python.exe"
RATE = 16000

sys.path.insert(0, ROOT)
import asr_harness as AH  # noqa: E402

CONDITIONS = [
    ("A_snr00_loud", -30.0, -30.0),
    ("B_snr-6", -36.0, -30.0),
    ("C_snr-12", -42.0, -30.0),
    ("D_snr00_quiet", -42.0, -42.0),
]


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
    mf = json.load(open(os.path.join(r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5",
                                     "heldout-manifest.strict-v5.json"), encoding="utf-8"))
    base = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5"
    items = []
    for c in mf["recordings"][:n]:
        p = os.path.join(base, c["audio"].replace("/", os.sep))
        items.append({"id": os.path.splitext(os.path.basename(p))[0], "audio": p,
                      "speaker": c.get("speaker", ""), "reference": c["reference"]})
    path = os.path.join(OUT, f"clips{n}.json")
    json.dump(items, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    return path


def capture(tag: str, clips_path: str, play: float, noise: float) -> list[dict]:
    d = os.path.join(OUT, tag)
    os.makedirs(d, exist_ok=True)
    proc = subprocess.Popen([PY_FULL, os.path.join(ROOT, "box_capture.py"),
                             "--manifest", clips_path, "--play-dbfs", str(play),
                             "--noise-dbfs", str(noise), "--noise-mouth", "--out", d],
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
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--backend", default="vulkan")
    args = ap.parse_args()

    clips_path = build_manifest(args.clips)
    clips = {e["id"]: e for e in json.load(open(clips_path, encoding="utf-8"))}
    eng = AH.AsrEngine(backend=args.backend)
    summary = []
    for tag, play, noise in CONDITIONS:
        rows = capture(tag, clips_path, play, noise)
        d = os.path.join(OUT, tag)
        res = []
        peaks, rmss = [], []
        for r in rows:
            p = os.path.join(d, f"{r['id']}_L.wav")
            if not os.path.exists(p):
                continue
            with wave.open(p, "rb") as w:
                n, ch = w.getnframes(), w.getnchannels()
                x = np.frombuffer(w.readframes(n), dtype="<i2").astype(np.float32) / 32768.0
            pcm = x.reshape(-1, ch)[:, 0] if ch > 1 else x
            hyp = eng.transcribe(pcm, RATE)
            ref = clips[r["id"]]["reference"]
            rr, hh = norm(ref), norm(hyp)
            dd = edit_distance(rr, hh) if rr else 0
            res.append({"id": r["id"], "reference": ref, "hyp": hyp, "dist": dd,
                        "ref_len": len(rr), "cer": (dd / len(rr)) if rr else 0.0})
            seg = pcm[int(1.5 * RATE):int(3.0 * RATE)]
            if len(seg):
                rmss.append(20 * np.log10(float(np.sqrt(np.mean(seg.astype(np.float64) ** 2))) + 1e-12))
            peaks.append(20 * np.log10(float(np.max(np.abs(pcm))) + 1e-12))
        with open(os.path.join(d, "asr.jsonl"), "w", encoding="utf-8") as f:
            for x in res:
                f.write(json.dumps(x, ensure_ascii=False) + "\n")
        dist = sum(x["dist"] for x in res)
        tot = sum(x["ref_len"] for x in res)
        entry = {"tag": tag, "play_dbfs": play, "noise_dbfs": noise,
                 "nominal_snr_db": round(play - noise, 1),
                 "clips": len(res), "cer": round(100 * dist / tot, 2) if tot else None,
                 "perfect": sum(1 for x in res if x["dist"] == 0),
                 "speech_rms_median": round(statistics.median(rmss), 1) if rmss else None,
                 "peak_max": round(max(peaks), 1) if peaks else None}
        summary.append(entry)
        print(f"  {tag:16s} play {play:6.1f} noise {noise:6.1f} -> CER {entry['cer']:6.2f}%  "
              f"perfect {entry['perfect']}/{entry['clips']}  rms {entry['speech_rms_median']} dBFS", flush=True)

    eng.close()
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print("\n=== 汇总 ===")
    print(f"{'条件':18s} {'标称SNR':>8s} {'CER':>8s} {'全对':>8s} {'语音RMS':>9s}")
    for e in summary:
        print(f"{e['tag']:18s} {e['nominal_snr_db']:7.1f} {e['cer']:7.2f}% "
              f"{e['perfect']:4d}/{e['clips']:<3d} {e['speech_rms_median']:8.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
