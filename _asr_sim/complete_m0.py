"""Complete the bypass-mode G.722 capture to all 196 clips, then score the whole
directory in one pass.

Usage: python complete_m0.py
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import unicodedata
import wave

import numpy as np

ROOT = r"D:\App\cppApp\audio_qos\_asr_sim"
OUT = os.path.join(ROOT, "spatial_eval")
DEST = os.path.join(OUT, "m0_g722")
PY_FULL = r"D:\App\cppApp\audio_qos\_praenv\Scripts\python.exe"
PY_HID = r"D:\App\OTA\.venv\Scripts\python.exe"
RATE = 16000
MANIFEST = os.path.join(ROOT, "physical_full", "clips196.json")


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


def have_ids() -> set[str]:
    return {f[:-6] for f in os.listdir(DEST) if f.endswith("_L.wav")} if os.path.isdir(DEST) else set()


def capture_range(man: list[dict], start: int, count: int) -> None:
    slice_items = man[start:start + count]
    path = os.path.join(OUT, f"slice_{start}_{count}.json")
    json.dump(slice_items, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"=== capture index {start}-{start+count-1} ({count} clips) ===", flush=True)
    proc = subprocess.Popen([PY_FULL, os.path.join(ROOT, "box_capture.py"),
                             "--manifest", path, "--play-dbfs", "-30",
                             "--noise-dbfs", "-30", "--noise-mouth", "--out", DEST],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, encoding="utf-8", errors="replace", bufsize=1)
    n = 0
    for line in proc.stdout:
        if line.startswith("{"):
            n += 1
            if n % 20 == 0:
                print(f"    captured {n}/{count}", flush=True)
    proc.wait()
    print(f"    range done: {n} rows", flush=True)


def main() -> int:
    sys.path.insert(0, ROOT)
    import asr_harness as AH

    man = json.load(open(MANIFEST, encoding="utf-8"))
    have = have_ids()
    missing = [i for i, e in enumerate(man) if e["id"] not in have]
    print(f"total {len(man)}, have {len(have)}, missing {len(missing)}", flush=True)

    ranges, s, p = [], None, None
    for i in missing:
        if s is None:
            s = p = i
        elif i == p + 1:
            p = i
        else:
            ranges.append((s, p))
            s = p = i
    if s is not None:
        ranges.append((s, p))
    print("ranges: " + ", ".join(f"{a}-{b}" for a, b in ranges), flush=True)

    for cmd in (["--mode", "0"], ["--codec", "4"]):
        r = subprocess.run([PY_HID, os.path.join(ROOT, "set_spatial.py")] + cmd,
                           capture_output=True, text=True, encoding="utf-8", errors="replace")
        first = (r.stdout or "").strip().splitlines()
        print("set: " + (first[0] if first else "failed"), flush=True)

    for a, b in ranges:
        capture_range(man, a, b - a + 1)

    print("=== scoring all clips in the directory ===", flush=True)
    eng = AH.AsrEngine()
    results = []
    for e in man:
        p = os.path.join(DEST, f"{e['id']}_L.wav")
        if not os.path.exists(p):
            print(f"  MISSING {e['id']}", flush=True)
            continue
        with wave.open(p, "rb") as w:
            n, ch = w.getnframes(), w.getnchannels()
            raw = w.readframes(n)
        x = np.frombuffer(raw, dtype="<i2")
        pcm = (x.reshape(-1, ch)[:, 0] if ch > 1 else x).astype(np.float32) / 32768.0
        hyp = eng.transcribe(pcm, RATE)
        r, h = norm(e["reference"]), norm(hyp)
        dist = edit_distance(r, h) if r else 0
        results.append({"id": e["id"], "speaker": e.get("speaker", ""), "reference": e["reference"],
                        "hyp": hyp, "dist": dist, "ref_len": len(r),
                        "cer": (dist / len(r)) if r else 0.0})
        if len(results) % 20 == 0:
            print(f"    scored {len(results)}", flush=True)
    eng.close()

    with open(os.path.join(DEST, "asr.jsonl"), "w", encoding="utf-8") as f:
        for r in results:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")

    d = sum(r["dist"] for r in results)
    t = sum(r["ref_len"] for r in results)
    perfect = sum(1 for r in results if r["dist"] == 0)
    per = [r["cer"] * 100 for r in results]
    summary = {"mode": 0, "mode_name": "bypass", "codec": 4, "codec_name": "G.722",
               "noise_dbfs": -30, "clips": len(results),
               "cer_char": round(100 * d / t, 3) if t else 0.0,
               "cer_mean": round(sum(per) / len(per), 3) if per else 0.0,
               "perfect": perfect, "dist": d, "ref_len": t}
    json.dump(summary, open(os.path.join(DEST, "summary.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
    print("\n=== DONE ===", flush=True)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
