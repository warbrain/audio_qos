"""Drive the 40-clip physical evaluation: capture at several noise levels, then
score both UAC channels with ASR.

Runs box_capture.py and score_capture.py as subprocesses (list args, no shell),
so argument quoting and encoding never go through PowerShell.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

PY = r"D:\App\cppApp\audio_qos\_praenv\Scripts\python.exe"
ROOT = r"D:\App\cppApp\audio_qos\_asr_sim"
OUT = os.path.join(ROOT, "physical_eval")
DATASET = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5"
N_CLIPS = 40

CONDITIONS = [
    ("quiet", []),
    ("snr10", ["--noise-dbfs", "-40", "--noise-mouth"]),
    ("snr00", ["--noise-dbfs", "-30", "--noise-mouth"]),
]


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
    print(f"manifest: {len(items)} clips -> {path}")
    return path


def run(args: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(args, capture_output=True, text=True, encoding="utf-8", errors="replace")


def main() -> int:
    clips = build_manifest()
    log_path = os.path.join(OUT, "results.txt")
    lines = []
    for name, extra in CONDITIONS:
        d = os.path.join(OUT, name)
        os.makedirs(d, exist_ok=True)
        print(f"=== capture {name} ===", flush=True)
        cap = run([PY, os.path.join(ROOT, "box_capture.py"),
                   "--manifest", clips, "--play-dbfs", "-30", "--out", d] + extra)
        rows = [l for l in cap.stdout.splitlines() if l.startswith("{")]
        print(f"    captured {len(rows)} rows (exit {cap.returncode})", flush=True)
        if not rows:
            print(cap.stderr[-800:], flush=True)
            continue
        with open(os.path.join(d, "capture.json"), "w", encoding="utf-8") as f:
            for r in rows:
                f.write(r + "\n")
        print(f"=== score {name} ===", flush=True)
        sc = run([PY, os.path.join(ROOT, "score_capture.py"), "--dir", d, "--clips", clips])
        with open(os.path.join(d, "score.txt"), "w", encoding="utf-8") as f:
            f.write(sc.stdout)
        summary = [l for l in sc.stdout.splitlines() if l.startswith(("降噪 L", "原始 R"))]
        lines.append(f"--- {name} (n={len(rows)}) ---")
        lines.extend(summary)
        for s in summary:
            print("   ", s, flush=True)
    with open(log_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\nwrote {log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
