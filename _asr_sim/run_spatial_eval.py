"""Compare the dual-microphone firmware's spatial modes on real hardware.

For each mode it sets the mode over HID, captures N clips through the box, then
transcribes channel 0 with Qwen3-ASR and scores CER.

HID read-back is unreliable on this Windows host (the HID class driver returns
the cached SET payload instead of the device reply), so each mode switch is
verified behaviourally instead: the firmware's dual-mono modes emit identical
left/right channels while the stereo modes do not.

Usage: python run_spatial_eval.py [--clips 24] [--modes 0,5]
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import unicodedata
import wave

import numpy as np

ROOT = r"D:\App\cppApp\audio_qos\_asr_sim"
OUT = os.path.join(ROOT, "spatial_eval")
PY_FULL = r"D:\App\cppApp\audio_qos\_praenv\Scripts\python.exe"
PY_HID = r"D:\App\OTA\.venv\Scripts\python.exe"
DATASET = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5"
RATE = 16000

# The firmware's layout table: these modes fuse to dual mono, the rest stay stereo.
DUAL_MONO = {1, 2, 4, 6}


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


def build_manifest(n: int, start: int = 0) -> str:
    os.makedirs(OUT, exist_ok=True)
    mf = json.load(open(os.path.join(DATASET, "heldout-manifest.strict-v5.json"), encoding="utf-8"))
    items = []
    for c in mf["recordings"][start:start + n]:
        p = os.path.join(DATASET, c["audio"].replace("/", os.sep))
        items.append({"id": os.path.splitext(os.path.basename(p))[0],
                      "audio": os.path.join(DATASET, c["audio"].replace("/", os.sep)),
                      "speaker": c.get("speaker", ""), "reference": c["reference"]})
    path = os.path.join(OUT, f"clips{n}" + (f"_from{start}" if start else "") + ".json")
    json.dump(items, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    return path


def set_mode(mode: int) -> str:
    r = subprocess.run([PY_HID, os.path.join(ROOT, "set_spatial.py"), "--mode", str(mode)],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    return (r.stdout or "").strip().splitlines()[0] if r.stdout else ""


def capture(mode: int, clips_path: str, extra: list[str], tag: str = "") -> list[dict]:
    name = f"m{mode}" + (f"_{tag}" if tag else "")
    d = os.path.join(OUT, name)
    os.makedirs(d, exist_ok=True)
    proc = subprocess.Popen([PY_FULL, os.path.join(ROOT, "box_capture.py"),
                             "--manifest", clips_path, "--play-dbfs", "-30",
                             "--spatial-mode", str(mode), "--out", d] + extra,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, encoding="utf-8", errors="replace", bufsize=1)
    rows = []
    for line in proc.stdout:
        if line.startswith("{"):
            rows.append(json.loads(line))
            if len(rows) % 20 == 0:
                print(f"    mode {mode}: captured {len(rows)}", flush=True)
    proc.wait()
    with open(os.path.join(d, "capture.json"), "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    return rows


def channel_identity(d: str) -> tuple[float, float]:
    """Return (correlation, level difference dB) between the first L/R pair."""
    ls = sorted(f for f in os.listdir(d) if f.endswith("_L.wav"))
    if not ls:
        return 0.0, 0.0
    def rd(p):
        with wave.open(p, "rb") as w:
            n, ch = w.getnframes(), w.getnchannels()
            raw = w.readframes(n)
        x = np.frombuffer(raw, dtype="<i2")
        return x.reshape(-1, ch)[:, 0].astype(np.float64) if ch > 1 else x.astype(np.float64)
    a = rd(os.path.join(d, ls[0]))
    b = rd(os.path.join(d, ls[0].replace("_L.wav", "_R.wav")))
    n = min(len(a), len(b))
    a, b = a[:n], b[:n]
    corr = float(np.corrcoef(a, b)[0, 1]) if np.std(a) > 0 and np.std(b) > 0 else 1.0
    la = 20 * np.log10(np.sqrt(np.mean(a ** 2)) + 1e-9)
    lb = 20 * np.log10(np.sqrt(np.mean(b ** 2)) + 1e-9)
    return corr, la - lb


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--start", type=int, default=0, help="first manifest index to use")
    ap.add_argument("--modes", default="0,5",
                    help="firmware 0.5.27 exposes only 0 (bypass) and 5 (stereo Wiener)")
    ap.add_argument("--noise-dbfs", type=float, default=None,
                    help="pink-noise level mixed into the mouth channel (speech is -30 dBFS)")
    ap.add_argument("--noise-mouth", action="store_true",
                    help="mix the noise into the mouth channel (box speakers unavailable)")
    ap.add_argument("--tag", default="", help="suffix for the output directory, e.g. snr00")
    ap.add_argument("--backend", default="vulkan",
                    help="ASR backend; the cuda build is blocked by Smart App Control here")
    ap.add_argument("--codec", type=int, default=None,
                    help="4=G.722 only; Opus (3/5) was removed in firmware 0.5.27")
    args = ap.parse_args()
    modes = [int(x) for x in args.modes.split(",") if x.strip()]
    removed = [m for m in modes if m not in (0, 5)]
    if removed:
        raise SystemExit(
            f"spatial mode(s) {removed} were removed in firmware 0.5.27; "
            f"supported modes are 0 (bypass) and 5 (stereo Wiener)"
        )

    extra: list[str] = []
    if args.noise_dbfs is not None:
        extra += ["--noise-dbfs", str(args.noise_dbfs)]
        if args.noise_mouth:
            extra += ["--noise-mouth"]

    sys.path.insert(0, ROOT)
    import asr_harness as AH

    if args.codec is not None:
        r = subprocess.run([PY_HID, os.path.join(ROOT, "set_spatial.py"), "--codec", str(args.codec)],
                           capture_output=True, text=True, encoding="utf-8", errors="replace")
        print((r.stdout or "").strip().splitlines()[0] if r.stdout else "codec set failed", flush=True)

    clips_path = build_manifest(args.clips, args.start)
    clips = {e["id"]: e for e in json.load(open(clips_path, encoding="utf-8"))}
    eng = AH.AsrEngine(backend=args.backend)

    summary = []
    for mode in modes:
        info = set_mode(mode)
        rows = capture(mode, clips_path, extra, args.tag)
        d = os.path.join(OUT, f"m{mode}" + (f"_{args.tag}" if args.tag else ""))
        corr, ldiff = channel_identity(d)
        expect_mono = mode in DUAL_MONO
        looks_mono = corr > 0.9999
        verdict = "ok" if looks_mono == expect_mono else "UNEXPECTED"
        print(f"mode {mode}: captured {len(rows)}, L/R corr={corr:.4f} "
              f"(expect {'dual-mono' if expect_mono else 'stereo'}) -> {verdict}", flush=True)

        results = []
        for rid in [r["id"] for r in rows]:
            p = os.path.join(d, f"{rid}_L.wav")
            if not os.path.exists(p):
                continue
            with wave.open(p, "rb") as w:
                n, ch = w.getnframes(), w.getnchannels()
                raw = w.readframes(n)
            x = np.frombuffer(raw, dtype="<i2")
            pcm = (x.reshape(-1, ch)[:, 0] if ch > 1 else x).astype(np.float32) / 32768.0
            hyp = eng.transcribe(pcm, RATE)
            ref = clips[rid]["reference"]
            r, h = norm(ref), norm(hyp)
            dist = edit_distance(r, h) if r else 0
            results.append({"id": rid, "reference": ref, "hyp": hyp, "dist": dist,
                            "ref_len": len(r), "cer": (dist / len(r)) if r else 0.0})
        with open(os.path.join(d, "asr.jsonl"), "w", encoding="utf-8") as f:
            for r in results:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
        dist = sum(r["dist"] for r in results)
        tot = sum(r["ref_len"] for r in results)
        perfect = sum(1 for r in results if r["dist"] == 0)
        entry = {"mode": mode, "clips": len(results),
                 "cer": round(100 * dist / tot, 3) if tot else 0.0,
                 "perfect": perfect, "dist": dist, "ref_len": tot,
                 "lr_corr": round(corr, 5), "lr_level_diff_db": round(ldiff, 2),
                 "layout_check": verdict}
        summary.append(entry)
        print(f"    mode {mode}: CER {entry['cer']:.2f}%  perfect {perfect}/{len(results)}", flush=True)

    summary_name = "summary.json" if not args.tag else f"summary_{args.tag}.json"
    with open(os.path.join(OUT, summary_name), "w", encoding="utf-8") as f:
        json.dump({"clips": args.clips, "tag": args.tag,
                   "noise_dbfs": args.noise_dbfs, "results": summary},
                  f, ensure_ascii=False, indent=2)
    eng.close()
    print("\n=== SUMMARY ===")
    print(f"{'mode':>4s} {'clips':>6s} {'CER':>8s} {'perfect':>9s} {'layout':>10s}")
    for e in summary:
        print(f"{e['mode']:4d} {e['clips']:6d} {e['cer']:7.2f}% {e['perfect']:5d}/{e['clips']:<3d} {e['layout_check']:>10s}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())



