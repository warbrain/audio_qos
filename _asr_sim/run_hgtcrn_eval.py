"""Run the SDK's H-GTCRN dual-channel enhancement over captured box audio and
measure what it does to ASR accuracy.

H-GTCRN takes interleaved synchronous stereo (frames x 2 floats, 16 kHz),
applies WPE + AuxIVA + a masking-on-noisy network, and returns mono 16 kHz PCM.
The comparison baseline is the same captures scored on channel 0 alone.

Usage: python run_hgtcrn_eval.py [--limit 40] [--dir <capture dir>]
"""
from __future__ import annotations

import argparse
import ctypes
import json
import os
import statistics
import subprocess
import sys
import time
import unicodedata
import wave

import numpy as np

ROOT = r"D:\App\cppApp\audio_qos\_asr_sim"
PY_FULL = r"D:\App\cppApp\audio_qos\_praenv\Scripts\python.exe"
DLL = r"D:\App\cppApp\audio_qos\hmi\native-console\build\bin\Release\native\native.dll"
MODEL = r"D:\App\cppApp\audio_qos\hmi\native-console\models\frontend\hgtcrn_noisy.safetensors"
DEFAULT_DIR = r"D:\App\cppApp\audio_qos\_asr_sim\spatial_eval\m0_g722"
RATE = 16000
MIN_FRAMES = 4608

sys.path.insert(0, ROOT)
import asr_harness as AH  # noqa: E402


def bind_hgtcrn(lib):
    P, I, U32, SZ, U64, F = (ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32,
                             ctypes.c_size_t, ctypes.c_uint64, ctypes.c_float)

    class Config(ctypes.Structure):
        _fields_ = [("struct_size", U32), ("cpu_threads", U32),
                    ("model_path", ctypes.c_char_p), ("max_frames", U32),
                    ("reserved", U32)]

    sig = {
        "audiocpp_hgtcrn_create": (I, [ctypes.POINTER(Config), ctypes.POINTER(P)]),
        "audiocpp_hgtcrn_run": (I, [P, ctypes.POINTER(F), SZ, U64, ctypes.POINTER(P)]),
        "audiocpp_hgtcrn_free": (None, [P]),
        "audiocpp_result_audio": (I, [P, ctypes.POINTER(ctypes.POINTER(F)), ctypes.POINTER(SZ),
                                      ctypes.POINTER(I), ctypes.POINTER(I)]),
    }
    for name, (restype, argtypes) in sig.items():
        fn = getattr(lib, name)
        fn.restype = restype
        fn.argtypes = argtypes
    return Config


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


def read_stereo(lp: str, rp: str) -> np.ndarray:
    def rd(p):
        with wave.open(p, "rb") as w:
            n = w.getnframes()
            raw = w.readframes(n)
        # H-GTCRN expects float samples in the int16 range, not +/-1
        return np.frombuffer(raw, dtype="<i2").astype(np.float32)
    a, b = rd(lp), rd(rp)
    n = min(len(a), len(b))
    st = np.empty((n, 2), dtype=np.float32)
    st[:, 0] = a[:n]
    st[:, 1] = b[:n]
    return st


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=40)
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--backend", default="vulkan")
    ap.add_argument("--mono-dup", action="store_true",
                    help="duplicate channel 0 into both inputs to neutralise WPE/AuxIVA")
    args = ap.parse_args()

    base = os.path.dirname(args.dir.rstrip("\\/"))
    clips = json.load(open(os.path.join(os.path.dirname(base), "physical_full", "clips196.json"),
                           encoding="utf-8"))
    ok = [c for c in clips if os.path.exists(os.path.join(args.dir, f"{c['id']}_L.wav"))]
    if args.limit:
        ok = ok[: args.limit]
    print(f"clips: {len(ok)} from {args.dir}", flush=True)
    baseline = {}
    bp = os.path.join(args.dir, "asr.jsonl")
    if os.path.exists(bp):
        for line in open(bp, encoding="utf-8"):
            if line.strip():
                r = json.loads(line)
                baseline[r["id"]] = r
        print(f"baseline loaded: {len(baseline)} rows", flush=True)

    sdk = AH.Sdk()
    lib = sdk.lib
    Config = bind_hgtcrn(lib)
    cfg = Config()
    cfg.struct_size = ctypes.sizeof(Config)
    cfg.cpu_threads = 1
    cfg.model_path = MODEL.encode("utf-8")
    cfg.max_frames = 0
    cfg.reserved = 0
    handle = ctypes.c_void_p()
    sdk.check(lib.audiocpp_hgtcrn_create(ctypes.byref(cfg), ctypes.byref(handle)), "hgtcrn_create")
    print("H-GTCRN ready", flush=True)

    eng = AH.AsrEngine(backend=args.backend)
    results = []
    t0 = time.time()
    for i, c in enumerate(ok, 1):
        cid = c["id"]
        st = read_stereo(os.path.join(args.dir, f"{cid}_L.wav"),
                         os.path.join(args.dir, f"{cid}_R.wav"))
        if args.mono_dup:
            st = np.stack([st[:, 0], st[:, 0]], axis=1)
        if len(st) < MIN_FRAMES:
            continue
        frames = len(st)
        ptr = st.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        result = ctypes.c_void_p()
        sdk.check(lib.audiocpp_hgtcrn_run(handle, ptr, frames, i, ctypes.byref(result)),
                  "hgtcrn_run")
        out_ptr = ctypes.POINTER(ctypes.c_float)()
        out_frames = ctypes.c_size_t()
        out_rate = ctypes.c_int()
        out_ch = ctypes.c_int()
        try:
            sdk.check(lib.audiocpp_result_audio(result, ctypes.byref(out_ptr),
                                                ctypes.byref(out_frames),
                                                ctypes.byref(out_rate), ctypes.byref(out_ch)),
                      "result_audio")
            pcm = np.ctypeslib.as_array(out_ptr, shape=(out_frames.value,)).copy() / 32768.0
            hyp = eng.transcribe(pcm, out_rate.value)
        finally:
            lib.audiocpp_result_free(result)

        # baseline comes from the existing CUDA-scored asr.jsonl (same clip, ch0)
        base_row = baseline.get(cid)
        hyp0 = base_row["hyp"] if base_row else ""

        ref = c["reference"]
        r, h = norm(ref), norm(hyp)
        r0, h0 = norm(ref), norm(hyp0)
        d = edit_distance(r, h) if r else 0
        d0 = base_row["dist"] if base_row else (edit_distance(r0, h0) if r0 else 0)
        results.append({"id": cid, "reference": ref, "ref_len": len(r),
                        "hyp_ch0": hyp0, "dist_ch0": d0, "cer_ch0": (d0 / len(r)) if r else 0,
                        "hyp_hgtcrn": hyp, "dist_hgtcrn": d, "cer_hgtcrn": (d / len(r)) if r else 0})
        if i % 10 == 0:
            el = time.time() - t0
            print(f"  {i}/{len(ok)}  {el:.0f}s", flush=True)
    eng.close()
    lib.audiocpp_hgtcrn_free(handle)

    out_jsonl = os.path.join(base, "hgtcrn_eval.jsonl")
    with open(out_jsonl, "w", encoding="utf-8") as f:
        for r in results:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")

    def agg(key):
        d = sum(r[f"dist_{key}"] for r in results)
        t = sum(r["ref_len"] for r in results)
        p = sum(1 for r in results if r[f"dist_{key}"] == 0)
        per = [r[f"cer_{key}"] * 100 for r in results]
        return 100 * d / t, p, statistics.mean(per)

    c0, p0, m0 = agg("ch0")
    c1, p1, m1 = agg("hgtcrn")
    print()
    print(f"{'路径':22s} {'字符CER':>9s} {'整句全对':>9s} {'逐条均值':>9s}")
    print(f"  原始通道 ch0            {c0:8.2f}% {p0:5d}/{len(results):<3d} {m0:8.2f}%")
    print(f"  H-GTCRN 增强后          {c1:8.2f}% {p1:5d}/{len(results):<3d} {m1:8.2f}%")
    print(f"  差值（增强 − 原始）      {c1-c0:+8.2f} pp   {p1-p0:+d} 条")
    with open(os.path.join(base, "hgtcrn_summary.json"), "w", encoding="utf-8") as f:
        json.dump({"clips": len(results), "dir": args.dir,
                   "ch0": {"cer": round(c0, 3), "perfect": p0},
                   "hgtcrn": {"cer": round(c1, 3), "perfect": p1},
                   "delta_pp": round(c1 - c0, 3)}, f, ensure_ascii=False, indent=2)
    print(f"\nwrote {out_jsonl}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
