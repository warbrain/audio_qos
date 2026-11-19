"""Single-microphone 1 m^3 box simulation + ASR scoring on the strict-v5 test set.

Renders the 196 scored clips of heldout-manifest.strict-v5.json through a
pyroomacoustics shoebox (1x1x1 m, one microphone, artificial mouth 5 cm away,
two noise speakers), then transcribes each render through native.dll via
asr_harness and reports CER per condition.

Conditions (dry reference + three box settings):
    clean        no room, no noise          (ceiling)
    t010_snr20   T60 0.10 s, SNR +20 dB
    t030_snr10   T60 0.30 s, SNR +10 dB
    t060_snr00   T60 0.60 s, SNR +0 dB
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import sys
import time
import unicodedata

import numpy as np
import pyroomacoustics as pra
from scipy.signal import resample_poly

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import asr_harness as AH  # noqa: E402

FS = 48000
FS_OUT = 16000
ROOM = np.array([1.0, 1.0, 1.0])
MIC = np.array([0.5, 0.5, 0.5])
MOUTH_D = 0.05
SPK = (np.array([0.15, 0.75, 0.5]), np.array([0.85, 0.75, 0.5]))
TARGET_RMS_DBFS = -26.0
GAIN_CAP_DB = 55.0

CONDITIONS = [
    ("clean", None, None),
    ("t010_snr20", 0.10, 20.0),
    ("t030_snr10", 0.30, 10.0),
    ("t060_snr00", 0.60, 0.0),
]


def build_room(t60: float, cap: int = 60, mouth_p: float | None = 0.5,
               mouth_dist: float = 0.10):
    """1 m^3 shoebox, one microphone, artificial mouth `mouth_dist` ahead.

    mouth_p is the cardioid-family parameter for the mouth simulator:
    0 figure-eight, 0.25 hyper-cardioid, 0.5 cardioid, 0.75 sub-cardioid,
    1 / None omnidirectional (a bare point source, i.e. not a mouth simulator).
    The mouth faces the microphone: its axis is (0, -1, 0).
    """
    from pyroomacoustics.directivities import CardioidFamily, DirectionVector
    absorption, order = pra.inverse_sabine(t60, ROOM)
    room = pra.ShoeBox(ROOM, fs=FS, materials=pra.Material(absorption),
                       max_order=min(order, cap), air_absorption=True)
    room.add_microphone(MIC)
    mouth = MIC + np.array([0.0, mouth_dist, 0.0])
    if mouth_p is None:
        room.add_source(mouth)
    else:
        # -y axis: azimuth -90 deg, colatitude 90 deg, pointing at the microphone.
        aim = DirectionVector(azimuth=-90.0, colatitude=90.0, degrees=True)
        room.add_source(mouth, directivity=CardioidFamily(orientation=aim, p=mouth_p))
    room.add_source(SPK[0])
    room.add_source(SPK[1])
    room.compute_rir()
    return room, absorption, min(order, cap)


def read_mono48(path: str) -> np.ndarray:
    import wave
    with wave.open(path, "rb") as w:
        rate, ch, n = w.getframerate(), w.getnchannels(), w.getnframes()
        raw = w.readframes(n)
    if w.getsampwidth() != 2:
        raise RuntimeError(f"expect 16-bit PCM: {path}")
    x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    if rate != FS:
        g = math.gcd(int(rate), FS)
        x = resample_poly(x, FS // g, int(rate) // g).astype(np.float32)
    return x


def pink(n: int, rng) -> np.ndarray:
    w = rng.standard_normal(n)
    X = np.fft.rfft(w)
    f = np.fft.rfftfreq(n, 1.0 / FS)
    X[1:] /= np.sqrt(np.maximum(f[1:], 1.0))
    X[0] = 0.0
    x = np.fft.irfft(X, n)
    return (x / (np.max(np.abs(x)) + 1e-9)).astype(np.float32)


def normalize(x: np.ndarray) -> np.ndarray:
    rms = float(np.sqrt(np.mean(x.astype(np.float64) ** 2)))
    if rms <= 0:
        return x
    gain_db = min(TARGET_RMS_DBFS - 20.0 * math.log10(rms), GAIN_CAP_DB)
    y = x * (10.0 ** (gain_db / 20.0))
    peak = float(np.max(np.abs(y)))
    if peak > 0.95:
        y = y * (0.95 / peak)
    return y.astype(np.float32)


def render(dry: np.ndarray, room, t60, snr_db, seed: int) -> np.ndarray:
    """Convolve with the room RIR, mix pink noise at snr_db, return 16 kHz."""
    n_len = len(dry) + FS // 2

    def fit(x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float32)
        return x[:n_len] if len(x) >= n_len else np.pad(x, (0, n_len - len(x)))

    if room is None:
        speech = fit(dry)
    else:
        speech = fit(np.convolve(dry, room.rir[0][0], mode="full"))
    speech = normalize(speech)
    if room is None or snr_db is None:
        mix = speech
    else:
        rng = np.random.default_rng(seed)
        nz = fit(np.convolve(pink(n_len, rng), room.rir[0][1], mode="full"))
        nz = nz + fit(np.convolve(pink(n_len, rng), room.rir[0][2], mode="full"))
        srms = float(np.sqrt(np.mean(speech.astype(np.float64) ** 2)))
        nrms = float(np.sqrt(np.mean(nz.astype(np.float64) ** 2))) + 1e-12
        nz = nz * (srms / (10.0 ** (snr_db / 20.0)) / nrms)
        mix = speech + nz
        peak = float(np.max(np.abs(mix)))
        if peak > 0.95:
            mix = mix * (0.95 / peak)
    return resample_poly(mix.astype(np.float32), FS_OUT, FS).astype(np.float32)


def norm_text(s: str) -> str:
    s = unicodedata.normalize("NFKC", s or "").lower()
    return "".join(ch for ch in s if unicodedata.category(ch)[0] not in ("P", "S", "Z", "C"))


def edit_distance(a: str, b: str) -> int:
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", default=r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5")
    ap.add_argument("--manifest", default="heldout-manifest.strict-v5.json")
    ap.add_argument("--out", default=r"D:\App\cppApp\audio_qos\_asr_sim\testset")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--backend", default="cuda")
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--language", default="")
    ap.add_argument("--conditions", default="")  # comma separated subset
    ap.add_argument("--t60s", default="", help="comma separated T60 grid, e.g. 0.1,0.3,0.6")
    ap.add_argument("--snrs", default="", help="comma separated SNR grid, e.g. 20,10,0")
    ap.add_argument("--mouth-p", type=float, default=0.5,
                    help="mouth-simulator cardioid parameter (0.25 hyper, 0.5 cardioid, "
                         "0.75 sub, >=1 or -1 for omnidirectional point source)")
    ap.add_argument("--mouth-cm", type=float, default=10.0,
                    help="distance from the artificial mouth to the microphone in cm")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    root = args.dataset
    mf = json.load(open(os.path.join(root, args.manifest), encoding="utf-8"))
    clips = mf["recordings"]
    if args.limit:
        clips = clips[: args.limit]
    wanted = [c.strip() for c in args.conditions.split(",") if c.strip()]
    if args.t60s:
        t60s = [float(x) for x in args.t60s.split(",") if x.strip()]
        snrs = [float(x) for x in args.snrs.split(",") if x.strip()] if args.snrs else []
        conds = [("clean", None, None)]
        for t in t60s:
            for s in snrs:
                conds.append((f"t{int(round(t*100)):03d}_snr{int(s):+03d}", t, s))
    else:
        conds = [c for c in CONDITIONS if not wanted or c[0] in wanted]
    print(f"# clips={len(clips)} conditions={[c[0] for c in conds]}", file=sys.stderr)

    mouth_p = None if args.mouth_p is None or args.mouth_p >= 1.0 or args.mouth_p < 0 else args.mouth_p
    print(f"# mouth directivity: p={mouth_p} "
          f"({'omnidirectional point source' if mouth_p is None else 'cardioid family, aim -y'}) "
          f"distance={args.mouth_cm:g} cm", file=sys.stderr)
    rooms = {}
    for name, t60, snr in conds:
        if t60 is None or t60 in rooms:
            continue
        t0 = time.time()
        room, ab, order = build_room(t60, mouth_p=mouth_p,
                                     mouth_dist=args.mouth_cm / 100.0)
        rooms[t60] = room
        print(f"# room T60={t60}: absorption={ab:.4f} order={order} "
              f"build={time.time()-t0:.1f}s", file=sys.stderr)

    t0 = time.time()
    eng = AH.AsrEngine(backend=args.backend, threads=args.threads,
                       language=args.language or None)
    print(f"# engine ready load={time.time()-t0:.1f}s", file=sys.stderr)

    rows = []
    t_start = time.time()
    for i, c in enumerate(clips, 1):
        path = os.path.join(root, c["audio"].replace("/", os.sep))
        try:
            dry = read_mono48(path)
        except Exception as exc:
            print(f"# skip {c['audio']}: {exc}", file=sys.stderr)
            continue
        for name, t60, snr in conds:
            pcm = render(dry, rooms.get(t60) if t60 else None, t60, snr, seed=1000 + i)
            t1 = time.time()
            hyp = eng.transcribe(pcm)
            dt = time.time() - t1
            ref = c["reference"]
            r, h = norm_text(ref), norm_text(hyp)
            d = edit_distance(r, h) if r else (0 if not h else 1)
            rows.append({
                "id": c["audio"].split("/")[-1].rsplit(".", 1)[0],
                "speaker": c.get("speaker", ""),
                "condition": name, "t60": t60, "snr_db": snr,
                "audio_s": round(len(pcm) / FS_OUT, 2),
                "reference": ref, "hypothesis": hyp,
                "cer": round((d / len(r)) if r else (0.0 if not h else 1.0), 6),
                "dist": d, "ref_len": len(r),
                "asr_ms": round(dt * 1000, 1),
                "rtf": round(dt / (len(pcm) / FS_OUT), 3) if pcm.size else 0.0,
            })
        if i % 20 == 0 or i == len(clips):
            el = time.time() - t_start
            print(f"# {i}/{len(clips)} elapsed={el:.0f}s eta={el/i*(len(clips)-i):.0f}s",
                  file=sys.stderr)

    eng.close()
    out_jsonl = os.path.join(args.out, "results.jsonl")
    with open(out_jsonl, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")

    agg = collections.defaultdict(lambda: [0, 0, 0, 0.0, 0])
    for r in rows:
        a = agg[r["condition"]]
        a[0] += r["dist"]; a[1] += r["ref_len"]; a[2] += 1
        a[3] += r["rtf"]; a[4] += (r["dist"] == 0)
    summary = []
    print(f"\n{'condition':14s} {'clips':>6s} {'CER':>8s} {'完全正确':>9s} {'RTF':>6s}")
    print("-" * 50)
    for name, _, _ in conds:
        d, n, c, rtf, perfect = agg[name]
        ce = 100.0 * d / n if n else 0.0
        summary.append({"condition": name, "clips": c, "cer": round(ce, 3),
                        "perfect": perfect, "dist": d, "ref_len": n,
                        "rtf": round(rtf / c, 3) if c else 0.0})
        print(f"{name:14s} {c:6d} {ce:7.2f}% {perfect:6d}/{c:<3d} {rtf/c if c else 0:6.3f}")
    td = sum(s["dist"] for s in summary); tn = sum(s["ref_len"] for s in summary)
    print(f"\n总体 CER = {100*td/tn:.2f}%  ({td}/{tn} 字)")
    with open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"clips": len(clips), "total_cer": round(100 * td / tn, 3),
                   "conditions": summary}, f, ensure_ascii=False, indent=2)
    print(f"# wrote {out_jsonl}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
