"""Play a reference clip through the box's artificial mouth and record stereo UAC.

Signal path:
    WAV (16/48 kHz mono) -> normalise -> 4ch @48 kHz buffer, speech on Out2
    -> US-4x4HR -> artificial mouth (5 cm) -> microphone front end
    -> ESP-NOW -> Dongle UAC -> 2ch @16 kHz

Playback is 48 kHz/4ch while the UAC microphone is 16 kHz/2ch, so playback and
recording run as two independent PortAudio streams instead of one playrec()
call (playrec forces a single samplerate).

Each clip produces <out>/<id>_L.wav and <out>/<id>_R.wav in USB channel order.
The capture.json lines include L/R levels and the caller's requested spatial
mode when supplied. Expected channel processing follows that requested mode.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
import wave

import numpy as np
import sounddevice as sd


SPATIAL_PROCESSING = {0: "spatial_bypass", 5: "wiener"}


def load_wav(path: str):
    with wave.open(path, "rb") as w:
        rate, ch, n, width = w.getframerate(), w.getnchannels(), w.getnframes(), w.getsampwidth()
        raw = w.readframes(n)
    if width != 2:
        raise SystemExit(f"only 16-bit PCM supported: {path}")
    x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    return x, rate


def resample_linear(x: np.ndarray, src: int, dst: int) -> np.ndarray:
    if src == dst:
        return x
    n_out = int(round(len(x) * dst / src))
    return np.interp(np.linspace(0.0, len(x) - 1.0, n_out),
                     np.arange(len(x)), x).astype(np.float32)


def pick_device(name: str, need_out: int, need_in: int, rate: int):
    best = None
    def _norm(s: str) -> str:
        return ''.join(ch for ch in s.lower() if ch.isalnum())

    virtual = ("primarysounddriver", "microsoftsoundmapper", "nahimic")
    aliases = [_norm(a) for a in name.split(",") if a.strip()]
    for i, d in enumerate(sd.query_devices()):
        if not any(a in _norm(d["name"]) for a in aliases):
            continue
        # Skip PortAudio pseudo-devices: they enumerate the right channel count
        # but fail to open with PaErrorCode -9996.
        if any(v in _norm(d["name"]) for v in virtual):
            continue
        if need_out and d["max_output_channels"] < need_out:
            continue
        if need_in and d["max_input_channels"] < need_in:
            continue
        score = 0 if int(d["default_samplerate"]) == rate else 1
        if best is None or score < best[0]:
            best = (score, i, d)
    return (best[1], best[2]) if best else (None, None)


class Recorder:
    def __init__(self, frames: int, channels: int):
        self.frames = frames
        self.channels = channels
        self.lock = threading.Lock()
        self.done = 0

    def callback(self, indata, frames, time_info, status):
        with self.lock:
            n = min(frames, self.frames - self.done)
            if n > 0:
                self.buf[self.done:self.done + n] = indata[:n]
                self.done += n


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--wav", nargs="*", default=[])
    ap.add_argument("--manifest", default="")
    ap.add_argument("--out", default=r"D:\App\cppApp\audio_qos\_asr_sim\capture")
    ap.add_argument("--play-rate", type=int, default=48000)
    ap.add_argument("--rec-rate", type=int, default=16000)
    ap.add_argument("--out-device", default="", help="explicit output device index or name")
    ap.add_argument("--mouth-channel", type=int, default=1, help="0-based output index (1 = Out2)")
    ap.add_argument("--noise-dbfs", type=float, default=None,
                    help="pink-noise RMS on the two box speakers (Out3/Out4); omit for quiet")
    ap.add_argument("--noise-coherent", action="store_true",
                    help="feed the same noise to both speakers (coherent field)")
    ap.add_argument("--noise-channel", choices=["left", "right", "both"], default="both",
                    help="哪一侧房间音箱播放噪声；单侧才有方向性")
    ap.add_argument("--noise-mouth", action="store_true",
                    help="mix the noise into the mouth channel instead of Out3/Out4 "
                         "(use when the box speakers are not connected)")
    ap.add_argument("--noise-seed", type=int, default=7)
    ap.add_argument("--play-dbfs", type=float, default=-20.0)
    ap.add_argument("--pre-roll", type=float, default=0.6)
    ap.add_argument("--post-roll", type=float, default=1.0)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--play-device", default="US-4x4HR",
                    help="comma separated aliases; matched after normalisation")
    ap.add_argument("--rec-device", default="WhisPrompt")
    ap.add_argument("--spatial-mode", type=int, choices=SPATIAL_PROCESSING,
                    help="label the requested Scepter mode (0 = bypass, 5 = stereo Wiener); "
                         "capture does not query or switch the device")
    args = ap.parse_args()
    expected_processing = SPATIAL_PROCESSING.get(args.spatial_mode, "unknown")

    items = []
    if args.manifest:
        base = os.path.dirname(os.path.abspath(args.manifest))
        for e in json.load(open(args.manifest, encoding="utf-8")):
            p = e["audio"]
            if not os.path.isabs(p):
                p = os.path.join(base, p.replace("/", os.sep))
            items.append({"id": e.get("id", os.path.splitext(os.path.basename(p))[0]),
                          "wav": p, "reference": e.get("reference", "")})
    for w in args.wav:
        items.append({"id": os.path.splitext(os.path.basename(w))[0], "wav": w, "reference": ""})
    if not items:
        ap.error("need --wav or --manifest")

    if args.out_device:
        out_dev = int(args.out_device) if args.out_device.isdigit() else args.out_device
        out_info = sd.query_devices(out_dev)
    else:
        out_dev, out_info = pick_device(args.play_device, 4, 0, args.play_rate)
    in_dev, in_info = pick_device(args.rec_device, 0, 2, args.rec_rate)
    if out_dev is None or in_dev is None:
        print(f"device not found: play={args.play_device} rec={args.rec_device}", file=sys.stderr)
        for i, d in enumerate(sd.query_devices()):
            print(f"  [{i}] {d['name']} in={d['max_input_channels']} "
                  f"out={d['max_output_channels']} sr={int(d['default_samplerate'])}", file=sys.stderr)
        return 2
    print(f"# play [{out_dev}] {out_info['name']} @{args.play_rate} mouth_ch={args.mouth_channel}",
          file=sys.stderr)
    print(f"# rec  [{in_dev}] {in_info['name']} @{args.rec_rate} 2ch", file=sys.stderr)

    os.makedirs(args.out, exist_ok=True)
    rows = []
    for it in items:
        dry, rate = load_wav(it["wav"])
        sp = resample_linear(dry, rate, args.play_rate)
        rms = float(np.sqrt(np.mean(sp.astype(np.float64) ** 2))) + 1e-12
        sp = sp * (10.0 ** (args.play_dbfs / 20.0) / rms)
        speech = np.concatenate([np.zeros(int(args.pre_roll * args.play_rate), dtype=np.float32),
                                 sp.astype(np.float32),
                                 np.zeros(int(args.post_roll * args.play_rate), dtype=np.float32)])
        # Optional noise bed on Out3/Out4 (0-based channels 2 and 3).
        noise = None
        if args.noise_dbfs is not None:
            rng = np.random.default_rng(args.noise_seed)
            n = len(speech)
            w = rng.standard_normal(n)
            spec = np.fft.rfft(w)
            f = np.fft.rfftfreq(n, 1.0 / args.play_rate)
            spec[1:] /= np.sqrt(np.maximum(f[1:], 1.0))
            spec[0] = 0.0
            left = np.fft.irfft(spec, n).astype(np.float32)
            left /= (np.sqrt(np.mean(left.astype(np.float64) ** 2)) + 1e-12)
            left *= 10.0 ** (args.noise_dbfs / 20.0)
            # 粉噪峰值约为 RMS 的 3-4 倍，直接按 RMS 归一化会削顶；
            # 这里按峰值再压一次，保证不削顶（实际 RMS 会略低）。
            pk = float(np.max(np.abs(left)))
            if pk > 0.95:
                left *= 0.95 / pk
            right = left.copy() if args.noise_coherent else np.roll(left[::-1], 1)
            if args.noise_mouth:
                # Same-loudspeaker mixing: noise arrives through the mouth path.
                speech = np.clip(speech + left, -1.0, 1.0).astype(np.float32)
                noise = None
            else:
                # Separated field: the noise goes to Out3/Out4 and must be played.
                noise = (left, right)
        for rep in range(args.repeat):
            total_s = len(speech) / args.play_rate
            rec_frames = int(round((total_s + 0.3) * args.rec_rate)) + args.rec_rate // 5
            rec = Recorder(rec_frames, 2)
            rec.buf = np.zeros((rec_frames, 2), dtype=np.float32)

            pos = {"i": 0}
            lock = threading.Lock()

            def out_cb(outdata, frames, time_info, status):
                with lock:
                    i = pos["i"]
                    n = min(frames, len(speech) - i)
                    if n > 0:
                        outdata[:n, :] = 0.0
                        outdata[:n, args.mouth_channel] = speech[i:i + n]
                        if noise is not None:
                            if args.noise_channel in ("left", "both"):
                                outdata[:n, 2] = noise[0][i:i + n]
                            if args.noise_channel in ("right", "both"):
                                outdata[:n, 3] = noise[1][i:i + n]
                        pos["i"] = i + n
                    if n < frames:
                        outdata[n:, :] = 0.0

            in_stream = sd.InputStream(device=in_dev, samplerate=args.rec_rate, channels=2,
                                       dtype="float32", callback=rec.callback)
            out_stream = sd.OutputStream(device=out_dev, samplerate=args.play_rate, channels=4,
                                         dtype="float32", callback=out_cb)
            in_stream.start()
            time.sleep(args.pre_roll * 0.5)
            out_stream.start()
            deadline = time.time() + total_s + 0.4
            while time.time() < deadline:
                time.sleep(0.05)
            out_stream.stop()
            out_stream.close()
            time.sleep(0.2)
            in_stream.stop()
            in_stream.close()

            captured = rec.buf[:max(rec.done, 1)]
            L = resample_linear(captured[:, 0], args.rec_rate, args.rec_rate)
            R = resample_linear(captured[:, 1], args.rec_rate, args.rec_rate)
            stats = []
            for c in (L, R):
                r = float(np.sqrt(np.mean(c.astype(np.float64) ** 2))) + 1e-12
                pk = float(np.max(np.abs(c))) + 1e-12
                tail_n = max(1, int(0.5 * args.rec_rate))
                noise_p = float(np.mean(c[-tail_n:].astype(np.float64) ** 2)) + 1e-20
                total_p = float(np.mean(c.astype(np.float64) ** 2)) + 1e-20
                sig_p = max(total_p - noise_p, 1e-20)
                stats.append({"rms_dbfs": round(20 * math.log10(r), 1),
                              "peak_dbfs": round(20 * math.log10(pk), 1),
                              "noise_rms_dbfs": round(10 * math.log10(noise_p), 1),
                              "snr_db": round(10 * math.log10(sig_p / noise_p), 1)})
            tag = it["id"] if args.repeat == 1 else f"{it['id']}_r{rep}"
            for name, chan in (("L", L), ("R", R)):
                with wave.open(os.path.join(args.out, f"{tag}_{name}.wav"), "wb") as w:
                    w.setnchannels(1)
                    w.setsampwidth(2)
                    w.setframerate(args.rec_rate)
                    w.writeframes(np.clip(chan * 32767, -32768, 32767).astype("<i2").tobytes())
            row = {"id": tag, "reference": it["reference"], "source": it["wav"],
                   "play_dbfs": args.play_dbfs, "L": stats[0], "R": stats[1],
                   "spatial_mode_requested": args.spatial_mode,
                   "expected_channel_processing": {"L": expected_processing, "R": expected_processing},
                   "dur_s": round(len(L) / args.rec_rate, 2)}
            rows.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)

    with open(os.path.join(args.out, "capture.json"), "a", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
