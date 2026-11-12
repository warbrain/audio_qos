#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""asr_cer_eval.py — A/B speech enhancement by ASR character error rate.

One table over every (spatial mode × fixed gain) condition:

    16 kHz WAV -> win_spatial_wav.exe (production firmware DSP, per condition)
               -> raw float32 mono -> hmi_voice_probe.exe (Qwen3-ASR) -> text
               -> CER vs reference, S/D/I, clip ratio, paired bootstrap.

The probe only accepts raw float32 mono 16 kHz inside the project directory, so
intermediate PCM is written under <project>/build/tmp/asr-eval.

Example:
    python asr_cer_eval.py --manifest manifest.tsv ^
        --project D:\\App\\cppApp\\project ^
        --modes 0,2,4 --gains 1,4,16 --level-normalize input --out results
"""
from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import json
import os
import re
import struct
import subprocess
import threading
import sys
import time
import unicodedata
import wave
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

PROBE_RE = re.compile(
    r"^PROBE (PASS|FAIL) iteration=\d+ total_ms=([\d.]+) finish_ms=([\d.]+) "
    r"confidence=\S+ text=(.*)$"
)
SAMPLE_RATE = 16000
MAX_PROBE_SAMPLES = 480000  # 30 s, enforced by hmi_voice_probe
MAX_LEVEL_GAIN_DB = 30.0
DUAL_END_FRAME_SAMPLES = 320  # 20 ms at 16 kHz, the firmware audio frame
NOISE_PATTERN_RE = re.compile(r"(?:^|_)s\d+_(white|shape|babble)_snr([+-]?\d+)(?:_|$)")
SPATIAL_MODE_LABELS = {
    0: "Bypass",
    1: "DAS",
    2: "NLMS + DAS",
    3: "Independent stereo NLMS",
    4: "DAS + Wiener",
    5: "Independent stereo Wiener",
    6: "Dual-channel soft-mask MVDR",
}


# ---------------------------------------------------------------------------
# manifest / io
# ---------------------------------------------------------------------------
@dataclass
class Item:
    uid: str
    wav: Path
    ref: str


@dataclass(frozen=True)
class Condition:
    label: str
    mode: int
    gain: float
    drop: float = 0.0
    conceal: str = "repeat"
    codec_sim: str = "none"


CODEC_SIM_TAGS = {"none": "pcm", "g722": "g722", "opus32": "o32", "opus62": "o62"}


def apply_codec_sim(pcm: np.ndarray, kind: str, ffmpeg: Path, workdir: Path) -> np.ndarray:
    """Round-trip the PCM through a real codec with ffmpeg.

    g722    -> 64 kbps ADPCM, the firmware default.
    opus32  -> libopus CBR 32 kbps, 20 ms, lowdelay (matches codec 3).
    opus62  -> libopus CBR 62 kbps, 20 ms, lowdelay (matches codec 5).
    """
    if kind == "none":
        return pcm
    if not ffmpeg.is_file():
        raise SystemExit(f"ffmpeg not found: {ffmpeg} (needed by --codec-sims {kind})")
    workdir.mkdir(parents=True, exist_ok=True)
    raw_in = workdir / "codec_in.raw"
    raw_out = workdir / "codec_out.raw"
    payload = np.clip(pcm, -1.0, 1.0)
    raw_in.write_bytes((payload * 32767.0).astype("<i2").tobytes())
    base = [str(ffmpeg), "-hide_banner", "-loglevel", "error", "-y"]
    if kind == "g722":
        enc = base + ["-f", "s16le", "-ar", "16000", "-ac", "1", "-i", str(raw_in),
                      "-c:a", "g722", "-f", "g722", str(workdir / "codec.g722")]
        dec = base + ["-f", "g722", "-i", str(workdir / "codec.g722"),
                      "-f", "s16le", "-ar", "16000", "-ac", "1", str(raw_out)]
    elif kind in ("opus32", "opus62"):
        bitrate = "32k" if kind == "opus32" else "62k"
        enc = base + ["-f", "s16le", "-ar", "16000", "-ac", "1", "-i", str(raw_in),
                      "-c:a", "libopus", "-b:a", bitrate, "-vbr", "off",
                      "-application", "lowdelay", "-frame_duration", "20",
                      str(workdir / "codec.opus")]
        dec = base + ["-i", str(workdir / "codec.opus"),
                      "-f", "s16le", "-ar", "16000", "-ac", "1", str(raw_out)]
    else:
        raise SystemExit(f"unknown --codec-sims value: {kind}")
    for cmd in (enc, dec):
        proc = subprocess.run(cmd, capture_output=True, timeout=120)
        if proc.returncode != 0:
            raise SystemExit(f"ffmpeg {kind} failed: {proc.stderr.decode('utf-8', 'replace')[-400:]}")
    out = np.frombuffer(raw_out.read_bytes(), dtype="<i2").astype(np.float64) / 32768.0
    # Codec delay can change length by a few samples; keep the ASR input aligned.
    if out.size >= pcm.size:
        return out[: pcm.size]
    return np.pad(out, (0, pcm.size - out.size))


def load_manifest(path: Path) -> list[Item]:
    raw = path.read_text(encoding="utf-8-sig").splitlines()
    rows = [line for line in raw if line.strip() and not line.lstrip().startswith("#")]
    if not rows:
        raise SystemExit(f"empty manifest: {path}")
    delim = "\t" if "\t" in rows[0] else ","
    reader = csv.DictReader(rows, delimiter=delim)
    header = {(f or "").strip().lower() for f in reader.fieldnames or []}
    if not {"id", "wav", "ref"}.issubset(header):
        raise SystemExit(f"manifest needs columns id/wav/ref, got {reader.fieldnames}")
    items: list[Item] = []
    for i, row in enumerate(reader, 2):
        row = {(k or "").strip().lower(): (v or "").strip() for k, v in row.items()}
        wav = Path(row["wav"])
        if not wav.is_absolute():
            wav = (path.parent / wav).resolve()
        if not wav.is_file():
            raise SystemExit(f"line {i}: wav not found: {wav}")
        if not row["ref"]:
            raise SystemExit(f"line {i}: empty reference text")
        items.append(Item(row["id"] or wav.stem, wav, row["ref"]))
    return items


def read_wav_channels16k(path: Path) -> np.ndarray:
    """Read 16-bit 16 kHz PCM as frames x channels without losing spatial data."""
    import wave

    with wave.open(str(path), "rb") as w:
        ch, width, rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if width != 2:
        raise SystemExit(f"{path}: only 16-bit PCM WAV is supported (got {width * 8}-bit)")
    if rate != SAMPLE_RATE:
        raise SystemExit(f"{path}: expected 16 kHz, got {rate} Hz")
    x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    return x.reshape(-1, ch)


def downmix_channels(channels: np.ndarray, downmix: str = "mean") -> np.ndarray:
    """Apply an explicit mono rule after spatial DSP, or for level analysis."""
    if channels.ndim != 2 or channels.shape[1] == 1:
        return channels.reshape(-1)
    if downmix == "left":
        return channels[:, 0]
    if downmix == "right":
        return channels[:, 1]
    if downmix == "best":
        # Pick the healthier speech channel: lowest clipping first, then RMS.
        def score(c: int) -> tuple[bool, float]:
            track = channels[:, c]
            clip = float(np.mean(np.abs(track) >= 0.999))
            return (clip < 0.001, float(np.sqrt(np.mean(track ** 2))))
        return channels[:, max(range(channels.shape[1]), key=score)]
    return channels.mean(axis=1)


def read_wav_mono16k(path: Path, downmix: str = "mean") -> np.ndarray:
    """16-bit PCM WAV -> float64 mono; spatial modes receive the original WAV."""
    return downmix_channels(read_wav_channels16k(path), downmix)


def apply_frame_drops(x: np.ndarray, drop_rate: float, conceal: str,
                      seed: int) -> tuple[np.ndarray, float]:
    """Simulate 20 ms frame loss at the decoder output.

    zero   -> pre-0.5.19 behaviour: a silent hole.
    repeat -> the P0-2 concealment: replay the last good frame under a
              one-frame linear fade (gain frame_len..1).
    The same seed drops the same frames in both modes, keeping the A/B paired.
    """
    frame = DUAL_END_FRAME_SAMPLES
    frames = x.size // frame
    if drop_rate <= 0.0 or frames == 0 or x.size == 0:
        return x, 0.0
    rng = np.random.default_rng(seed)
    dropped = rng.random(frames) < drop_rate
    out = x.copy()
    last_good = np.zeros(frame)
    fade = np.arange(frame, 0, -1, dtype=np.float64) / float(frame)
    for i in range(frames):
        start = i * frame
        if dropped[i]:
            if conceal == "repeat":
                out[start:start + frame] = last_good * fade
            else:
                out[start:start + frame] = 0.0
        else:
            last_good = out[start:start + frame].copy()
    return out, float(dropped.mean())


def rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x)))) if x.size else 0.0


def write_f32(path: Path, x: np.ndarray) -> None:
    """Raw little-endian float32 mono; clip only, never rescale."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(np.clip(np.asarray(x, dtype=np.float64), -1.0, 1.0).astype("<f4").tobytes())


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def append_jsonl(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8", newline="\n") as output:
        output.write(json.dumps(value, ensure_ascii=False, separators=(",", ":")) + "\n")
        output.flush()
        os.fsync(output.fileno())


def wav_metadata(path: Path) -> dict:
    with wave.open(str(path), "rb") as audio:
        return {
            "channels": audio.getnchannels(), "sample_rate": audio.getframerate(),
            "sample_width_bytes": audio.getsampwidth(), "frames": audio.getnframes(),
        }


def level_adjust(x: np.ndarray, target_rms: float) -> np.ndarray:
    if x.size == 0:
        return x
    current = rms(x)
    if current <= 0.0:
        return x
    gain_db = float(np.clip(20.0 * np.log10(target_rms / current),
                            -MAX_LEVEL_GAIN_DB, MAX_LEVEL_GAIN_DB))
    return x * (10.0 ** (gain_db / 20.0))


# ---------------------------------------------------------------------------
# external tools
# ---------------------------------------------------------------------------
def run_dsp_tool(tool: Path, src: Path, dst: Path, mode: int, gain: float) -> str:
    dst.parent.mkdir(parents=True, exist_ok=True)
    cmd = [str(tool), str(src), str(dst), "--mode", str(mode), "--gain", f"{gain:g}"]
    proc = subprocess.run(cmd, capture_output=True, timeout=1800)
    if proc.returncode != 0:
        raise RuntimeError(f"dsp tool failed ({proc.returncode}): {proc.stderr.decode('utf-8', 'replace')}")
    return proc.stderr.decode("utf-8", "replace").strip()


def native_worker_frame(value: dict) -> bytes:
    body = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    if not body or len(body) > 1024 * 1024:
        raise RuntimeError("HMI native-worker request frame exceeds 1 MiB")
    return struct.pack(">I", len(body)) + body


def native_worker_frames(data: bytes) -> list[dict]:
    frames = []
    offset = 0
    while offset < len(data):
        if offset + 4 > len(data):
            raise RuntimeError("HMI native worker returned a partial frame header")
        size = struct.unpack_from(">I", data, offset)[0]
        offset += 4
        if size == 0 or size > 1024 * 1024 or offset + size > len(data):
            raise RuntimeError("HMI native worker returned an invalid frame length")
        try:
            frames.append(json.loads(data[offset:offset + size].decode("utf-8")))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("HMI native worker returned invalid JSON") from exc
        offset += size
    return frames


def run_hmi_worker_asr(worker: Path, pcm: Path, model: Path, vad: Path,
                       timeout: float) -> tuple[str, float]:
    """Use HMI's own Qwen3 native worker on the evaluator's float32 PCM.

    The worker writes its ggml backend banner to stdout before the framed
    responses, and it does not necessarily exit after Commit.  Read frames
    incrementally, skip non-frame bytes one at a time, and stop as soon as a
    Final/Error frame arrives.
    """
    samples = np.fromfile(pcm, dtype="<f4")
    if samples.size == 0 or samples.size > MAX_PROBE_SAMPLES or not np.isfinite(samples).all():
        raise RuntimeError("HMI worker input must be finite, non-empty, and at most 30 seconds")
    quantized = np.rint(np.clip(samples, -1.0, 32767.0 / 32768.0) * 32768.0).astype("<i2")
    request = bytearray(native_worker_frame({
        "type": "Init", "sample_rate": SAMPLE_RATE, "channels": 1, "archive": False,
        "test_pipeline": {"denoise": "off", "vad": False},
    }))
    for offset in range(0, quantized.size, 8000):
        pcm16 = quantized[offset:offset + 8000].tobytes()
        request.extend(native_worker_frame({
            "type": "AudioFrame", "samples": base64.b64encode(pcm16).decode("ascii"),
        }))
    request.extend(native_worker_frame({"type": "Commit"}))

    started = time.perf_counter()
    proc = subprocess.Popen(
        [str(worker), "--native-audio-worker", "--model-path", str(model), "--vad-path", str(vad)],
        cwd=str(worker.parent), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    chunks: list[bytes] = []

    def _reader() -> None:
        while True:
            data = proc.stdout.read(65536)
            if not data:
                return
            chunks.append(data)

    threading.Thread(target=_reader, daemon=True).start()
    try:
        proc.stdin.write(bytes(request))
        proc.stdin.close()
    except OSError:
        pass

    buffer = bytearray()
    frames: list[dict] = []
    deadline = started + timeout
    while time.perf_counter() < deadline:
        buffer.extend(b"".join(chunks))
        chunks.clear()
        offset = 0
        while len(buffer) - offset >= 4:
            size = struct.unpack_from(">I", buffer, offset)[0]
            if size == 0 or size > 1024 * 1024:
                offset += 1
                continue
            if len(buffer) - offset - 4 < size:
                break
            body = bytes(buffer[offset + 4:offset + 4 + size])
            try:
                frame = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                offset += 1
                continue
            frames.append(frame)
            offset += 4 + size
            if frame.get("type") in ("Final", "Error"):
                break
        del buffer[:offset]
        if any(frame.get("type") in ("Final", "Error") for frame in frames):
            break
        time.sleep(0.05)

    elapsed_ms = (time.perf_counter() - started) * 1000.0
    if proc.poll() is None:
        proc.kill()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    error = next((frame.get("message") for frame in frames if frame.get("type") == "Error"), None)
    final = next((frame for frame in reversed(frames) if frame.get("type") == "Final"), None)
    if final is None:
        tail = b"".join(chunks).decode("utf-8", "replace")[-1200:]
        raise RuntimeError(f"HMI native worker produced no Final frame: {error or tail or 'timeout'}")
    return str(final.get("text", "")).strip(), elapsed_ms

def run_asr(probe: Path, pcm: Path, project_root: Path, timeout: float,
            hmi_worker: tuple[Path, Path, Path] | None = None) -> tuple[str, float]:
    """Return (text, finish_ms). Probe exit 7 only means --expect-text mismatch."""
    if hmi_worker is not None:
        worker, model, vad = hmi_worker
        return run_hmi_worker_asr(worker, pcm, model, vad, timeout)
    cmd = [str(probe), "--pcm", str(pcm), "--final-only", "--iterations", "1",
           "--backend", "auto"]
    proc = subprocess.run(cmd, cwd=str(project_root), capture_output=True, timeout=timeout)
    out = proc.stdout.decode("utf-8", "replace")
    for line in out.splitlines():
        m = PROBE_RE.match(line.strip())
        if m:
            return m.group(4).strip(), float(m.group(3))
    tail = "\n".join(out.splitlines()[-6:])
    raise RuntimeError(f"cannot parse ASR output (exit {proc.returncode}):\n{tail}")


# ---------------------------------------------------------------------------
# text normalisation / CER
# ---------------------------------------------------------------------------
def normalize_text(text: str, keep_punct: bool = False, drop_spaces: bool = True) -> str:
    text = unicodedata.normalize("NFKC", text).lower()
    if not keep_punct:
        out = []
        for ch in text:
            cat = unicodedata.category(ch)
            if cat.startswith("P") or cat.startswith("S") or cat in ("Cf", "Cc", "Zs"):
                out.append(" " if ch.isspace() else "")
            else:
                out.append(ch)
        text = "".join(out)
    if drop_spaces:
        text = re.sub(r"\s+", "", text)
    else:
        text = re.sub(r"\s+", " ", text).strip()
    return text


def tokenize(text: str, unit: str) -> list[str]:
    return text.split() if unit == "word" else list(text)


def edit_ops(ref: Sequence[str], hyp: Sequence[str]) -> tuple[int, int, int, int]:
    """Levenshtein with backtrace. Returns (distance, subs, dels, ins)."""
    n, m = len(ref), len(hyp)
    dp = np.zeros((n + 1, m + 1), dtype=np.int32)
    dp[:, 0] = np.arange(n + 1)
    dp[0, :] = np.arange(m + 1)
    for i in range(1, n + 1):
        ri = ref[i - 1]
        for j in range(1, m + 1):
            cost = 0 if ri == hyp[j - 1] else 1
            dp[i, j] = min(dp[i - 1, j] + 1, dp[i, j - 1] + 1, dp[i - 1, j - 1] + cost)
    i, j = n, m
    sub = dele = ins = 0
    while i > 0 or j > 0:
        if i > 0 and j > 0 and dp[i, j] == dp[i - 1, j - 1] + (0 if ref[i - 1] == hyp[j - 1] else 1):
            if ref[i - 1] != hyp[j - 1]:
                sub += 1
            i, j = i - 1, j - 1
        elif i > 0 and dp[i, j] == dp[i - 1, j] + 1:
            dele += 1
            i -= 1
        else:
            ins += 1
            j -= 1
    return int(dp[n, m]), sub, dele, ins


@dataclass
class Score:
    cer: float
    ref_len: int
    dist: int
    sub: int
    dele: int
    ins: int


def score_pair(ref: str, hyp: str, unit: str, keep_punct: bool) -> Score:
    r = tokenize(normalize_text(ref, keep_punct, drop_spaces=(unit == "char")), unit)
    h = tokenize(normalize_text(hyp, keep_punct, drop_spaces=(unit == "char")), unit)
    dist, sub, dele, ins = edit_ops(r, h)
    return Score(dist / max(1, len(r)), len(r), dist, sub, dele, ins)


def paired_bootstrap(base: np.ndarray, enh: np.ndarray, iters: int, seed: int) -> dict:
    rng = np.random.default_rng(seed)
    n = len(base)
    if n == 0:
        return {"delta": 0.0, "ci_low": 0.0, "ci_high": 0.0, "p_better": 0.5}
    deltas = enh - base
    obs = float(deltas.mean())
    boots = np.empty(iters)
    for k in range(iters):
        boots[k] = deltas[rng.integers(0, n, n)].mean()
    return {
        "delta": obs,
        "ci_low": float(np.percentile(boots, 2.5)),
        "ci_high": float(np.percentile(boots, 97.5)),
        "p_better": float(np.mean(boots < 0.0) + 0.5 * np.mean(boots == 0.0)),
    }


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def build_conditions(modes: list[int], gains: list[float],
                     drops: list[float], conceals: list[str],
                     codecs: list[str]) -> list[Condition]:
    multi_codec = len(codecs) > 1 or codecs[0] != "none"
    out: list[Condition] = []
    for mode in modes:
        for gain in gains:
            for codec in codecs:
                ctag = f"_c{CODEC_SIM_TAGS[codec]}" if multi_codec else ""
                for drop in drops:
                    if drop <= 0.0:
                        out.append(Condition(f"m{mode}_g{gain:g}{ctag}", mode, gain,
                                             0.0, "repeat", codec))
                        continue
                    for conceal in conceals:
                        label = f"m{mode}_g{gain:g}{ctag}_d{int(round(drop * 100))}{conceal[0]}"
                        out.append(Condition(label, mode, gain, drop, conceal, codec))
    return out


def noise_pattern(uid: str) -> tuple[str, int] | None:
    match = NOISE_PATTERN_RE.search(uid)
    return (match.group(1), int(match.group(2))) if match else None


def pattern_report(summary: dict, rows: list[dict], conditions: list[Condition],
                   baseline_label: str, items: list[Item], args: argparse.Namespace,
                   model_metadata: dict) -> dict:
    """Write the bounded, offline spatial-pattern report consumed by HMI."""
    by_condition = {condition.label: [row for row in rows if row["condition"] == condition.label]
                    for condition in conditions}
    baseline_rows = {row["id"]: row for row in by_condition[baseline_label]}
    baseline_cer = {
        row["id"]: score_pair(row["ref"], row["hyp"], "char", args.keep_punct).cer
        for row in by_condition[baseline_label]
    }
    arms = []
    input_metadata = []
    for item in items:
        try:
            relative = item.wav.resolve().relative_to(args.manifest.resolve().parent)
            source_name = str(relative)
        except ValueError:
            source_name = item.wav.name
        input_metadata.append({
            "id": item.uid, "wav": source_name,
            "sha256": sha256_file(item.wav), **wav_metadata(item.wav),
        })
    for condition in conditions:
        selected = by_condition[condition.label]
        scored_rows = []
        strata: dict[tuple[str, int], list[tuple[dict, object]]] = {}
        character_scores = [score_pair(row["ref"], row["hyp"], "char", args.keep_punct)
                            for row in selected]
        for row in selected:
            pattern = noise_pattern(row["id"])
            pattern_name, snr_db = pattern if pattern else ("unclassified", 0)
            dataset = f"{pattern_name} / SNR {snr_db:+d} dB" if pattern else "unclassified"
            cer = score_pair(row["ref"], row["hyp"], "char", args.keep_punct)
            wer = score_pair(row["ref"], row["hyp"], "word", args.keep_punct)
            scored_rows.append({
                "id": row["id"], "dataset": dataset, "reference": row["ref"],
                "status": "completed", "raw_text": row["hyp"],
                "raw_score": {
                    "cer": {"errors": cer.dist, "n": cer.ref_len},
                    "wer": {"errors": wer.dist, "n": wer.ref_len},
                },
                "capture_quality": {"rms_dbfs": row["rms_dbfs"], "basis": "offline_dsp_output"},
            })
            if pattern:
                strata.setdefault(pattern, []).append((row, cer))
        condition_summary = summary["conditions"][condition.label]
        condition_summary["micro_cer"] = sum(score.dist for score in character_scores) / max(
            1, sum(score.ref_len for score in character_scores))
        condition_summary["wer_mean"] = float(np.mean([
            score_pair(row["ref"], row["hyp"], "word", args.keep_punct).cer for row in selected
        ])) if selected else None
        stratum_summaries = []
        for (pattern_name, snr_db), group in sorted(strata.items()):
            ref_errors = sum(cer.dist for _, cer in group)
            ref_chars = sum(cer.ref_len for _, cer in group)
            deltas = []
            if condition.label != baseline_label:
                paired = [(baseline_cer[row["id"]], cer.cer)
                          for row, cer in group if row["id"] in baseline_rows]
                if paired:
                    result = paired_bootstrap(
                        np.array([item[0] for item in paired]),
                        np.array([item[1] for item in paired]), args.bootstrap,
                        args.seed + len(stratum_summaries) + condition.mode * 101,
                    )
                    deltas.append(result)
            stratum_summaries.append({
                "pattern": pattern_name, "snr_db": snr_db, "n": len(group),
                "cer_mean": float(np.mean([cer.cer for _, cer in group])) if group else None,
                "micro_cer": ref_errors / ref_chars if ref_chars else None,
                "vs_baseline": deltas[0] if deltas else None,
            })
        paired = None
        if condition.label != baseline_label:
            paired_rows = [row for row in selected if row["id"] in baseline_cer]
            if paired_rows:
                paired = paired_bootstrap(
                    np.array([baseline_cer[row["id"]] for row in paired_rows]),
                    np.array([score_pair(row["ref"], row["hyp"], "char", args.keep_punct).cer
                              for row in paired_rows]),
                    args.bootstrap,
                    args.seed,
                )
        arms.append({
            "mode": condition.mode,
            "label": f"Mode {condition.mode} · {SPATIAL_MODE_LABELS.get(condition.mode, 'Unknown')}",
            "condition": condition.label,
            "complete": len(selected) == len(items),
            "coverage": {
                "planned": len(items), "attempted": len(selected),
                "transcribed": sum(bool(row["hyp"].strip()) for row in selected),
                "empty": sum(not row["hyp"].strip() for row in selected),
                "failed": 0, "cancelled": 0,
                "raw": {
                    "cer": {"errors": sum(score.dist for score in character_scores),
                            "n": sum(score.ref_len for score in character_scores)},
                    "wer": {
                        "errors": sum(score_pair(row["ref"], row["hyp"], "word", args.keep_punct).dist for row in selected),
                        "n": sum(score_pair(row["ref"], row["hyp"], "word", args.keep_punct).ref_len for row in selected),
                    },
                },
            },
            "rows": scored_rows,
            "conditions": {
                "condition": condition.label, "gain": condition.gain,
                "codec_sim": condition.codec_sim, "level_normalize": args.level_normalize,
                "baseline": baseline_label, "paired_bootstrap_cer": paired,
                "wer_policy": "whitespace_tokens; secondary and coarse for unsegmented Chinese",
                "strata": stratum_summaries,
            },
        })
    return {
        "schema": 1,
        "kind": "offline_spatial_pattern_asr",
        "evidence_scope": "offline_firmware_dsp_qwen3_asr",
        "complete": all(arm["complete"] for arm in arms),
        "baseline": baseline_label,
        "unit": args.unit,
        "metadata": model_metadata,
        "conditions": {
            "manifest": str(args.manifest.resolve()), "manifest_sha256": sha256_file(args.manifest),
            "source_items": len(items), "noise_pattern_fields": "derived from stable sample IDs; unknown IDs remain unclassified",
            "inputs": input_metadata,
            "dsp_input_channels": 2 if any(condition.mode == 6 for condition in conditions) else
                                 (input_metadata[0]["channels"] if input_metadata else None),
            "spatial_assessment": "The evaluator passes the source WAV to production DSP without pre-downmixing. Mode 6 requires stereo 16 kHz PCM and uses both microphone channels; ASR receives the enhanced mono downmix. Mode 0/4 comparisons retain their existing meanings.",
            "physical_device_tested": False, "audio_capture_performed": False,
            "interpretation": "Offline firmware-source DSP and Qwen3-ASR comparison; CER is character-based and primary. WER uses whitespace tokens and is coarse for unsegmented Chinese. This is not a device, microphone, TASCAM routing, or enclosure measurement.",
        },
        "arms": arms,
    }


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="A/B speech enhancement by ASR CER")
    ap.add_argument("--manifest", required=True, type=Path)
    ap.add_argument("--project", required=True, type=Path, help="Qwen3-ASR project root")
    ap.add_argument("--tool", type=Path, default=Path(__file__).resolve().parents[2] / "build" / "asr-eval-host" / "bin" / "win_spatial_wav.exe")
    ap.add_argument("--hmi-worker-exe", type=Path, default=None,
                    help="run ASR through HMI's native worker instead of hmi_voice_probe.exe")
    ap.add_argument("--model-path", type=Path, default=None,
                    help="HMI Qwen3-ASR model (default <project>/src-tauri/native/models/asr.gguf)")
    ap.add_argument("--vad-path", type=Path, default=None,
                    help="HMI VAD asset path; VAD is disabled for this offline comparison")
    ap.add_argument("--modes", default="0,4,6", help="spatial modes (default 0,4,6; mode 6 requires stereo input)")
    ap.add_argument("--gains", default="1", help="fixed gain sweep, e.g. 1,4,16 (default 1)")
    ap.add_argument("--level-normalize", choices=("none", "input", "dbfs"), default="none",
                    help="none = keep the condition's own level; input = match the source RMS; "
                         "dbfs = fixed target (see --target-dbfs)")
    ap.add_argument("--target-dbfs", type=float, default=-26.0)
    ap.add_argument("--downmix", choices=("mean", "left", "right", "best"), default="mean",
                    help="stereo -> mono rule for the enhanced WAV (default mean)")
    ap.add_argument("--drop-rates", default="0",
                    help="simulated 20 ms frame-loss rates, e.g. 0,0.05,0.1 (default 0)")
    ap.add_argument("--conceals", default="zero,repeat",
                    help="concealment strategies paired at every non-zero drop rate")
    ap.add_argument("--codec-sims", default="none",
                    help="codec round-trips before ASR: none,g722,opus32,opus62 (ffmpeg)")
    ap.add_argument("--ffmpeg", type=Path, default=Path(r"D:\PKG\ffmpeg\bin\ffmpeg.exe"),
                    help="ffmpeg used by --codec-sims")
    ap.add_argument("--drop-seed", type=int, default=7,
                    help="frame-loss RNG seed, shared by both conceal modes")
    ap.add_argument("--unit", choices=("char", "word"), default="char")
    ap.add_argument("--keep-punct", action="store_true")
    ap.add_argument("--limit", type=int, default=0, help="evaluate only the first N items")
    ap.add_argument("--bootstrap", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--timeout", type=float, default=600.0, help="per ASR call, seconds")
    ap.add_argument("--work-dir", type=Path, default=None,
                    help="must stay inside the project (default <project>/build/tmp/asr-eval)")
    ap.add_argument("--scratch", type=Path, default=None,
                    help="where the DSP tool writes intermediate WAVs (default <out>/_wav)")
    ap.add_argument("--out", type=Path, default=Path("results"))
    ap.add_argument("--baseline", default=None, help="condition label used as baseline (default: first)")
    ap.add_argument("--dry-run", action="store_true",
                    help="skip ASR, use the reference text as hypothesis (pipeline self-test)")
    args = ap.parse_args(list(argv) if argv is not None else None)

    project = args.project.resolve()
    probe = project / "build" / "bin" / "Release" / "hmi_voice_probe.exe"
    hmi_worker = None
    if args.hmi_worker_exe is not None:
        worker = args.hmi_worker_exe.resolve()
        model = (args.model_path or (project / "src-tauri" / "native" / "models" / "asr.gguf")).resolve()
        vad = (args.vad_path or (project / "src-tauri" / "native" / "models" / "vad.safetensors")).resolve()
        for label, path in (("HMI worker", worker), ("Qwen3-ASR model", model), ("HMI VAD asset", vad)):
            if not path.is_file():
                raise SystemExit(f"{label} not found: {path}")
        hmi_worker = (worker, model, vad)
    elif args.model_path is not None or args.vad_path is not None:
        raise SystemExit("--model-path and --vad-path require --hmi-worker-exe")
    if not args.dry_run and hmi_worker is None and not probe.is_file():
        raise SystemExit(f"probe not found: {probe}\n"
                         f"build it with: cmake --build {project / 'build'} --config Release --target hmi_voice_probe")
    if not args.tool.is_file():
        build_script = Path(__file__).with_name("host") / "build.ps1"
        raise SystemExit(f'dsp tool not found: {args.tool}\nbuild it with: powershell -File "{build_script}"')

    work = (args.work_dir or (project / "build" / "tmp" / "asr-eval")).resolve()
    try:
        work.mkdir(parents=True, exist_ok=True)
        probe_file = work / ".write-test"
        probe_file.write_text("ok", encoding="utf-8")
        probe_file.unlink()
    except OSError as exc:
        raise SystemExit(f"work dir not writable: {work} ({exc})\n"
                         f"the probe requires PCM files inside the project directory")
    if not args.dry_run and project not in work.parents:
        raise SystemExit(f"work dir must be inside the project: {work}")

    items = load_manifest(args.manifest)
    if args.limit:
        items = items[: args.limit]
    modes = [int(x) for x in args.modes.split(",") if x.strip()]
    gains = [float(x) for x in args.gains.split(",") if x.strip()]
    drops = [float(x) for x in args.drop_rates.split(",") if x.strip()]
    conceals = [x.strip() for x in args.conceals.split(",") if x.strip()]
    codecs = [x.strip() for x in args.codec_sims.split(",") if x.strip()]
    if not modes or not gains or not drops or not conceals or not codecs:
        raise SystemExit("empty --modes/--gains/--drop-rates/--conceals/--codec-sims")
    if any(mode < 0 or mode > 6 for mode in modes):
        raise SystemExit("spatial modes must be in 0..6")
    if 6 in modes:
        mono_items = [item.uid for item in items if wav_metadata(item.wav)["channels"] != 2]
        if mono_items:
            raise SystemExit("mode 6 requires physical two-channel WAV inputs; mono samples: " +
                             ", ".join(mono_items[:8]))
    for c in conceals:
        if c not in ("zero", "repeat"):
            raise SystemExit(f"unknown conceal strategy: {c}")
    for c in codecs:
        if c not in CODEC_SIM_TAGS:
            raise SystemExit(f"unknown --codec-sims value: {c}")
    conditions = build_conditions(modes, gains, drops, conceals, codecs)

    baseline_label = args.baseline or conditions[0].label
    if baseline_label not in {c.label for c in conditions}:
        raise SystemExit(f"baseline {baseline_label} is not among {[c.label for c in conditions]}")

    if args.out.exists() and (not args.out.is_dir() or any(args.out.iterdir())):
        raise SystemExit(f"output path already contains evidence or is not a directory; choose a new immutable output path: {args.out}")
    args.out.mkdir(parents=True, exist_ok=True)
    scratch = (args.scratch or (args.out / "_wav")).resolve()
    scratch.mkdir(parents=True, exist_ok=True)
    target_rms = 10.0 ** (args.target_dbfs / 20.0)

    print(f"items={len(items)} conditions={[c.label for c in conditions]} "
          f"baseline={baseline_label} level={args.level_normalize} "
          f"unit={args.unit} dry_run={args.dry_run}")
    print(f"work={work}")

    rows: list[dict] = []
    asr_ms: dict[str, list[float]] = {c.label: [] for c in conditions}

    # Resumable ASR cache: one JSON line per (PCM sha1, transcript). Long runs
    # are ~8 s per call, so a killed run must not throw away finished work.
    cache_path = args.out / "asr_cache.jsonl"
    asr_cache: dict[str, str] = {}
    if cache_path.is_file():
        for line in cache_path.read_text(encoding="utf-8").splitlines():
            try:
                rec = json.loads(line)
                asr_cache[rec["k"]] = rec["text"]
            except (json.JSONDecodeError, KeyError):
                continue
    print(f"asr cache: {len(asr_cache)} entries from {cache_path.name}")
    cache_hits = 0
    partial_rows_path = args.out / "per_utterance.partial.jsonl"
    input_hashes = {str(item.wav.resolve()): sha256_file(item.wav) for item in items}

    for cond in conditions:
        for n, item in enumerate(items, 1):
            stem = f"{item.uid}_{cond.label}"
            wav_out = scratch / f"{stem}.wav"
            f32_out = work / f"{stem}.f32"
            stats = run_dsp_tool(args.tool, item.wav, wav_out, cond.mode, cond.gain)

            source_channels = read_wav_channels16k(item.wav)
            src = downmix_channels(source_channels, args.downmix)
            # Clipping happens at the gain stage, before any spatial/Wiener work,
            # so measure it on the source scaled by the condition's gain.
            clip_pct = float(np.mean(np.any(np.abs(source_channels) * cond.gain >= 0.999, axis=1))) * 100.0 if src.size else 0.0

            pcm = read_wav_mono16k(wav_out, args.downmix)
            # Normalise level BEFORE injecting loss: otherwise the zero-fill
            # condition would be quieter and earn extra gain, biasing the A/B.
            if args.level_normalize == "input":
                pcm = level_adjust(pcm, rms(src))
            elif args.level_normalize == "dbfs":
                pcm = level_adjust(pcm, target_rms)
            if cond.codec_sim != "none":
                # Codec sits between the sender and the receiver, so it runs
                # before the frame-loss simulation (loss happens post-decode).
                pcm = apply_codec_sim(pcm, cond.codec_sim, args.ffmpeg,
                                      scratch / "codec_tmp")
            if cond.drop > 0.0:
                # Stable per-item seed (crc32, not hash()) so reruns and both
                # concealment modes drop exactly the same frames.
                item_seed = args.drop_seed + zlib.crc32(item.uid.encode("utf-8")) % 100000
                pcm, _ = apply_frame_drops(pcm, cond.drop, cond.conceal, item_seed)

            if pcm.size > MAX_PROBE_SAMPLES:
                raise SystemExit(f"{item.uid}: {pcm.size / SAMPLE_RATE:.1f}s exceeds the probe's 30s limit; "
                                 f"split the utterance first")
            write_f32(f32_out, pcm)

            if args.dry_run:
                hyp, finish_ms = item.ref, 0.0
            else:
                key = hashlib.sha1(f32_out.read_bytes()).hexdigest()
                if key in asr_cache:
                    hyp, finish_ms = asr_cache[key], 0.0
                    cache_hits += 1
                else:
                    hyp, finish_ms = run_asr(probe, f32_out, project, args.timeout, hmi_worker)
                    asr_ms[cond.label].append(finish_ms)
                    asr_cache[key] = hyp
                    with cache_path.open("a", encoding="utf-8") as cf:
                        cf.write(json.dumps({"k": key, "text": hyp}, ensure_ascii=False) + "\n")

            sc = score_pair(item.ref, hyp, args.unit, args.keep_punct)
            word_score = score_pair(item.ref, hyp, "word", args.keep_punct)
            row = {
                "id": item.uid, "condition": cond.label, "mode": cond.mode, "gain": cond.gain,
                "drop": cond.drop, "conceal": cond.conceal, "codec_sim": cond.codec_sim,
                "ref": item.ref, "hyp": hyp, "cer": sc.cer, "ref_len": sc.ref_len,
                "dist": sc.dist, "sub": sc.sub, "del": sc.dele, "ins": sc.ins,
                "wer": word_score.cer, "wer_ref_len": word_score.ref_len, "wer_dist": word_score.dist,
                "asr_ms": finish_ms, "clip_pct": clip_pct,
                "input_wav_sha256": input_hashes[str(item.wav.resolve())],
                "rms_dbfs": 20.0 * np.log10(rms(pcm) + 1e-12), "stats": stats,
            }
            rows.append(row)
            append_jsonl(partial_rows_path, row)
            print(f"[{cond.label}] {n}/{len(items)} {item.uid}: CER={sc.cer*100:6.2f}% "
                  f"({sc.dist}/{sc.ref_len}) clip={clip_pct:4.1f}% {hyp[:24]!r}")

    config = {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()}
    config.update({"manifest": str(args.manifest), "project": str(project),
                   "work_dir": str(work), "conditions": [c.label for c in conditions]})
    summary = {"config": config, "baseline": baseline_label, "unit": args.unit, "conditions": {}}

    by_cond = {c.label: [r for r in rows if r["condition"] == c.label] for c in conditions}
    for cond in conditions:
        rs = by_cond[cond.label]
        cer = np.array([r["cer"] for r in rs])
        summary["conditions"][cond.label] = {
            "mode": cond.mode, "gain": cond.gain,
            "drop_rate": cond.drop, "conceal": cond.conceal, "codec_sim": cond.codec_sim, "n": len(rs),
            "cer_mean": float(cer.mean()),
            "cer_median": float(np.median(cer)),
            "wer_mean": float(np.mean([r["wer"] for r in rs])) if rs else 0.0,
            "micro_cer": float(sum(r["dist"] for r in rs) / max(1, sum(r["ref_len"] for r in rs))),
            "micro_wer": float(sum(r["wer_dist"] for r in rs) / max(1, sum(r["wer_ref_len"] for r in rs))),
            "sub": int(sum(r["sub"] for r in rs)),
            "del": int(sum(r["del"] for r in rs)),
            "ins": int(sum(r["ins"] for r in rs)),
            "clip_pct_mean": float(np.mean([r["clip_pct"] for r in rs])),
            "rms_dbfs_mean": float(np.mean([r["rms_dbfs"] for r in rs])),
            "asr_ms_mean": float(np.mean(asr_ms[cond.label])) if asr_ms[cond.label] else 0.0,
        }

    ids = [r["id"] for r in by_cond[baseline_label]]
    base_vec = np.array([r["cer"] for r in by_cond[baseline_label]])
    for cond in conditions:
        if cond.label == baseline_label:
            continue
        vec = np.array([r["cer"] for r in by_cond[cond.label]])
        summary["conditions"][cond.label]["vs_baseline"] = paired_bootstrap(
            base_vec, vec, args.bootstrap, args.seed)

    with (args.out / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    fields = ["id", "condition", "mode", "gain", "drop", "conceal", "codec_sim",
              "ref", "hyp", "cer", "ref_len", "dist", "sub", "del", "ins", "wer", "wer_ref_len", "wer_dist",
              "asr_ms", "clip_pct", "rms_dbfs", "stats"]
    with (args.out / "per_utterance.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for cond in conditions:
            for row in by_cond[cond.label]:
                writer.writerow({k: row[k] for k in fields})

    if not args.dry_run and hmi_worker is not None:
        manifest_path = Path(__file__).parents[2] / "SOURCE_MANIFEST.json"
        manifest_data = json.loads(manifest_path.read_text(encoding="utf-8-sig")) if manifest_path.is_file() else {}
        firmware_sources = []
        for relative in (
            Path("components/dual_end_audio_codec/dual_end_spatial.c"),
            Path("components/dual_end_audio_codec/dual_end_wiener.c"),
            Path("components/dual_end_audio_codec/dual_end_mvdr.c"),
        ):
            source = Path(__file__).parents[2] / relative
            firmware_sources.append({
                "path": relative.as_posix(), "sha256": sha256_file(source),
            })
        evaluator = Path(__file__).resolve()
        host_build_script = Path(__file__).with_name("host") / "build.ps1"
        model_metadata = {
            "recognizer": "HMI native worker · Qwen3-ASR",
            "model_path": str(hmi_worker[1]), "model_sha256": sha256_file(hmi_worker[1]),
            "hmi_executable": str(hmi_worker[0]), "hmi_executable_sha256": sha256_file(hmi_worker[0]),
            "dsp_tool": str(args.tool.resolve()), "dsp_tool_sha256": sha256_file(args.tool),
            "evaluator_script": evaluator.name, "evaluator_script_sha256": sha256_file(evaluator),
            "host_build_script_sha256": sha256_file(host_build_script),
            "firmware_version_from_source_manifest": manifest_data.get("firmware_version"),
            "esp_idf_version_from_source_manifest": manifest_data.get("esp_idf_version"),
            "source_manifest_file_sha256": sha256_file(manifest_path) if manifest_path.is_file() else None,
            "firmware_dsp_sources": firmware_sources,
            "vad_enabled": False, "host_denoise": "off",
            "asr_sample_rate": SAMPLE_RATE, "asr_channels": 1,
        }
        report = pattern_report(summary, rows, conditions, baseline_label, items, args, model_metadata)
        report_path = args.out / "pattern-results.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        # Pattern report adds character CER values even if the legacy summary's
        # configurable --unit is set to word. Keep both artifacts consistent.
        with (args.out / "summary.json").open("w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)

    print("\n=== summary ===")
    print(f"{'condition':>10} {'n':>4} {'CER%':>7} {'WER%':>7} {'S':>5} {'D':>5} {'I':>5} "
          f"{'clip%':>6} {'dBFS':>7} {'dCER':>7} {'95% CI':>16} {'P':>5}")
    for cond in conditions:
        s = summary["conditions"][cond.label]
        v = s.get("vs_baseline")
        if v:
            delta = f"{v['delta']*100:+.2f}"
            ci = f"[{v['ci_low']*100:+.2f},{v['ci_high']*100:+.2f}]"
            p = f"{v['p_better']:.3f}"
        else:
            delta, ci, p = "-", "-", "-"
        print(f"{cond.label:>10} {s['n']:>4} {s['cer_mean']*100:>7.2f} {s['wer_mean']*100:>7.2f} "
              f"{s['sub']:>5} {s['del']:>5} {s['ins']:>5} {s['clip_pct_mean']:>6.1f} "
              f"{s['rms_dbfs_mean']:>7.1f} {delta:>7} {ci:>16} {p:>5}")
    print(f"\nbaseline = {baseline_label}")
    print(f"asr cache hits: {cache_hits}")
    print(f"wrote {args.out / 'summary.json'} and {args.out / 'per_utterance.csv'}")
    if not args.dry_run and hmi_worker is not None:
        print(f"wrote {args.out / 'pattern-results.json'} for the HMI report library")
    return 0


if __name__ == "__main__":
    sys.exit(main())
