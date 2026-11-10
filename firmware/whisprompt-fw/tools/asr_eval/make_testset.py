"""Build a small, reproducible ASR test set from the project's TTS output.

Five synthesized sentences, three noise types (white / speech-shaped / babble),
three SNRs, written as 16 kHz mono WAV plus an asr_cer_eval manifest.
"""
from __future__ import annotations

import csv
import os
import tempfile
from pathlib import Path

import numpy as np

PROJECT_TMP = Path(
    os.environ.get(
        "AUDIO_QOS_TMP",
        str(Path(tempfile.gettempdir()) / "audio_qos" / "asr-eval"),
    )
)
OUT = Path(__file__).with_name("testset")
REFS = {
    1: "今天天气不错我们出去走走吧",
    2: "请把会议纪要发到我的邮箱",
    3: "这个方案的成本还需要再核算一遍",
    4: "他昨天下午去了图书馆借书",
    5: "系统升级将在今晚十点开始",
}
SNRS = (0, -5, -10)
KINDS = ("white", "shape", "babble")


def read_f32(path: Path) -> np.ndarray:
    return np.frombuffer(path.read_bytes(), dtype="<f4").astype(np.float64)


def write_wav(path: Path, x: np.ndarray, fs: int = 16000) -> None:
    import wave

    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(fs)
        w.writeframes((np.clip(x, -1.0, 1.0) * 32767.0).astype("<i2").tobytes())


def speech_shaped(rng: np.random.Generator, n: int, template_spectrum: np.ndarray) -> np.ndarray:
    """White noise shaped by an average speech magnitude spectrum."""
    spec = rng.normal(0.0, 1.0, n // 2 + 1) + 1j * rng.normal(0.0, 1.0, n // 2 + 1)
    # Sentences differ in length, so their spectra differ in bin count: map the
    # template onto a normalised frequency axis before applying it.
    grid_src = np.linspace(0.0, 1.0, template_spectrum.size)
    grid_dst = np.linspace(0.0, 1.0, spec.size)
    mag = np.abs(spec) * np.interp(grid_dst, grid_src, template_spectrum)
    shaped = np.fft.irfft(mag * np.exp(1j * np.angle(spec)), n=n)
    return shaped


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(20261103)
    sources = {n: read_f32(PROJECT_TMP / f"tts_s{n}.f32") for n in REFS}

    # Average speech magnitude spectrum, used to colour the shaped noise.
    spectra = [np.abs(np.fft.rfft(s)) for s in sources.values()]
    width = min(s.size for s in spectra)
    template = np.mean([s[:width] for s in spectra], axis=0)

    rows = []
    for n, ref in REFS.items():
        x = sources[n]
        speech_rms = float(np.sqrt(np.mean(x ** 2)))
        for kind in KINDS:
            if kind == "white":
                noise_base = rng.normal(0.0, 1.0, x.size)
            elif kind == "shape":
                noise_base = speech_shaped(rng, x.size, template)
            else:  # babble: the other four sentences tiled as competing speech
                parts = [sources[k] for k in REFS if k != n]
                babble = np.concatenate([np.zeros(1600)] + parts)
                noise_base = np.resize(babble, x.size)
            noise_base = noise_base - float(np.mean(noise_base))

            for snr in SNRS:
                target = speech_rms / (10.0 ** (snr / 20.0))
                cur = float(np.sqrt(np.mean(noise_base ** 2)))
                y = x + noise_base * (target / max(cur, 1e-12))
                peak = float(np.max(np.abs(y)))
                if peak > 0.99:
                    y *= 0.99 / peak
                uid = f"s{n}_{kind}_snr{snr:+03d}"
                wav = OUT / f"{uid}.wav"
                write_wav(wav, y)
                rows.append((uid, str(wav), ref, n, kind, snr))
        print(f"s{n}: {x.size/16000:.2f}s rms={20*np.log10(speech_rms):.1f} dBFS")

    with (OUT / "manifest.tsv").open("w", encoding="utf-8", newline="") as f:
        f.write("id\twav\tref\n")
        for uid, wav, ref, _, _, _ in rows:
            f.write(f"{uid}\t{wav}\t{ref}\n")
    with (OUT / "meta.csv").open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "sentence", "noise", "snr_db"])
        for uid, _, _, n, kind, snr in rows:
            w.writerow([uid, n, kind, snr])
    print(f"wrote {len(rows)} items -> {OUT / 'manifest.tsv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
