#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build an asr_cer_eval manifest from a folder of WAV + transcript pairs.

Accepted transcript layouts:
    wav/utt001.wav + txt/utt001.txt
    wav/utt001.wav + wav/utt001.txt
    wav/utt001.wav + utt001.txt        (flat, transcripts beside the wavs)

Usage:
    python make_manifest.py --wav-dir D:\\data\\wav --txt-dir D:\\data\\txt -o manifest.tsv
"""
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--wav-dir", required=True, type=Path)
    ap.add_argument("--txt-dir", type=Path, default=None, help="default: beside each wav")
    ap.add_argument("-o", "--out", required=True, type=Path)
    args = ap.parse_args()

    wavs = sorted(args.wav_dir.glob("*.wav"))
    if not wavs:
        raise SystemExit(f"no .wav under {args.wav_dir}")

    rows, missing = [], []
    for wav in wavs:
        txt = (args.txt_dir / f"{wav.stem}.txt") if args.txt_dir else wav.with_suffix(".txt")
        if not txt.is_file():
            missing.append(wav.name)
            continue
        text = txt.read_text(encoding="utf-8-sig").strip().replace("\t", " ").replace("\n", " ")
        if not text:
            missing.append(wav.name)
            continue
        rows.append((wav.stem, str(wav), text))

    if not rows:
        raise SystemExit("no wav/txt pairs found")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", encoding="utf-8", newline="") as f:
        f.write("id\twav\tref\n")
        for uid, wav, text in rows:
            f.write(f"{uid}\t{wav}\t{text}\n")

    print(f"wrote {args.out} with {len(rows)} pairs")
    if missing:
        print(f"skipped {len(missing)} without transcript: {', '.join(missing[:5])}"
              + (" ..." if len(missing) > 5 else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
