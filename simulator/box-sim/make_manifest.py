#!/usr/bin/env python3
"""Build an ASR CER manifest from a reference list and a directory of wavs.

Input ``--refs`` is a TSV with a header and at least ``id`` and ``ref`` columns
(the ``wav`` column, when present, is ignored).  Wavs are matched to references
in sorted filename order; pass ``--ids`` to match by filename prefix instead.

Output TSV: ``id<TAB>wav<TAB>ref`` with absolute wav paths.
"""
from __future__ import annotations

import argparse
import csv
import io
import os


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--refs", required=True, help="TSV with id and ref columns")
    parser.add_argument("--wav-dir", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--ids", action="store_true",
                        help="match by filename prefix <id>_ instead of sorted order")
    args = parser.parse_args()

    with io.open(args.refs, encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle, delimiter="\t"))
    if not rows or "ref" not in rows[0]:
        raise SystemExit("reference file needs a 'ref' column")
    if "id" not in rows[0]:
        raise SystemExit("reference file needs an 'id' column")

    wavs = sorted(f for f in os.listdir(args.wav_dir) if f.lower().endswith(".wav"))
    if not wavs:
        raise SystemExit("no wav files in --wav-dir")

    if args.ids:
        by_id = {}
        for name in wavs:
            key = name.split("_", 1)[0]
            by_id.setdefault(key, name)
        pairs = [(row["id"], by_id.get(row["id"], "")) for row in rows]
        missing = [rid for rid, name in pairs if not name]
        if missing:
            raise SystemExit(f"no wav for ids: {', '.join(missing[:5])}")
    else:
        if len(wavs) < len(rows):
            raise SystemExit(f"need {len(rows)} wavs, found {len(wavs)}")
        pairs = [(rows[i]["id"], wavs[i]) for i in range(len(rows))]

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    with io.open(args.out, "w", encoding="utf-8", newline="\n") as handle:
        handle.write("id\twav\tref\n")
        for rid, name in pairs:
            row = next(r for r in rows if r["id"] == rid)
            path = os.path.abspath(os.path.join(args.wav_dir, name))
            handle.write(f"{rid}\t{path}\t{row['ref']}\n")
    print(f"wrote {args.out}: {len(pairs)} entries")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
