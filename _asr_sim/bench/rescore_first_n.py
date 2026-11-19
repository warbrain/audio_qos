"""用同一 ASR 后端重评指定目录的前 N 条，保证跨工位可比。

用法：
    python rescore_first_n.py --src ../snr_threshold/snr+0_m0 --n 24 --backend cuda
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

import simcore as S
import score as SC


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--backend", default="cuda")
    args = ap.parse_args()

    items = S.load_manifest(args.n)
    refs = {it["id"]: it["reference"] for it in items}
    keep = set(refs)

    tmp = tempfile.mkdtemp(prefix="rescore_")
    try:
        copied = 0
        for f in os.listdir(args.src):
            cid = f.rsplit("_", 1)[0]
            if cid in keep and f.endswith((".wav", ".json", ".jsonl")):
                shutil.copy2(os.path.join(args.src, f), os.path.join(tmp, f))
                if f.endswith("_L.wav"):
                    copied += 1
        if copied == 0:
            print("  没有匹配到前 N 条文件")
            return 1
        eng = SC.open_asr(args.backend)
        try:
            res = SC.score_dir(tmp, refs, eng)
        finally:
            eng.close()
        print(f"  {os.path.basename(args.src)}  前 {copied} 条  backend={args.backend}")
        print(f"    CER {res['cer']:6.2f}%   全对 {res['perfect']}/{res['clips']}")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
