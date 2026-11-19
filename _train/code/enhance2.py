"""用修复版模型（model2/ckpt_e）增强一个采集目录，输出可被评分工具读取的 wav。"""
from __future__ import annotations

import argparse
import os
import sys
import wave

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data2 import FS, istft, read_wav, stft  # noqa: E402
from model2 import CRN  # noqa: E402


def write_wav(path: str, x: np.ndarray) -> None:
    y = np.clip(np.round(x * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(y.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="_train/code/ckpt_e.pt")
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--chunk-s", type=float, default=4.0)
    args = ap.parse_args()

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = CRN().to(dev)
    model.load_state_dict(torch.load(args.ckpt, map_location=dev)["model"])
    model.eval()
    print(f"  loaded {args.ckpt}  device={dev}")

    os.makedirs(args.out, exist_ok=True)
    n_chunk = int(args.chunk_s * FS)
    count = 0
    for d in [s.strip() for s in args.src.split(",") if s.strip()]:
        for f in sorted(os.listdir(d)):
            if not f.endswith("_L.wav") or "_clean_" in f:
                continue
            cid = f[: -len("_L.wav")]
            rp = os.path.join(d, cid + "_R.wav")
            if not os.path.exists(rp):
                continue
            l = read_wav(os.path.join(d, f))
            r = read_wav(rp)
            l = l if l.ndim == 1 else l[:, 0]
            r = r if r.ndim == 1 else r[:, 0]
            n = min(len(l), len(r))
            out = np.zeros(n, dtype=np.float32)
            for a in range(0, n, n_chunk):
                b = min(a + n_chunk, n)
                x = torch.from_numpy(np.stack([l[a:b], r[a:b]])).unsqueeze(0).to(dev)
                with torch.no_grad():
                    y, _, _ = model(stft(x))
                    w = istft(y, x.shape[-1])
                out[a:b] = w[0].cpu().numpy()
            write_wav(os.path.join(args.out, cid + "_L.wav"), out)
            count += 1
            if count % 100 == 0:
                print(f"  {count}", flush=True)
    print(f"  完成 {count} 条 -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
