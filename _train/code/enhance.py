"""用训练好的模型增强一个采集目录，输出可被现有评分工具读取的 wav。

输出目录结构与采集目录一致（<id>_L.wav），因此可以直接用
bench/score.py / bench/compare_dirs.py 评分，与 Fusion::Left / Mean 对比。

用法：
    python enhance.py --ckpt ckpt.pt --src ../spatial_dir/m5_2cm196,../spatial_dir/v6_new196 \
        --out ../spatial_dir/model392
"""
from __future__ import annotations

import argparse
import os
import sys
import wave

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import FS, istft, read_wav, stft  # noqa: E402
from model import CRN  # noqa: E402


def write_wav(path: str, x: np.ndarray) -> None:
    y = np.clip(np.round(x * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(y.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="_train/code/ckpt.pt")
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--chunk-s", type=float, default=4.0)
    args = ap.parse_args()

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = CRN().to(dev)
    state = torch.load(args.ckpt, map_location=dev)
    model.load_state_dict(state["model"])
    model.eval()
    print(f"  loaded {args.ckpt}  device={dev}")

    os.makedirs(args.out, exist_ok=True)
    n_chunk = int(args.chunk_s * FS)
    count = 0
    for d in [s.strip() for s in args.src.split(",") if s.strip()]:
        for f in sorted(os.listdir(d)):
            if not f.endswith("_L.wav"):
                continue
            cid = f[: -len("_L.wav")]
            rp = os.path.join(d, cid + "_R.wav")
            if not os.path.exists(rp):
                continue
            l = read_wav(os.path.join(d, f))
            r = read_wav(rp)
            n = min(len(l), len(r))
            out = np.zeros(n, dtype=np.float32)
            for a in range(0, n, n_chunk):
                b = min(a + n_chunk, n)
                x = torch.from_numpy(np.stack([l[a:b], r[a:b]])).unsqueeze(0).to(dev)
                with torch.no_grad():
                    spec = stft(x)
                    y, _, _ = model(spec)
                    w = istft(y, x.shape[-1])
                out[a:b] = w[0].cpu().numpy()
            write_wav(os.path.join(args.out, cid + "_L.wav"), out)
            count += 1
            if count % 50 == 0:
                print(f"  {count} 条", flush=True)
    print(f"  完成 {count} 条 -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
