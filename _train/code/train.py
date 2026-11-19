"""训练双通道降噪模型（WSL + CUDA）。

用法：
    python train.py --smoke                 # 单批过拟合，验证链路
    python train.py --epochs 40 --batch 8   # 正式训练
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import torch
from torch.utils.data import DataLoader

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import PairedDataset, istft, read_wav, stft  # noqa: E402
from model import CRN  # noqa: E402

ROOT = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir"
NOISY = [os.path.join(ROOT, "train600_paired")]
QUIET = os.path.join(ROOT, "train600_paired")
IDS = "/mnt/d/App/cppApp/audio_qos/_asr_sim/train504_ids.json"


def loss_fn(pred: torch.Tensor, target: torch.Tensor, length: int, dev) -> torch.Tensor:
    ps = torch.abs(stft(pred))
    ts = torch.abs(stft(target))
    spec = (ps - ts).abs().mean()
    t1 = (pred - target).abs().mean()
    return spec + 2.0 * t1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--clip-s", type=float, default=2.0)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--out", default="_train/code/ckpt.pt")
    ap.add_argument("--ids", default=IDS, help="对齐质量过滤清单")
    ap.add_argument("--min-r", type=float, default=0.5)
    args = ap.parse_args()

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    allowed = None
    if args.ids and os.path.exists(args.ids):
        import json as _json
        allowed = {x["id"] for x in _json.load(open(args.ids, encoding="utf-8"))
                   if x.get("r", 0) >= args.min_r}
        print(f"  过滤白名单 {len(allowed)} 条（r>={args.min_r}）")
    ds = PairedDataset(NOISY, QUIET, clip_s=args.clip_s,
                       samples_per_clip=1 if args.smoke else 4, allowed=allowed)
    print(f"  pairs={len(ds.pairs)}  samples={len(ds)}  device={dev}")
    dl = DataLoader(ds, batch_size=args.batch, shuffle=not args.smoke, num_workers=2)

    model = CRN().to(dev)
    print(f"  params={sum(p.numel() for p in model.parameters())/1e3:.1f}k")
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)

    step = 0
    for ep in range(1, args.epochs + 1):
        t0 = time.time()
        total = 0.0
        pwr = 0.0
        for x, y in dl:
            x, y = x.to(dev), y.to(dev)
            spec = stft(x)                     # (B,2,F,T)
            out, _, _ = model(spec)
            pred = istft(out, x.shape[-1])
            loss = loss_fn(pred, y, x.shape[-1], dev)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            opt.step()
            total += float(loss)
            pwr += float(pred.pow(2).mean())
            step += 1
            if args.smoke:
                print(f"  step {step} loss {float(loss):.4f}")
                if step >= 30:
                    break
        n_steps = max(1, len(dl))
        print(f"  epoch {ep} loss {total/n_steps:.4f}  out_rms {np.sqrt(pwr/n_steps):.4f}  {time.time()-t0:.1f}s", flush=True)
        if args.smoke and step >= 30:
            break

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    torch.save({"model": model.state_dict()}, args.out)
    print(f"  saved {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
