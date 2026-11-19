"""训练（修复版）：精确配对数据 + 能量归一化损失 + SI-SDR + 验证集早停。

关键改动：
  1. 数据：干净录音 + 真实噪声数字混合（目标 = 干净录音本身）
  2. 损失：先做能量归一化，再加谱 L1 与 SI-SDR，**禁止靠缩放取胜**
  3. 验证集 + 早停 + 学习率衰减
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import torch
from torch.utils.data import DataLoader, random_split

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data2 import MixedPairDataset, istft, stft  # noqa: E402
from model2 import CRN  # noqa: E402

ROOT = "/mnt/d/App/cppApp/audio_qos/_asr_sim/spatial_dir"
CLEAN = os.path.join(ROOT, "train600_paired")
NOISE = os.path.join(ROOT, "noise_only.wav")


def si_sdr_loss(pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    """负 SI-SDR（逐样本计算后取平均）。"""
    p = pred - pred.mean(dim=-1, keepdim=True)
    t = target - target.mean(dim=-1, keepdim=True)
    a = (p * t).sum(dim=-1, keepdim=True) / (t.pow(2).sum(dim=-1, keepdim=True) + 1e-8)
    proj = a * t
    noise = p - proj
    ratio = (proj.pow(2).sum(dim=-1) + 1e-8) / (noise.pow(2).sum(dim=-1) + 1e-8)
    return -torch.mean(10 * torch.log10(ratio))


def norm_rms(x: torch.Tensor) -> torch.Tensor:
    r = x.pow(2).mean(dim=-1, keepdim=True).sqrt() + 1e-8
    return x / r


def loss_fn(pred: torch.Tensor, target: torch.Tensor, n: int) -> torch.Tensor:
    """不做 RMS 归一化：反相会让波形项翻倍，缩放也会被惩罚。

    之前的版本同时做了幅度谱 L1（尺度/极性强不变）与 SI-SDR（尺度不变），
    模型因此可以反相衰减而损失不变，导致 alpha=-0.055、CER 21.79%。
    """
    spec = (torch.abs(stft(pred)) - torch.abs(stft(target))).abs().mean()
    wave = (pred - target).abs().mean()
    return spec + 2.0 * wave


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--clip-s", type=float, default=2.0)
    ap.add_argument("--lr", type=float, default=5e-4)
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--out", default="_train/code/ckpt_e.pt")
    args = ap.parse_args()

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ds = MixedPairDataset(CLEAN, NOISE, clip_s=args.clip_s,
                          samples_per_clip=1 if args.smoke else 4)
    n_val = max(2, int(0.1 * len(ds)))
    tr, va = random_split(ds, [len(ds) - n_val, n_val],
                          generator=torch.Generator().manual_seed(0))
    print(f"  样本 {len(ds)}（验证 {n_val}）  device={dev}")
    tl = DataLoader(tr, batch_size=args.batch, shuffle=True, num_workers=0)
    vl = DataLoader(va, batch_size=args.batch, shuffle=False, num_workers=0)

    model = CRN().to(dev)
    print(f"  params={sum(p.numel() for p in model.parameters())/1e3:.1f}k")
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    sched = torch.optim.lr_scheduler.ReduceLROnPlateau(opt, factor=0.5, patience=3)

    best = float("inf")
    for ep in range(1, args.epochs + 1):
        t0 = time.time()
        model.train()
        tot = 0.0
        for x, y, _ in tl:
            x, y = x.to(dev), y.to(dev)
            spec = stft(x)
            out, _, w = model(spec)
            pred = istft(out, x.shape[-1])
            loss = loss_fn(pred, y, x.shape[-1])
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            opt.step()
            tot += float(loss.detach())
            if args.smoke and tot > 0:
                break
        model.eval()
        vsum = 0.0
        with torch.no_grad():
            for x, y, _ in vl:
                x, y = x.to(dev), y.to(dev)
                out, _, _ = model(stft(x))
                vsum += float(loss_fn(istft(out, x.shape[-1]), y, x.shape[-1]))
        vloss = vsum / max(1, len(vl))
        sched.step(vloss)
        print(f"  epoch {ep} train {tot/max(1,len(tl)):.4f}  val {vloss:.4f}  "
              f"lr {opt.param_groups[0]['lr']:.1e}  {time.time()-t0:.1f}s", flush=True)
        if vloss < best:
            best = vloss
            os.makedirs(os.path.dirname(args.out), exist_ok=True)
            torch.save({"model": model.state_dict()}, args.out)
        if args.smoke:
            break

    print(f"  best val {best:.4f}  saved {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
