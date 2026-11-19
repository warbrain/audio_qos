"""定位增强输出为静音的原因：检查 STFT/ISTFT 往返与模型输出幅度。"""
import os
import sys

import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data import FS, istft, read_wav, stft  # noqa: E402
from model import CRN  # noqa: E402

dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 1) STFT/ISTFT 往返
x = torch.randn(2, FS, device=dev) * 0.1
spec = stft(x)
back = istft(spec, FS)
err = (back - x).abs().max().item()
print(f"  往返误差 max={err:.2e}   输入RMS={x.pow(2).mean().sqrt():.4f} 输出RMS={back.pow(2).mean().sqrt():.4f}")

# 2) 模型输出幅度
src = r"_asr_sim/spatial_dir/m5_2cm196"
f = sorted(f for f in os.listdir(src) if f.endswith("_L.wav"))[0]
l = read_wav(os.path.join(src, f))
r = read_wav(os.path.join(src, f.replace("_L.wav", "_R.wav")))
n = min(len(l), len(r), FS * 2)
import numpy as np
xx = torch.from_numpy(np.stack([l[:n], r[:n]])).unsqueeze(0).to(dev)

model = CRN().to(dev)
ck = torch.load("_train/code/ckpt_c.pt", map_location=dev)
model.load_state_dict(ck["model"])
model.eval()
with torch.no_grad():
    s = stft(xx)
    y, fused, w = model(s)
print(f"  输入谱 |spec| 均值 {s.abs().mean():.4f}")
print(f"  fused  |.| 均值 {fused.abs().mean():.4f}")
print(f"  输出谱 |y| 均值 {y.abs().mean():.6f}")
print(f"  mask 实部范围 [{y.real.min():.4f}, {y.real.max():.4f}]")
with torch.no_grad():
    out = istft(y, n)
print(f"  增强波形 RMS {out.pow(2).mean().sqrt():.6f}   峰值 {out.abs().max():.6f}")
