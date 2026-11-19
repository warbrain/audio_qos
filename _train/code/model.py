"""双通道→单通道降噪模型（轻量 CRN，对标 H-GTCRN 的规模）。

设计依据（本仓库实测）：
  * 20 mm broadside 阵列，L/R 波形相关 0.9838 → 空间线索很弱，
    因此不做波束形成，只做「通道可信度加权 + 谱域掩码」。
  * R 通道比 L 响 1.1 dB 但单独使用时 CER 差 1.78 pp → 需要可学习的
    通道权重，而不是固定平均。
  * 非流式（HMI 侧整段处理），可用 GRU 获取长上下文。
"""
from __future__ import annotations

import torch
import torch.nn as nn


class ChannelFusion(nn.Module):
    """按 TF 单元学习 L/R 的可信度权重（softmax 归一化）。"""

    def __init__(self, feat: int = 32):
        super().__init__()
        self.score = nn.Sequential(
            nn.Conv2d(4, feat, 3, padding=1), nn.PReLU(),
            nn.Conv2d(feat, 1, 1),
        )

    def forward(self, spec: torch.Tensor) -> torch.Tensor:
        # spec: complex (B, 2, F, T)
        ri = torch.cat([spec.real, spec.imag], dim=1)          # (B,4,F,T)
        logits = self.score(ri)                                  # (B,1,F,T)
        w = torch.softmax(logits, dim=1)
        return w * spec[:, 0:1] + (1 - w) * spec[:, 1:2], w


class CRN(nn.Module):
    def __init__(self, ch: int = 32, gru: int = 96):
        super().__init__()
        self.fusion = ChannelFusion()
        self.enc = nn.ModuleList([
            nn.Sequential(nn.Conv2d(2, ch, 3, padding=1), nn.PReLU()),
            nn.Sequential(nn.Conv2d(ch, ch * 2, 3, stride=(2, 1), padding=1), nn.PReLU()),
        ])
        self.gru = nn.GRU(ch * 2, gru, batch_first=True, bidirectional=True)
        self.dec = nn.Sequential(
            nn.ConvTranspose2d(gru * 2, ch * 2, (2, 1), stride=(2, 1)),
            nn.PReLU(),
            nn.Conv2d(ch * 2, ch, 3, padding=1), nn.PReLU(),
            nn.Conv2d(ch, 2, 1),
        )

    def forward(self, spec: torch.Tensor):
        fused, w = self.fusion(spec)                             # (B,1,F,T)
        x0 = torch.cat([fused.real, fused.imag], dim=1)
        h = self.enc[0](x0)
        h = self.enc[1](h)                                       # (B,C,F/2,T)
        b, c, f, t = h.shape
        seq = h.permute(0, 2, 3, 1).reshape(b * f, t, c)
        seq, _ = self.gru(seq)
        h = seq.view(b, f, t, -1).permute(0, 3, 1, 2)
        m = self.dec(h)[:, :, : spec.shape[-2], :]               # (B,2,F,T)
        mask = torch.sigmoid(m[:, 0]) + 1j * torch.sigmoid(m[:, 1])
        return fused * mask, fused, w
