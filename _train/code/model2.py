"""双通道→单通道降噪模型（修复版）。

相对旧版 model.py 的两处关键修正：
  1. ChannelFusion 输出 **2 个通道** 的 logits，softmax 才有意义
     （旧版只输出 1 通道，softmax 恒为 1.0，等价于"只取左通道"）。
  2. 掩码改为 **tanh 有界复数掩码**，实部/虚部均落在 [-1.5, 1.5]，
     可以表示相位反转与 >1 的增益；旧版 sigmoid 锁死在 [0,1]，只能衰减。

设计依据（本仓库实测）：
  * 20 mm broadside 阵列，L/R 波形相关 0.9838 → 空间线索弱，
    不做波束形成，只做「通道可信度加权 + 谱域掩码」。
  * R 通道更响但单独使用 CER 差 1.78 pp → 需要可学习的逐 TF 权重。
  * 非流式（HMI 侧整段处理），可用 GRU 获取长上下文。
"""
from __future__ import annotations

import torch
import torch.nn as nn

MASK_SCALE = 1.5


class ChannelFusion(nn.Module):
    """按 TF 单元学习 L/R 的可信度权重（2 通道 softmax）。"""

    def __init__(self, feat: int = 32):
        super().__init__()
        self.score = nn.Sequential(
            nn.Conv2d(4, feat, 3, padding=1), nn.PReLU(),
            nn.Conv2d(feat, 2, 1),          # ← 2 通道：每个输入通道一个 logit
        )

    def forward(self, spec: torch.Tensor):
        ri = torch.cat([spec.real, spec.imag], dim=1)          # (B,4,F,T)
        logits = self.score(ri)                                  # (B,2,F,T)
        w = torch.softmax(logits, dim=1)                         # 沿通道维归一化
        fused = w[:, 0:1] * spec[:, 0:1] + w[:, 1:2] * spec[:, 1:2]
        return fused, w


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
        # 幅度严格在 [0,1]（不许放大/反相）；相位限制 ±45°，避免任意旋转
        mag = torch.sigmoid(m[:, 0])
        ang = 0.25 * torch.pi * torch.tanh(m[:, 1])
        mask = torch.polar(mag, ang)
        return fused * mask, fused, w
