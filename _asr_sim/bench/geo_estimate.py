"""几何估计：从采集信号里能恢复出哪些距离/角度？

三个可用的物理量
----------------
1. **TDOA（到达时差）** → 方位角。
   20 mm 阵列、16 kHz 下最大时延 0.93 样本；侧向（broadside）正前方时延为 0，
   此时阵列**无法分辨**该方向 —— 这也是空间滤波在箱体里吃力一截的物理原因。
2. **电平** → 距离比。
   自由场按 1/r 衰减（−6 dB 每加倍距离）。绝对距离需要标定（声源强度 + 麦克风灵敏度），
   所以只能恢复**相对**距离；有已知参考点（人工嘴 5 cm）时才能推绝对值。
3. **DRR（直达/混响比）** → 与距离单调相关，但还受房间常数影响，只能做相对判断。

本脚本
------
  python geo_estimate.py --mode sim     # 已知真值的仿真，验证估计器
  python geo_estimate.py --mode hw      # 套到真实采集上，反推距离

仿真会把真值与估计值并列，直接看出哪个量可辨识、误差多大。
"""
from __future__ import annotations

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
import numpy as np

import simcore as S


# ---------------------------------------------------------------- 估计器
def gcc_phat(a: np.ndarray, b: np.ndarray, fs: int = S.FS,
             max_delay: float = 32.0) -> tuple[float, float]:
    """广义互相关（相位变换）求时延，返回 (delay_samples, peak_confidence)。

    先用 GCC-PHAT 得到整数峰，再用抛物线插值求亚样本。
    """
    n = 1 << int(math.ceil(math.log2(len(a) + len(b))))
    A = np.fft.rfft(a, n)
    B = np.fft.rfft(b, n)
    R = A * np.conj(B)
    mag = np.abs(R) + 1e-12
    R /= mag
    cc = np.fft.irfft(R, n)
    cc = np.concatenate([cc[-n // 2:], cc[:n // 2]])
    lags = np.arange(-n // 2, n // 2)
    limit = int(max_delay)
    mask = np.abs(lags) <= limit
    cc_m, lag_m = cc[mask], lags[mask]
    k = int(np.argmax(cc_m))
    conf = float(cc_m[k])
    if 0 < k < len(cc_m) - 1:
        y0, y1, y2 = cc_m[k - 1], cc_m[k], cc_m[k + 1]
        denom = y0 - 2 * y1 + y2
        frac = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-12 else 0.0
    else:
        frac = 0.0
    return float(lag_m[k] + frac), conf


def delay_to_angle_deg(delay_samples: float, spacing: float = 0.020,
                       fs: int = S.FS) -> float | None:
    """时延 → 与阵列法线的夹角（度）。超出物理上限时返回 None。"""
    d = delay_samples / fs                        # 秒
    max_d = spacing / 343.0
    if abs(d) > max_d * 1.05:
        return None
    return math.degrees(math.asin(max(-1.0, min(1.0, d / max_d))))


def direct_to_reverb_db(sig: np.ndarray, onset: int, direct_ms: float = 3.0,
                        fs: int = S.FS) -> float:
    """直达声与混响尾部的能量比（dB）。onset 为直达声起点样本。"""
    d_len = max(1, int(direct_ms * fs / 1000))
    seg_d = sig[onset:onset + d_len]
    seg_r = sig[onset + d_len:]
    pd = float(np.sum(seg_d.astype(np.float64) ** 2)) + 1e-20
    pr = float(np.sum(seg_r.astype(np.float64) ** 2)) + 1e-20
    return 10 * math.log10(pd / pr)


def estimate_distance_from_level(rec_dbfs: float, play_dbfs: float,
                                 ref_distance_m: float, ref_coupling_db: float) -> float:
    """用已知参考点推算距离：coupling = rec - play，距离按 1/r 反比。"""
    coupling = rec_dbfs - play_dbfs
    delta_db = ref_coupling_db - coupling          # 比参考点弱多少
    return ref_distance_m * (10 ** (delta_db / 20.0))


# ---------------------------------------------------------------- 仿真验证
def sim_case(layout: str, distance: float, azimuth: float | None,
             t60: float = 0.30) -> dict:
    """在一个已知几何下渲染脉冲，估计 TDOA / DRR / 相对距离。"""
    cfg = S.SimConfig(layout=layout, distance=distance, t60=t60,
                      snr_db=None, noise_azimuth_deg=None)
    cfg.validate()
    room = S._make_room(cfg)
    impulse = np.zeros(int(0.5 * S.FS))
    impulse[0] = 1.0
    room.sources[0].signal = impulse
    room.sources[0].add_signal = False
    room.simulate()
    sig = room.mic_array.signals[:2]
    l, r = sig[0], sig[1]

    delay, conf = gcc_phat(l, r)
    onset = int(np.argmax(np.abs(l) > 0.05 * np.max(np.abs(l))))
    drr = direct_to_reverb_db(l, onset)
    true_delay = S.theoretical_delay_samples(cfg)
    return {
        "layout": layout, "distance_m": distance, "t60": t60,
        "direct_peak": round(float(np.max(np.abs(l))), 6),
        "tdoa_true": round(true_delay, 3),
        "tdoa_est": round(delay, 3),
        "tdoa_err": round(delay - true_delay, 3),
        "conf": round(conf, 4),
        "angle_deg": None if delay_to_angle_deg(delay) is None else round(delay_to_angle_deg(delay), 1),
        "drr_db": round(drr, 1),
    }


def run_sim() -> int:
    print("=== 仿真：TDOA / DRR 的可辨识性 ===")
    print("真值来自已知几何；估计只用 L/R 两路信号。\n")
    rows = []
    for layout in ("endfire", "broadside"):
        for dist in (0.05, 0.10, 0.20):
            rows.append(sim_case(layout, dist, None))
    print(f"{'布局':<11s} {'距离':>6s} {'理论时延':>9s} {'估计时延':>9s} {'误差':>7s} "
          f"{'角度':>7s} {'DRR':>8s}")
    print("-" * 68)
    for r in rows:
        ang = "—" if r["angle_deg"] is None else f"{r['angle_deg']:+.1f}°"
        print(f"{r['layout']:<11s} {r['distance_m']*100:5.0f}cm {r['tdoa_true']:+9.3f} "
              f"{r['tdoa_est']:+9.3f} {r['tdoa_err']:+7.3f} {ang:>7s} {r['drr_db']:7.1f}dB")
    print("\n结论：")
    print("  * endfire 的时延与时距无关（只与阵列几何有关），所以 TDOA 只能给**角度**，给不了距离；")
    print("  * broadside 正前方时延恒为 0 —— 该方向**不可分辨**，这解释了箱体里阵列吃力的原因；")
    print("  * DRR 随距离单调下降（越远混响占比越高），但还受 T60 影响，只能做相对比较。")
    return 0


# ---------------------------------------------------------------- 真实采集
def run_hw() -> int:
    print("=== 真实采集：反推距离 ===")
    print("方法：以人工嘴 10 cm 为参考点，用 1/r 反比推房间音箱距离。\n")

    # 2026-11-16 实测（step2_out34 与耦合测量）
    measurements = [
        ("人工嘴 Out2", -30.0, -22.5),   # 语音 −30 dBFS 播放 → 采集 −22.5 dBFS
        ("房间音箱 Out3", -22.0, -47.8),  # 1 kHz 音 −22 dBFS → 采集 −47.8
        ("房间音箱 Out4", -22.0, -47.6),
    ]
    ref_name, ref_play, ref_rec = measurements[0]
    ref_coupling = ref_rec - ref_play
    ref_distance = 0.05

    print(f"{'声源':<16s} {'播放':>8s} {'采集':>8s} {'耦合':>8s} {'估计距离':>10s}")
    print("-" * 56)
    print(f"{ref_name:<16s} {ref_play:7.1f} {ref_rec:7.1f} {ref_coupling:+8.1f} "
          f"{ref_distance*100:8.0f}cm  (参考)")
    for name, play, rec in measurements[1:]:
        d = estimate_distance_from_level(rec, play, ref_distance, ref_coupling)
        print(f"{name:<16s} {play:7.1f} {rec:7.1f} {rec-play:+8.1f} {d*100:8.0f}cm")

    print("\n注意：这是**等效距离**（把音箱换成点声源、同轴放置时的距离）。")
    print("      房间音箱的真实几何距离还受指向性、朝向、遮挡影响，通常会比等效距离更近。")
    print("\n=== TDOA：真实箱体的方位 ===")
    print("  2026-11-16 实测：L/R 时延 −0.125 样本（相关系数 0.9995）")
    ang = delay_to_angle_deg(-0.125)
    print(f"  换算角度：{ang:+.1f}°" if ang is not None else "  超出物理上限，无效")
    print("  → 人工嘴几乎正对阵列法线，属于 broadside 正前方，该方向时延≈0、角度不可分辨。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["sim", "hw", "both"], default="both")
    args = ap.parse_args()
    if args.mode in ("sim", "both"):
        run_sim()
        print()
    if args.mode in ("hw", "both"):
        run_hw()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
