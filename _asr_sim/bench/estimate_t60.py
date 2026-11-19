"""从安静条件下的采集估计箱体混响时间 T60。

方法：找语音结束点，做 Schroeder 反向能量积分得到衰减曲线，
在 -5 到 -25 dB 区间线性拟合斜率，T60 = 60 / 斜率(dB/s)。
"""
import os
import sys
import numpy as np

sys.path.insert(0, r"D:\App\cppApp\audio_qos\_asr_sim\bench")
import simcore as S

FS = 16000


def t60_from_file(path: str) -> float | None:
    x, fs = S.load_wav_mono(path)
    if fs != FS:
        x = S.resample_linear(x, fs, FS)
    # 20 ms 帧能量
    fr = int(0.02 * FS)
    n = len(x) // fr
    if n < 20:
        return None
    e = np.array([np.mean(x[i * fr:(i + 1) * fr] ** 2) for i in range(n)])
    e = e + 1e-20
    edb = 10 * np.log10(e)
    peak = int(np.argmax(edb))
    tail = edb[peak:]
    if len(tail) < 30:
        return None
    # 反向积分（简化版：从尾部向前累积）
    energy = np.cumsum(e[peak:][::-1])[::-1]
    edc = 10 * np.log10(energy / energy[0] + 1e-20)
    # 取 -5 ~ -25 dB 区间拟合
    idx = np.where((edc <= -5) & (edc >= -25))[0]
    if len(idx) < 8:
        return None
    t = idx * 0.02
    slope, _ = np.polyfit(t, edc[idx], 1)
    if slope >= -0.5:
        return None
    return float(-60.0 / slope)


def main() -> int:
    for d in sys.argv[1:]:
        files = sorted(f for f in os.listdir(d) if f.endswith("_L.wav"))
        vals = []
        for f in files:
            v = t60_from_file(os.path.join(d, f))
            if v and 0.01 < v < 5.0:
                vals.append(v)
        if vals:
            print(f"  {os.path.basename(d):24s} {len(vals):3d}/{len(files):3d} 条可用  "
                  f"T60 中位 {np.median(vals):.3f} s  四分位 "
                  f"{np.percentile(vals,25):.3f}~{np.percentile(vals,75):.3f}")
        else:
            print(f"  {os.path.basename(d):24s} 无法估计（噪声/衰减不足）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
