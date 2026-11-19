"""第 2 步：验证 4 路输出的工作状态，特别是 Out3/Out4。

两种模式：
  python step2_out34.py            # 测量：逐通道放音，看麦克风端电平与左右差异
  python step2_out34.py --audible  # 听力：依次播放并打印标签，由人确认听到哪一路

判定依据：
  * 某通道放音后麦克风电平明显高于静音基线 → 该通道确实在响
  * Out3/Out4 若在麦克风端无反应，说明音箱未接、接口未路由或端点点错
  * L-R 电平差与到达时差反映双麦对该声源的方向分辨能力
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C  # noqa: E402

NAMES = {0: "Out1", 1: "Out2(人工嘴)", 2: "Out3", 3: "Out4"}
AMP = 0.08
DURATION = 2.0
FADE = 0.15


def tone(n: int, fs: int, amp: float) -> np.ndarray:
    t = np.arange(n) / fs
    x = np.sin(2 * np.pi * 1000 * t)
    w = np.ones(n)
    f = int(FADE * fs)
    w[:f] = np.linspace(0, 1, f)
    w[-f:] = np.linspace(1, 0, f)
    return (amp * x * w).astype(np.float32)


def lag_estimate(a: np.ndarray, b: np.ndarray, max_lag: int = 6) -> int:
    a = a - a.mean()
    b = b - b.mean()
    if np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return 0
    best = (-1e18, 0)
    for lag in range(-max_lag, max_lag + 1):
        x = a[max(0, lag):len(a) + min(0, lag)]
        y = b[max(0, -lag):len(b) + min(0, -lag)]
        m = min(len(x), len(y))
        c = float(np.dot(x[:m], y[:m]))
        if c > best[0]:
            best = (c, lag)
    return best[1]


def measure(out_dev: int, in_dev: int) -> int:
    fs = C.PLAY_RATE
    n = int(DURATION * fs)
    sig = tone(n, fs, AMP)
    rec_frames = int((DURATION + 0.4) * C.RATE)

    print(f"  输出设备 {out_dev} / 输入设备 {in_dev}，1 kHz 音，幅度 {AMP}")
    print(f"\n  {'通道':<16s} {'L dBFS':>8s} {'R dBFS':>8s} {'L-R':>7s} {'lag':>5s}  判定")
    print("  " + "-" * 66)

    baseline = None
    results = {}
    for ch in range(4):
        buf = np.zeros((n, 4), dtype=np.float32)
        buf[:, ch] = sig
        # 回调式边播边录（sd.rec 在这块 UAC 上会读到全零）。
        try:
            l, r = C.capture_while_playing(buf, in_dev, out_dev, DURATION)
            rec = np.stack([l, r], axis=1).astype(np.float64)
            if C.is_silent(rec):
                print(f"  {NAMES[ch]:<16s} 采集全零 —— 链路假死，先断电重启 Scepter")
                continue
        except Exception as exc:
            print(f"  {NAMES[ch]:<16s} 采集失败: {exc}")
            continue

        a, b = int(0.6 * C.RATE), int(1.5 * C.RATE)
        seg = rec[a:b]
        lv = C.dbfs(seg[:, 0])
        rv = C.dbfs(seg[:, 1])
        lag = lag_estimate(seg[:, 0], seg[:, 1])
        results[ch] = (lv, rv)
        if baseline is None or lv < baseline[0]:
            baseline = (lv, rv)
        print(f"  {NAMES[ch]:<16s} {lv:8.1f} {rv:8.1f} {lv - rv:+7.2f} {lag:+5d}  —")
        time.sleep(0.3)

    print()
    print(f"  静音基线: L {baseline[0]:.1f} / R {baseline[1]:.1f} dBFS")
    print("  相对基线的提升：")
    verdict = {}
    for ch in sorted(results):
        lv, rv = results[ch]
        dl, dr = lv - baseline[0], rv - baseline[1]
        ok = max(dl, dr) > 6.0
        verdict[ch] = ok
        print(f"    {NAMES[ch]:<16s} L {dl:+6.1f} dB   R {dr:+6.1f} dB   "
              f"{'✅ 在响' if ok else '❌ 麦克风端无反应'}")

    print()
    print("  === 结论 ===")
    if verdict.get(1):
        print("  ✅ Out2（人工嘴）工作正常")
    else:
        print("  ❌ Out2（人工嘴）无反应 —— 采集链路有问题，先别做其它实验")
    for ch in (2, 3):
        if verdict.get(ch):
            print(f"  ✅ {NAMES[ch]} 有效，噪声可从该路注入")
        else:
            print(f"  ❌ {NAMES[ch]} 无效：检查音箱接线、接口控制面板中该路音量、以及 Windows 端点是否选错")
    return 0 if verdict.get(1) else 1


def audible(out_dev: int) -> int:
    fs = C.PLAY_RATE
    n = int(DURATION * fs)
    sig = tone(n, fs, 0.25)
    seq = [(2, "Out3"), (3, "Out4"), (2, 3), (1, "Out2 人工嘴（对照）")]
    print("  接下来依次播放 4 段，每段约 2 秒，请记住听到了什么、在哪个喇叭：")
    for i, item in enumerate(seq, 1):
        chs = item[0] if isinstance(item[0], tuple) else (item[0],)
        label = item[1] if len(item) > 1 else "Out3+Out4"
        print(f"    ▶ [{i}/4] {label}", flush=True)
        buf = np.zeros((n, 4), dtype=np.float32)
        for c in chs:
            buf[:, c] = sig
        C.play_block(buf, out_dev)
        time.sleep(0.8)
    print("  播放结束。请把听到的段号/位置反馈给上级，用以判断接线与路由是否正确。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--audible", action="store_true", help="播放有人声可辨的测试序列")
    args = ap.parse_args()

    out_dev, _ = C.pick_device("US-4x4HR", 4, 0, C.PLAY_RATE)
    in_dev, _ = C.pick_device("WhisPrompt", 0, 2, C.RATE)
    if out_dev is None or in_dev is None:
        print("设备缺失：out=", out_dev, " in=", in_dev)
        return 2
    if args.audible:
        return audible(out_dev)
    return measure(out_dev, in_dev)


if __name__ == "__main__":
    raise SystemExit(main())
