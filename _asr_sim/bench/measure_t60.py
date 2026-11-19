"""测量箱体混响时间 T60（支持任意输入设备）。

三种用法
--------
  --list-devices          列出所有可用输入/输出设备，方便挑
  --sim-check             用合成指数衰减校验估计器（不需要硬件）
  --hw                    放激励、录音、估 T60

关键参数
--------
  --in-device NAME|IDX    输入设备（名字片段或索引），默认自动找 WhisPrompt
  --out-device NAME|IDX   输出设备，默认自动找 US-4x4HR
  --in-channels N         输入通道数，默认取设备最大值（上限 8）
  --method noise|click    noise=中断噪声法（推荐，绕过包络问题）；click=10 ms 脉冲
  --play-dbfs            激励电平

测量原理：播 0.5 s 带限粉噪，激励停止后对其余信号做 Schroeder 反向积分，
取 -5 dB 与 -25 dB 的穿越时刻，T60 = 60 × Δt / 20。噪声底以上不足 10 dB 时报"动态范围不足"。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
import numpy as np

import common as C

PLAY_RATE = 48000


# ---------------------------------------------------------------- 设备
def list_devices() -> int:
    import sounddevice as sd

    print("=== 输出设备 ===")
    for i, d in enumerate(sd.query_devices()):
        if d["max_output_channels"] > 0:
            print(f"  [{i:2d}] out={d['max_output_channels']} sr={int(d['default_samplerate']):5d}  {d['name']}")
    print("\n=== 输入设备 ===")
    for i, d in enumerate(sd.query_devices()):
        if d["max_input_channels"] > 0:
            print(f"  [{i:2d}] in ={d['max_input_channels']} sr={int(d['default_samplerate']):5d}  {d['name']}")
    return 0


def resolve_device(spec: str | None, default_name: str, need: str) -> tuple[int, dict] | tuple[None, None]:
    """spec 可以是索引字符串，也可以是名字片段；None 表示按 default_name 自动找。"""
    import sounddevice as sd

    devices = list(sd.query_devices())

    def norm(s: str) -> str:
        # 设备名会随驱动变化："US-4x4HR" / "US-4x4 HR" / "US-HR Series Audio"
        return ''.join(ch for ch in s.lower() if ch.isalnum())

    if spec is not None and spec.strip():
        s = spec.strip()
        if s.isdigit():
            idx = int(s)
            d = devices[idx]
            ok = d["max_input_channels"] > 0 if need == "in" else d["max_output_channels"] > 0
            return (idx, d) if ok else (None, None)
        cands = [(i, d) for i, d in enumerate(devices) if norm(s) in norm(d["name"])]
    else:
        cands = [(i, d) for i, d in enumerate(devices) if norm(default_name) in norm(d["name"])]

    key = "max_input_channels" if need == "in" else "max_output_channels"
    cands = [(i, d) for i, d in cands if d[key] > 0]
    if not cands:
        return None, None
    # 优先 WASAPI（MME 是半双工，同时开输入输出会失败），其次通道数多的
    apis = sd.query_hostapis()

    def rank(item):
        api_name = apis[item[1]["hostapi"]]["name"].lower()
        prio = 0 if "wasapi" in api_name else (1 if "directsound" in api_name else 2)
        return (prio, -item[1][key], int(item[1]["default_samplerate"]))

    cands.sort(key=rank)
    return cands[0]


# ---------------------------------------------------------------- 采集
class _Rec:
    def __init__(self, frames: int, channels: int):
        self.frames = frames
        self.channels = channels
        self.buf = np.zeros((frames, channels), dtype=np.float32)
        self.done = 0

    def cb(self, indata, frames, time_info, status):  # noqa: ARG002
        n = min(frames, self.frames - self.done)
        if n > 0:
            self.buf[self.done:self.done + n] = indata[:n]
            self.done += n


def capture(block: np.ndarray, in_dev: int, out_dev: int, channels: int,
            fs_in: int, seconds: float) -> np.ndarray:
    import sounddevice as sd

    frames = int((seconds + 0.5) * fs_in)
    rec = _Rec(frames, channels)
    stream = sd.InputStream(device=in_dev, samplerate=fs_in, channels=channels,
                            dtype="float32", callback=rec.cb)
    stream.start()
    try:
        sd.play(block, PLAY_RATE, device=out_dev, blocking=True)
        time.sleep(0.4)
    finally:
        stream.stop()
        stream.close()
    return rec.buf[:max(rec.done, 1)]


# ---------------------------------------------------------------- 估计器
def _cross_time(edc: np.ndarray, fs: int, level_db: float) -> float | None:
    below = np.where(edc <= level_db)[0]
    if len(below) == 0:
        return None
    i = int(below[0])
    if i == 0:
        return 0.0
    y0, y1 = edc[i - 1], edc[i]
    if abs(y1 - y0) < 1e-9:
        return i / fs
    return (i - 1 + (level_db - y0) / (y1 - y0)) / fs


def fit_t60(edc: np.ndarray, fs: int, lo: float = -25.0, hi: float = -5.0) -> float | None:
    t_hi = _cross_time(edc, fs, hi)
    t_lo = _cross_time(edc, fs, lo)
    if t_hi is None or t_lo is None or t_lo <= t_hi:
        return None
    return float(60.0 * (t_lo - t_hi) / (hi - lo))


def decay_after_offset(x: np.ndarray, fs: int, burst_s: float,
                       guard_s: float = 0.02) -> tuple[float | None, dict]:
    """中断噪声法：用累积能量定位激励结束点，再对之后的信号做 Schroeder 积分。"""
    e = x.astype(np.float64) ** 2
    total = float(np.sum(e))
    if total <= 1e-20:
        return None, {"note": "全静音"}
    cum = np.cumsum(e) / total
    idx = np.where(cum >= 0.995)[0]
    if len(idx) == 0:
        return None, {"note": "能量分布异常"}
    end = int(idx[0])                      # 99.5% 能量累积点 = 激励结束
    start = end + int(guard_s * fs)
    if start >= len(x) - int(0.1 * fs):
        start = end - int(burst_s * fs)    # 兜底：从激励末尾开始
        if start < 0:
            start = 0
    seg = x[start:].astype(np.float64)
    if len(seg) < int(0.1 * fs):
        return None, {"note": "衰减窗口不足"}
    rev = np.cumsum((seg ** 2)[::-1])[::-1]
    edc = 10 * np.log10(rev / (rev[0] + 1e-20) + 1e-20)
    tail = seg[-int(0.3 * fs):]
    floor_db = 10 * np.log10(np.mean(tail ** 2) + 1e-20)
    head = seg[:int(0.05 * fs)]
    peak_db = 10 * np.log10(np.mean(head ** 2) + 1e-20)
    lo = max(-25.0, floor_db - peak_db + 10.0)
    diag = {"end_s": round(end / fs, 2), "peak_db": round(peak_db, 1),
            "floor_db": round(floor_db, 1), "range_db": round(peak_db - floor_db, 1)}
    if lo >= -5.0:
        diag["note"] = f"动态范围不足（{diag['range_db']} dB）"
        return None, diag
    return fit_t60(edc, fs, lo=lo, hi=-5.0), diag


def synthetic_rir(t60: float, fs: int = 16000, seconds: float = 2.0, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    n = int(seconds * fs)
    t = np.arange(n) / fs
    h = rng.standard_normal(n) * (10 ** (-3.0 * t / t60)) * 0.05
    h[0] += 1.0
    return h


def sim_check() -> int:
    print("=== 估计器自校验（合成指数衰减，真值精确已知）===")
    print(f"  {'真值':>6s} {'估计':>8s} {'误差':>8s}")
    print("  " + "-" * 26)
    ok = 0
    for true_t60 in (0.10, 0.20, 0.30, 0.50):
        h = synthetic_rir(true_t60, seed=int(true_t60 * 100))
        rev = np.cumsum((h ** 2)[::-1])[::-1]
        edc = 10 * np.log10(rev / (rev[0] + 1e-20) + 1e-20)
        est = fit_t60(edc, 16000)
        err = (est - true_t60) if est else float("nan")
        if est and abs(err) / true_t60 < 0.25:
            ok += 1
        print(f"  {true_t60:6.2f} {est if est else float('nan'):8.3f} {err:+8.3f}")
    print(f"\n  误差 <25%: {ok}/4")
    return 0 if ok >= 3 else 1


# ---------------------------------------------------------------- 激励
def noise_burst(fs: int, dur_s: float = 0.5, seed: int = 11) -> np.ndarray:
    n = int(dur_s * fs)
    rng = np.random.default_rng(seed)
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    spec[(f < 100) | (f > 7000)] = 0.0
    x = np.fft.irfft(spec, n)
    x = x / (np.sqrt(np.mean(x ** 2)) + 1e-12)
    fade = int(0.01 * fs)
    env = np.ones(n)
    env[:fade] = np.linspace(0, 1, fade)
    env[-fade:] = np.linspace(1, 0, fade)
    return (x * env).astype(np.float32)


def make_click(fs: int, dur_ms: float = 10.0, f0: float = 1000.0) -> np.ndarray:
    n = int(dur_ms / 1000 * fs)
    t = np.arange(n) / fs
    return (0.5 * np.sin(2 * np.pi * f0 * t) * np.hanning(n)).astype(np.float32)


# ---------------------------------------------------------------- 测量
def hw_measure(args) -> int:
    in_dev, in_info = resolve_device(args.in_device, "WhisPrompt", "in")
    out_dev, out_info = resolve_device(args.out_device, "US-4x4HR", "out")
    if in_dev is None or out_dev is None:
        print(f"  设备未找到：in={in_dev} out={out_dev}")
        print("  用 --list-devices 查看可用设备，或直接传索引/名字片段")
        return 2

    channels = min(args.in_channels or in_info["max_input_channels"], 8)
    fs_in = int(args.fs or in_info["default_samplerate"])
    print("=== 箱体 T60 测量 ===")
    print(f"  输入: [{in_dev}] {in_info['name']}  {channels} ch @ {fs_in} Hz")
    print(f"  输出: [{out_dev}] {out_info['name']}  通道 {args.out_channel} (人工嘴)")
    print(f"  方法: {args.method}  激励 {args.play_dbfs:+.0f} dBFS  {args.trials} 次\n")

    exc = (noise_burst(PLAY_RATE) if args.method == "noise" else make_click(PLAY_RATE))
    exc = exc * (10 ** (args.play_dbfs / 20.0))
    pre, post = int(0.3 * PLAY_RATE), int(2.5 * PLAY_RATE)
    block = np.zeros((pre + len(exc) + post, 4), dtype=np.float32)
    if args.out_channel >= block.shape[1]:
        print(f"  输出通道 {args.out_channel} 超出 4 通道范围")
        return 2
    block[pre:pre + len(exc), args.out_channel] = exc

    results: list[tuple[float, int]] = []
    for t in range(args.trials):
        buf = capture(block, in_dev, out_dev, channels, fs_in, len(block) / PLAY_RATE)
        if buf.size == 0:
            print(f"  第 {t+1:2d} 次: 无数据")
            continue
        # 自动挑响应最强的通道
        levels = [20 * np.log10(np.sqrt(np.mean(buf[:, c] ** 2)) + 1e-12)
                  for c in range(buf.shape[1])]
        best = int(np.argmax(levels))
        if np.max(np.abs(buf[:, best])) < 1e-7:
            print(f"  第 {t+1:2d} 次: 全通道静音（检查输入设备/增益）")
            continue
        if args.method == "noise":
            t60, diag = decay_after_offset(buf[:, best], fs_in, burst_s=len(exc) / PLAY_RATE)
        else:
            h = buf[:, best].astype(np.float64)
            pk = int(np.argmax(np.abs(h)))
            seg = h[pk:]
            rev = np.cumsum((seg ** 2)[::-1])[::-1]
            edc = 10 * np.log10(rev / (rev[0] + 1e-20) + 1e-20)
            t60, diag = fit_t60(edc, fs_in), {}
        if t == 0:
            lv = " ".join(f"ch{c}={v:.0f}" for c, v in enumerate(levels))
            print(f"  通道电平(dBFS): {lv}")
            if diag:
                print(f"  诊断: 峰值 {diag.get('peak_db')} / 底噪 {diag.get('floor_db')} dBFS "
                      f"→ 动态范围 {diag.get('range_db')} dB"
                      + (f"  {diag['note']}" if diag.get("note") else ""))
        if t60:
            results.append((t60, best))
        print(f"  第 {t+1:2d} 次: ch{best}  T60 {t60 if t60 else float('nan'):6.3f} s")
        time.sleep(0.15)

    print()
    if not results:
        print("  ❌ 没有有效的 T60 结果")
        print("     排查：1) 输入设备是否真的收到激励（看通道电平）")
        print("           2) 动态范围是否 ≥15 dB（底噪太高就加大 --play-dbfs）")
        print("           3) 若衰减曲线出现断崖，说明链路有静音门控，需绕过固件")
        return 1

    vals = np.array([v for v, _ in results])
    med = float(np.median(vals))
    print(f"  ✅ T60 中位 {med:.3f} s   四分位 {np.percentile(vals,25):.3f}~{np.percentile(vals,75):.3f} s"
          f"   （{len(vals)}/{args.trials} 次有效）")
    print()
    print("  对照仿真表（20 cm → 5 cm 的总功率提升）：")
    print("    T60 0.05 s → +8.2 dB ; 0.10 → +3.5 ; 0.20 → +0.9 ; 0.30 → +0.3")
    if med <= 0.10:
        print("  → 箱体接近无混响：缩短嘴距能拿到接近自由场的收益。")
    elif med <= 0.20:
        print("  → 中等混响：缩短嘴距收益有限（约 +1~3 dB），优先考虑吸声。")
    else:
        print("  → 混响偏强：先做吸声处理，缩短嘴距几乎无效。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="箱体混响时间 T60 测量")
    ap.add_argument("--list-devices", action="store_true")
    ap.add_argument("--sim-check", action="store_true")
    ap.add_argument("--hw", action="store_true")
    ap.add_argument("--in-device", default=None, help="输入设备名片段或索引；默认 WhisPrompt")
    ap.add_argument("--out-device", default=None, help="输出设备名片段或索引；默认 US-4x4HR")
    ap.add_argument("--in-channels", type=int, default=0, help="输入通道数，0=设备最大值(≤8)")
    ap.add_argument("--fs", type=int, default=0, help="输入采样率，0=设备默认")
    ap.add_argument("--out-channel", type=int, default=1, help="激励输出通道(0基)，1=Out2 人工嘴")
    ap.add_argument("--method", choices=["noise", "click"], default="noise")
    ap.add_argument("--play-dbfs", type=float, default=-20.0)
    ap.add_argument("--trials", type=int, default=8)
    args = ap.parse_args()

    if args.list_devices:
        return list_devices()
    if not args.sim_check and not args.hw:
        args.sim_check = True
    rc = 0
    if args.sim_check:
        rc |= sim_check()
        print()
    if args.hw:
        rc |= hw_measure(args)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
