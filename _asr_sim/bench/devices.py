"""设备发现与通道路由自检。

历史上最浪费时间的两个问题都在这里兜住：
  1. "找不到 Out3/Out4"：同名 `Speakers (US-4x4HR)` 可能有 4 个端点（索引 5/12/15/32），
     接线的那个才是真的。本模块逐个探测每个通道的实际声学响应，而不是只看枚举结果。
  2. "采到全零"：链路假死时 UAC 输出数字静音。所有采集都要经过 `assert_not_silent`。
"""
from __future__ import annotations

import numpy as np

import bootstrap  # noqa: F401  (必须先于任何输出导入)
import common as C

OUT_NAME = "US-4x4HR"
IN_NAME = "WhisPrompt"
CHANNELS = {0: "Out1", 1: "Out2/人工嘴", 2: "Out3/左", 3: "Out4/右"}


# ---------------------------------------------------------------- 发现
def outputs() -> list[tuple[int, dict]]:
    return [(i, d) for i, d in C.list_candidates(OUT_NAME) if d["max_output_channels"] >= 4]


def inputs() -> list[tuple[int, dict]]:
    return [(i, d) for i, d in C.list_candidates(IN_NAME) if d["max_input_channels"] >= 2]


def pick_pair() -> tuple[int, int]:
    out_dev, _ = C.pick_device(OUT_NAME, 4, 0, C.PLAY_RATE)
    in_dev, _ = C.pick_device(IN_NAME, 0, 2, C.RATE)
    return out_dev, in_dev


# ---------------------------------------------------------------- 声学探测
def _tone(n: int, amp: float) -> np.ndarray:
    fs = C.PLAY_RATE
    t = np.arange(n) / fs
    x = np.sin(2 * np.pi * 1000 * t)
    fade = int(0.15 * fs)
    w = np.ones(n)
    w[:fade] = np.linspace(0, 1, fade)
    w[-fade:] = np.linspace(1, 0, fade)
    return (amp * x * w).astype(np.float32)


def _level(block: np.ndarray, out_dev: int, in_dev: int, seconds: float) -> tuple[float, float]:
    l, r = C.capture_while_playing(block, in_dev, out_dev, seconds)
    a, b = int(0.6 * C.RATE), int(1.5 * C.RATE)
    seg_l = l[a:b] if len(l) > b else l
    seg_r = r[a:b] if len(r) > b else r
    return C.dbfs(seg_l), C.dbfs(seg_r)


def probe_channels(seconds: float = 2.0, amp: float = 0.08) -> dict:
    """逐个通道放 1 kHz 音，返回每个通道相对静音基线的提升。"""
    out_dev, in_dev = pick_pair()
    if out_dev is None or in_dev is None:
        raise RuntimeError("设备缺失：需要 4 通道 US-4x4HR 输出和 2 通道 WhisPrompt 输入")

    n = int(seconds * C.PLAY_RATE)
    silence = np.zeros((n, 4), dtype=np.float32)
    base_l, base_r = _level(silence, out_dev, in_dev, seconds)

    result = {"out_dev": out_dev, "in_dev": in_dev,
              "baseline": {"L": base_l, "R": base_r}, "channels": {}}
    for ch in range(4):
        buf = np.zeros((n, 4), dtype=np.float32)
        buf[:, ch] = _tone(n, amp)
        lv, rv = _level(buf, out_dev, in_dev, seconds)
        result["channels"][ch] = {
            "name": CHANNELS[ch],
            "L": round(lv, 1), "R": round(rv, 1),
            "delta_db": round(max(lv - base_l, rv - base_r), 1),
        }
    return result


def verify_noise_path(min_delta_db: float = 6.0) -> tuple[bool, str]:
    """确认 Out3/Out4 的声音能到达麦克风。返回 (是否通过, 说明)。"""
    try:
        info = probe_channels(seconds=1.6)
    except Exception as exc:
        return False, f"探测失败: {exc}"
    ch2 = info["channels"][2]["delta_db"]
    ch3 = info["channels"][3]["delta_db"]
    ok = ch2 >= min_delta_db and ch3 >= min_delta_db
    if ok:
        return True, f"Out3 +{ch2} dB / Out4 +{ch3} dB（阈值 {min_delta_db} dB）"
    return False, (
        f"Out3 +{ch2} dB / Out4 +{ch3} dB，低于阈值 {min_delta_db} dB。"
        "请检查：① 音箱是否接到接口 OUT3/OUT4；② 接口控制面板该路音量；"
        "③ Windows 端点是否选错（同名设备可能有 4 个实例）；④ 隔离门是否关死")


def assert_not_silent(wave_l: np.ndarray, context: str = "采集") -> None:
    """链路假死时 UAC 输出数字静音，必须在送 ASR 之前拦住。"""
    if C.is_silent(wave_l):
        raise RuntimeError(
            f"{context}得到全零音频：ESP-NOW 链路假死。"
            "请断电重启 Scepter（不是软复位）后重试。")
