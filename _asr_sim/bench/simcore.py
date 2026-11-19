"""pyroomacoustics 仿真的核心实现（不依赖任何声卡/串口）。

设计要点
--------
1. **几何必须可配置**。2026-11-16 对真实箱体实测：两麦 L/R 时延 −0.125 样本、
   相关系数 0.9995 → 物理上是 **broadside**（人工嘴在阵列垂直方向）。
   旧脚本假设 endfire（时延 0.93 样本），那批仿真结论不代表硬件。
2. **信噪比用"两次渲染 + 实测功率混合"** 精确控制：
   先只渲染语音、再只渲染噪声，按语音窗/噪声窗实测功率缩放后相加。
   这样不受房间衰减、混响尾巴影响。
3. 输出与真实采集完全同构：`<id>_L.wav`、`<id>_R.wav`，16 kHz 单声道。
"""
from __future__ import annotations

import json
import math
import os
import wave
from dataclasses import dataclass, asdict

import numpy as np

FS = 16000
ROOM = np.array([1.0, 1.0, 1.0])          # 1 m 立方体
CENTER = np.array([0.5, 0.5, 0.5])
SPEECH_LEVEL_DBFS = -30.0

LAYOUTS = {
    "broadside": "两麦沿 x 轴，人工嘴在 +y（实测布局，L/R 时延 ≈ 0）",
    "endfire": "两麦沿 y 轴，人工嘴在 +y 延长线（旧假设，时延 ≈ spacing/c）",
}


@dataclass
class SimConfig:
    layout: str = "broadside"
    spacing: float = 0.020           # 两麦间距 m
    distance: float = 0.05           # 人工嘴到阵列中心 m（实测 5 cm）
    t60: float = 0.30
    snr_db: float | None = None      # 指定目标 SNR（会归一化噪声）
    noise_level_dbfs: float | None = None    # 固定噪声源电平，SNR 由几何/距离决定
    noise_azimuth_deg: float | None = 90.0   # 0=正前方, 90=正侧方
    noise_distance: float = 0.30
    seed: int = 7
    mic_gain_db: tuple[float, float] = (0.0, 0.0)   # 两麦灵敏度失配（实测 L 低 1.54 dB）
    max_order_cap: int = 20

    def validate(self) -> None:
        if self.layout not in LAYOUTS:
            raise ValueError(f"layout 必须是 {list(LAYOUTS)}")


def mic_positions(layout: str, spacing: float) -> np.ndarray:
    """返回 (3, 2) 麦克风坐标。"""
    if layout == "endfire":
        return np.array([CENTER + np.array([0.0, -spacing / 2, 0.0]),
                         CENTER + np.array([0.0, +spacing / 2, 0.0])]).T
    return np.array([CENTER + np.array([-spacing / 2, 0.0, 0.0]),
                     CENTER + np.array([+spacing / 2, 0.0, 0.0])]).T


def theoretical_delay_samples(cfg: SimConfig) -> float:
    """理论 L/R 时延：端射 = spacing/c，侧向 = 0。"""
    return (cfg.spacing / 343.0) * FS if cfg.layout == "endfire" else 0.0


def _make_room(cfg: SimConfig):
    # 惰性导入：pyroomacoustics 的 libroom DLL 会被 Application Control 拦截，
    # 而绝大多数脚本只用本模块的 WAV/清单/评分工具，不该因此无法 import。
    import pyroomacoustics as pra

    absorption, order = pra.inverse_sabine(cfg.t60, ROOM)
    room = pra.ShoeBox(ROOM, fs=FS, materials=pra.Material(absorption),
                       max_order=min(order, cfg.max_order_cap), air_absorption=True)
    room.add_microphone_array(pra.MicrophoneArray(mic_positions(cfg.layout, cfg.spacing), FS))

    mouth = CENTER + np.array([0.0, cfg.distance, 0.0])
    aim = CENTER - mouth
    aim = aim / (np.linalg.norm(aim) + 1e-12)
    room.add_source(mouth, directivity=pra.CardioidFamily(orientation=aim, p=0.5))
    return room


def _noise_position(cfg: SimConfig) -> np.ndarray | None:
    if cfg.noise_azimuth_deg is None:
        return None
    th = math.radians(cfg.noise_azimuth_deg)
    pos = CENTER + np.array([cfg.noise_distance * math.sin(th),
                             cfg.noise_distance * math.cos(th), 0.0])
    if np.any(pos < 0.02) or np.any(pos > 0.98):
        return None
    return pos


def _render(room, signals: dict[int, np.ndarray], length: int) -> np.ndarray:
    """把给定信号挂到对应声道号上渲染，返回 (2, length)。

    未使用的源必须给一个空信号：pyroomacoustics 的 simulate() 会对所有源取 len()，
    signal=None 会直接抛 TypeError。
    """
    for idx, src in enumerate(room.sources):
        if idx in signals:
            src.signal = signals[idx]
        else:
            src.signal = np.zeros(1)
        src.add_signal = False
    room.simulate()
    out = room.mic_array.signals
    if out.shape[0] < 2:
        raise RuntimeError("仿真输出不足 2 通道")
    ch = out[:2]
    if ch.shape[1] < length:
        ch = np.pad(ch, ((0, 0), (0, length - ch.shape[1])))
    return ch[:, :length]


def render_clip(speech: np.ndarray, cfg: SimConfig, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray, dict]:
    """渲染一条，返回 (L, R, 统计)。

    两种噪声控制方式：
      * cfg.snr_db          —— 渲染后把噪声缩放到目标 SNR（用于固定 SNR 的对比）
      * cfg.noise_level_dbfs —— 噪声源电平固定，SNR 由声源到麦克风的距离决定
                                （用于"嘴距变化"这类几何实验）
    """
    n = len(speech)
    sp = _render(_make_room(cfg), {0: speech}, n)

    noise_pos = _noise_position(cfg)
    nz = np.zeros_like(sp)
    want_noise = noise_pos is not None and (cfg.snr_db is not None or cfg.noise_level_dbfs is not None)
    if want_noise:
        room_n = _make_room(cfg)
        room_n.add_source(noise_pos)
        amp = 1.0 if cfg.noise_level_dbfs is None else 10 ** (cfg.noise_level_dbfs / 20.0)
        raw_noise = rng.standard_normal(n) * amp
        nz = _render(room_n, {1: raw_noise}, n)

        a, b = int(0.6 * FS), int(1.6 * FS)
        sp_p = float(np.mean(sp[0, a:b] ** 2)) + 1e-20
        tail = slice(max(0, n - int(0.8 * FS)), n)
        nz_p = float(np.mean(nz[0, tail] ** 2)) + 1e-20

        if cfg.snr_db is not None:
            # 归一化到目标 SNR
            scale = math.sqrt((sp_p / (10 ** (cfg.snr_db / 10.0))) / nz_p)
            nz = nz * scale
        # noise_level_dbfs 模式不缩放：SNR 由几何决定

    # 统一归一化：低吸收的小房间镜像源会发散（峰值可达数十），
    # 但语音与噪声用**同一个标量**归一化，两者比值（即 SNR）保持不变。
    a, b = int(0.6 * FS), int(1.6 * FS)
    ref = float(np.sqrt(np.mean(sp[0, a:b] ** 2))) + 1e-20
    scale = (10 ** (-30.0 / 20.0)) / ref
    sp = sp * scale
    nz = nz * scale

    mixed = sp + nz

    gains = [10 ** (g / 20.0) for g in cfg.mic_gain_db]
    mixed[0] *= gains[0]
    mixed[1] *= gains[1]

    peak = float(np.max(np.abs(mixed))) if mixed.size else 0.0
    if peak > 0.99:
        mixed *= 0.99 / peak

    # 实测 SNR（语音窗 vs 噪声窗，均来自混合后的左声道）
    a, b = int(0.6 * FS), int(1.6 * FS)
    tail = slice(max(0, n - int(0.8 * FS)), n)
    sp_p = float(np.mean(mixed[0, a:b] ** 2)) + 1e-20
    nz_p = float(np.mean(mixed[0, tail] ** 2)) + 1e-20
    stats = {
        "speech_rms_dbfs": round(20 * math.log10(float(np.sqrt(np.mean(mixed[0] ** 2))) + 1e-12), 1),
        "peak_dbfs": round(20 * math.log10(float(np.max(np.abs(mixed))) + 1e-12), 1),
        "snr_db": round(10 * math.log10(max(sp_p - nz_p, 1e-20) / nz_p), 1) if want_noise else None,
    }
    return mixed[0], mixed[1], stats


def write_wav(path: str, x: np.ndarray) -> None:
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(np.clip(x * 32767, -32768, 32767).astype("<i2").tobytes())


def measure_delay(l: np.ndarray, r: np.ndarray, max_lag: int = 8) -> int:
    """整数样本 L/R 时延，用于几何自检。"""
    a, b = l - l.mean(), r - r.mean()
    if np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return 0
    best = (-1e18, 0)
    for lag in range(-max_lag, max_lag + 1):
        xa = a[max(0, lag):len(a) + min(0, lag)]
        xb = b[max(0, -lag):len(b) + min(0, -lag)]
        m = min(len(xa), len(xb))
        c = float(np.dot(xa[:m], xb[:m]))
        if c > best[0]:
            best = (c, lag)
    return best[1]


def render_dataset(items: list[dict], out_dir: str, cfg: SimConfig,
                   verbose: bool = True) -> dict:
    """渲染整批，写出 wav + capture.json，返回汇总。"""
    cfg.validate()
    os.makedirs(out_dir, exist_ok=True)
    rng = np.random.default_rng(cfg.seed)
    rows = []
    for it in items:
        x, fs = load_wav_mono(it["audio"])
        x = resample_linear(x, fs, FS)
        rms = float(np.sqrt(np.mean(x ** 2))) + 1e-12
        speech = x * (10 ** (SPEECH_LEVEL_DBFS / 20.0) / rms)
        speech = np.concatenate([np.zeros(int(0.6 * FS)), speech, np.zeros(int(1.0 * FS))])

        l, r, stats = render_clip(speech, cfg, rng)
        write_wav(os.path.join(out_dir, f"{it['id']}_L.wav"), l)
        write_wav(os.path.join(out_dir, f"{it['id']}_R.wav"), r)
        row = {"id": it["id"], "reference": it["reference"], "source": it["audio"],
               "config": asdict(cfg), **stats}
        rows.append(row)
        if verbose:
            print(json.dumps({k: row[k] for k in ("id", "speech_rms_dbfs", "peak_dbfs")},
                             ensure_ascii=False), flush=True)

    with open(os.path.join(out_dir, "capture.json"), "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    return {"dir": out_dir, "clips": len(rows), "config": asdict(cfg)}


# ---------------------------------------------------------------- wav 工具
def load_wav_mono(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as w:
        fs, ch, n = w.getframerate(), w.getnchannels(), w.getnframes()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    return x, fs


def resample_linear(x: np.ndarray, fs_in: int, fs_out: int) -> np.ndarray:
    if fs_in == fs_out:
        return x
    n_out = int(round(len(x) * fs_out / fs_in))
    t_in = np.arange(len(x)) / fs_in
    t_out = np.arange(n_out) / fs_out
    return np.interp(t_out, t_in, x)


DATASET = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5"
# v6 扩容集：392 条留出集（v5 的 196 条是其子集）。默认仍用 v5，避免破坏既有脚本；
# 需要 v6 时显式传 dataset=... 或设置环境变量 DUAL_END_DATASET。
DATASET_V6 = r"D:\App\cppApp\audio_qos\qwen3-asr-strict-v6"


def load_manifest(n: int = 24, start: int = 0, dataset: str | None = None,
                  manifest_path: str | None = None) -> list[dict]:
    """读取留出集清单，返回 [{id, audio, reference, speaker}]。

    dataset       : 数据集目录，默认取环境变量 DUAL_END_DATASET 或 v5。
    manifest_path : 直接指定清单文件（优先于 dataset）。
    """
    if manifest_path:
        raw = json.load(open(manifest_path, encoding="utf-8"))
        recs = raw["recordings"] if isinstance(raw, dict) else raw
        base = os.path.dirname(manifest_path)
        items = []
        for c in recs[start:start + n]:
            a = c.get("audio") or c.get("wav") or c.get("path")
            p = a if os.path.isabs(a) else os.path.join(base, a.replace("/", os.sep))
            items.append({"id": c.get("id") or os.path.splitext(os.path.basename(p))[0],
                          "audio": p,
                          "reference": c.get("reference", ""),
                          "speaker": c.get("speaker", "")})
        return items

    root = dataset or os.environ.get("DUAL_END_DATASET") or DATASET
    version = "v6" if root.rstrip("\\/").endswith("strict-v6") else "v5"
    mf = json.load(open(os.path.join(root, f"heldout-manifest.strict-{version}.json"), encoding="utf-8"))
    items = []
    for c in mf["recordings"][start:start + n]:
        p = c["audio"] if os.path.isabs(c["audio"]) else os.path.join(root, c["audio"].replace("/", os.sep))
        if not os.path.exists(p):
            alt = os.path.join(root, "audio", os.path.basename(p))
            if os.path.exists(alt):
                p = alt
        items.append({"id": os.path.splitext(os.path.basename(p))[0],
                      "audio": p,
                      "reference": c["reference"],
                      "speaker": c.get("speaker", "")})
    return items
