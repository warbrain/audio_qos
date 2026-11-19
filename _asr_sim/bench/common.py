"""共享工具：设备选择、录音/放音、ASR 评测、静音防呆。

环境要点（都是踩过的坑）：
* WhisPrompt UAC 必须用**回调式 InputStream + float32** 读取；
  `sd.rec` 阻塞读取会拿到全零或垃圾。
* native.dll 的多个副本被 Smart App Control 拦截，必须自动挑能加载的那个。
* CUDA 后端在现有构建里未注册，ASR 必须用 vulkan。
* 链路偶发"显示 link=1 但实际全零"，采集后必须校验。
"""
from __future__ import annotations

import json
import os
import sys
import unicodedata
import wave

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401  先摆正 UTF-8 与子进程环境

import numpy as np


import bootstrap  # noqa: F401  UTF-8 输出与子进程环境

# 控制台默认可能是 cp1252/cp936，中英混排会崩；统一成 UTF-8。
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # _asr_sim
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

DATASET = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5"
RATE = 16000
PLAY_RATE = 48000


# ---------------------------------------------------------------- 设备选择
def pick_device(name: str, need_out: int, need_in: int, rate: int):
    """按名字挑设备，优先选默认采样率匹配的那个。"""
    import sounddevice as sd

    best = None
    for i, d in enumerate(sd.query_devices()):
        if name.lower() not in d["name"].lower():
            continue
        if need_out and d["max_output_channels"] < need_out:
            continue
        if need_in and d["max_input_channels"] < need_in:
            continue
        score = 0 if int(d["default_samplerate"]) == rate else 1
        if best is None or score < best[0]:
            best = (score, i, d)
    return (best[1], best[2]) if best else (None, None)


def list_candidates(name: str):
    import sounddevice as sd

    out = []
    for i, d in enumerate(sd.query_devices()):
        if name.lower() in d["name"].lower():
            out.append((i, d))
    return out


# ---------------------------------------------------------------- 放音 / 录音
def pink_noise(n: int, fs: int, seed: int = 7) -> np.ndarray:
    rng = np.random.default_rng(seed)
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    spec[1:] /= np.sqrt(np.maximum(f[1:], 1.0))
    spec[0] = 0.0
    x = np.fft.irfft(spec, n)
    return (x / (np.sqrt(np.mean(x ** 2)) + 1e-12)).astype(np.float32)


def rec_int16(frames: int, device: int, channels: int = 2) -> np.ndarray:
    """同步录音，返回 float64 ±1。必须 int16 读取。"""
    import sounddevice as sd

    rec = sd.rec(frames, samplerate=RATE, channels=channels, device=device, dtype="int16")
    sd.wait()
    return rec.astype(np.float64) / 32768.0


def play_block(buf: np.ndarray, device: int) -> None:
    import sounddevice as sd

    sd.play(buf, PLAY_RATE, device=device, blocking=True)


def dbfs(x: np.ndarray) -> float:
    return 20 * np.log10(float(np.sqrt(np.mean(np.asarray(x, dtype=np.float64) ** 2))) + 1e-12)


# ---------------------------------------------------------------- WAV
def read_wav_mono(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as w:
        fs, ch, n = w.getframerate(), w.getnchannels(), w.getnframes()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch)[:, 0]
    return x, fs


def check_capture_dir(d: str) -> dict:
    """采集完整性校验：找出全零/静音的 wav。"""
    files = sorted(f for f in os.listdir(d) if f.endswith(".wav")) if os.path.isdir(d) else []
    silent, good = [], []
    for f in files:
        x, _ = read_wav_mono(os.path.join(d, f))
        (silent if (len(x) == 0 or float(np.max(np.abs(x))) == 0.0) else good).append(f)
    return {"total": len(files), "good": len(good), "silent": len(silent), "silent_files": silent[:5]}


# ---------------------------------------------------------------- 文本 / CER
def norm(s: str) -> str:
    s = unicodedata.normalize("NFKC", s or "").lower()
    return "".join(c for c in s if unicodedata.category(c)[0] not in ("P", "S", "Z", "C"))


def edit_distance(a: str, b: str) -> int:
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def load_manifest(n: int = 24, start: int = 0) -> list[dict]:
    mf = json.load(open(os.path.join(DATASET, "heldout-manifest.strict-v5.json"), encoding="utf-8"))
    items = []
    for c in mf["recordings"][start:start + n]:
        p = os.path.join(DATASET, c["audio"].replace("/", os.sep))
        items.append({"id": os.path.splitext(os.path.basename(p))[0],
                      "audio": p, "reference": c["reference"],
                      "speaker": c.get("speaker", "")})
    return items


# ---------------------------------------------------------------- ASR
def open_asr(backend: str = "vulkan"):
    import asr_harness as AH

    return AH.AsrEngine(backend=backend)


def score_capture_dir(d: str, items: list[dict], eng, backend_note: str = "") -> dict:
    """对已采集目录逐条识别并打分。返回聚合结果 + 逐条明细。"""
    ref = {it["id"]: it["reference"] for it in items}
    rows = []
    for f in sorted(os.listdir(d)):
        if not f.endswith("_L.wav"):
            continue
        cid = f[:-6]
        x, fs = read_wav_mono(os.path.join(d, f))
        hyp = eng.transcribe(x.astype(np.float32), fs)
        rr = norm(ref.get(cid, ""))
        hh = norm(hyp)
        dd = edit_distance(rr, hh) if rr else 0
        rows.append({"id": cid, "reference": ref.get(cid, ""), "hyp": hyp,
                     "dist": dd, "ref_len": len(rr), "cer": (dd / len(rr)) if rr else 0.0})
    tot = sum(r["ref_len"] for r in rows)
    dist = sum(r["dist"] for r in rows)
    return {"dir": d, "clips": len(rows),
            "cer": round(100 * dist / tot, 2) if tot else None,
            "perfect": sum(1 for r in rows if r["dist"] == 0),
            "rows": rows, "note": backend_note}


def write_jsonl(path: str, rows: list[dict]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")

# ---------------------------------------------------------------- 录音（回调式）
class _Recorder:
    """回调式录音缓冲。这块 UAC 用 sd.rec 阻塞读取会拿到全零，
    必须用 InputStream + callback + float32（与 box_capture.py 一致）。"""

    def __init__(self, frames: int, channels: int = 2):
        self.frames = frames
        self.channels = channels
        self.buf = np.zeros((frames, channels), dtype=np.float32)
        self.done = 0

    def callback(self, indata, frames, time_info, status):  # noqa: ARG002
        n = min(frames, self.frames - self.done)
        if n > 0:
            self.buf[self.done:self.done + n] = indata[:n]
            self.done += n


def capture_while_playing(block: np.ndarray, in_dev: int, out_dev: int,
                          seconds: float) -> tuple[np.ndarray, np.ndarray]:
    """边播放边录音，返回 (L, R) float32。block 必须是 (N, 4) 的 PLAY_RATE 数据。"""
    import sounddevice as sd

    frames = int((seconds + 0.5) * RATE)
    rec = _Recorder(frames, 2)
    stream = sd.InputStream(device=in_dev, samplerate=RATE, channels=2,
                            dtype="float32", callback=rec.callback)
    stream.start()
    try:
        sd.play(block, PLAY_RATE, device=out_dev, blocking=True)
        import time as _t
        _t.sleep(0.4)
    finally:
        stream.stop()
        stream.close()
    n = max(rec.done, 1)
    return rec.buf[:n, 0].copy(), rec.buf[:n, 1].copy()


def is_silent(stream_data: np.ndarray) -> bool:
    return stream_data.size == 0 or float(np.max(np.abs(stream_data))) < 1e-7