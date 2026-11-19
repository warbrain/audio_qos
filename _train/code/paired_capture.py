"""同块配对采集：一次录音内先录"干净语音"，再录"含噪语音"。

为什么需要它：此前含噪与安静是两次独立采集，采样时钟不同步（波形相关仅 0.24），
导致 (a) 模型训练塌缩、(b) 残差法测 SNR 失效。同一录音块内两个片段共享同一个
时钟，可做到样本级对齐。

时间线（一个采集块）：
  | 前导 0.4s | 干净语音 N 样本 | 间隔 0.3s | 含噪语音 N 样本 | 尾部 0.4s |
  噪声声道：前 3 段静音，第 4 段起播放粉噪（与语音同起点，样本级对齐）

输出：
  <out>/<id>_clean_L.wav / _clean_R.wav   ← 干净段
  <out>/<id>_L.wav       / _R.wav         ← 含噪段（复用原命名）

用法：
    python paired_capture.py --manifest m.json --out dir [--limit 3]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import wave

import numpy as np
import sounddevice as sd

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

FS_PLAY = 48000
FS_CAP = 16000
PRE = 0.4
GAP = 0.3
POST = 0.4


def read_wav_any(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as w:
        sr, ch = w.getframerate(), w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch)[:, 0]
    return x, sr


def resample(x: np.ndarray, sr_in: int, sr_out: int) -> np.ndarray:
    if sr_in == sr_out:
        return x
    n = int(round(len(x) * sr_out / sr_in))
    t_in = np.arange(len(x)) / sr_in
    t_out = np.arange(n) / sr_out
    return np.interp(t_out, t_in, x).astype(np.float32)


def pink(n: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    w = rng.standard_normal(n)
    spec = np.fft.rfft(w)
    f = np.fft.rfftfreq(n, 1.0 / FS_PLAY)
    spec[1:] /= np.sqrt(np.maximum(f[1:], 1.0))
    spec[0] = 0.0
    x = np.fft.irfft(spec, n).astype(np.float32)
    return x / (np.sqrt(np.mean(x ** 2)) + 1e-12)


def align_pair(clean: np.ndarray, noisy: np.ndarray, max_lag: int = 64) -> tuple:
    """用互相关把 noisy 对齐到 clean（两个 USB 设备启动时序不同，残余偏移 0-3 ms）。"""
    n = min(len(clean), len(noisy))
    c = clean[:n, 0].astype(np.float64)
    c = c - c.mean()
    best = (-2.0, 0)
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            a, b = c[: n - lag], noisy[lag:, 0].astype(np.float64)
        else:
            a, b = c[-lag:], noisy[: n + lag, 0].astype(np.float64)
        b = b - b.mean()
        r = float(np.dot(a, b) / ((np.linalg.norm(a) * np.linalg.norm(b)) + 1e-12))
        if r > best[0]:
            best = (r, lag)
    lag = best[1]
    if lag > 0:
        noisy = noisy[lag:]
    elif lag < 0:
        clean = clean[-lag:]
    n2 = min(len(clean), len(noisy))
    return clean[:n2], noisy[:n2], lag, best[0]

def write_wav(path: str, x: np.ndarray, fs: int) -> None:
    y = np.clip(np.round(x * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(fs)
        w.writeframes(y.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--play-dbfs", type=float, default=-40.0)
    ap.add_argument("--noise-dbfs", type=float, default=-14.0)
    args = ap.parse_args()

    items = json.load(open(args.manifest, encoding="utf-8"))
    if args.limit:
        items = items[: args.limit]
    os.makedirs(args.out, exist_ok=True)

    dev_out = dev_in = None
    for d in sd.query_devices():
        n = "".join(c for c in d["name"].lower() if c.isalnum())
        # 输入：WhisPrompt UAC 双通道麦克风
        if dev_in is None and "whisprompt" in n and d["max_input_channels"] >= 2:
            dev_in = d["index"]
        # 输出：4 通道 USB 声卡（Out1-4 就是人工嘴与噪声音箱）
        if dev_out is None and d["max_output_channels"] == 4 and "us4x4" in n:
            dev_out = d["index"]
    if dev_out is None or dev_in is None:
        print(f"  设备未找到 out={dev_out} in={dev_in}")
        return 1

    rec = []
    def cb(indata, frames, t, status):
        rec.append(indata[:, :2].copy())

    stream = sd.InputStream(device=dev_in, channels=2, samplerate=FS_CAP,
                            blocksize=480, callback=cb)
    stream.start()

    aligned = []
    for k, it in enumerate(items, 1):
        sp, sr = read_wav_any(it["audio"])
        sp = resample(sp, sr, FS_PLAY)
        rms = float(np.sqrt(np.mean(sp.astype(np.float64) ** 2))) + 1e-12
        sp = (sp * (10 ** (args.play_dbfs / 20.0) / rms)).astype(np.float32)
        n_sp = len(sp)

        pre = np.zeros(int(PRE * FS_PLAY), dtype=np.float32)
        gap = np.zeros(int(GAP * FS_PLAY), dtype=np.float32)
        post = np.zeros(int(POST * FS_PLAY), dtype=np.float32)
        mouth = np.concatenate([pre, sp, gap, sp, post])
        noise = np.zeros(len(mouth), dtype=np.float32)
        start2 = len(pre) + n_sp + len(gap)
        block_n = pink(len(mouth) - start2, seed=k)
        noise[start2:] = block_n * (10 ** (args.noise_dbfs / 20.0))

        audio = np.zeros((len(mouth), 4), dtype=np.float32)
        audio[:, 0] = mouth
        audio[:, 1] = mouth
        audio[:, 2] = noise

        rec.clear()
        sd.play(audio, FS_PLAY, device=dev_out, blocking=True)
        time.sleep(0.4)
        sd.stop()
        cap = np.concatenate(rec) if rec else np.zeros((0, 2))
        time.sleep(0.05)

        # 按播放起点切片：从 pre 结束处开始取两段
        # 采集块包含播放前三段静音，用能量找起点更稳妥；这里用固定偏移近似，
        # 并在两端各留 50 ms 余量。
        # 用能量检测播放起点，避免"录音起点 = 播放起点"的错误假设
        seg = int(n_sp * FS_CAP / FS_PLAY) - 40
        mono = cap.mean(axis=1) if len(cap) else np.zeros(0)
        onset = 0
        if len(mono) > 1600:
            fl = int(0.02 * FS_CAP)
            e = np.array([np.mean(mono[i:i+fl] ** 2) for i in range(0, len(mono) - fl, fl)])
            floor = float(np.percentile(e, 10)) + 1e-12
            for i, v in enumerate(e):
                if v > floor * 25:
                    onset = max(0, i * fl - fl)
                    break
        off1 = onset + int(0.03 * FS_CAP)
        clean = cap[off1: off1 + seg]
        noisy_start = off1 + seg + int(GAP * FS_CAP)
        noisy = cap[noisy_start: noisy_start + seg]
        if len(clean) < seg or len(noisy) < seg:
            print(f"  {it['id'][:26]} 采集不足，跳过")
            continue

        cid = it["id"]
        clean, noisy, lag, r = align_pair(clean, noisy)
        aligned.append((lag, r))
        write_wav(os.path.join(args.out, cid + "_clean_L.wav"), clean[:, 0], FS_CAP)
        write_wav(os.path.join(args.out, cid + "_clean_R.wav"), clean[:, 1], FS_CAP)
        write_wav(os.path.join(args.out, cid + "_L.wav"), noisy[:, 0], FS_CAP)
        write_wav(os.path.join(args.out, cid + "_R.wav"), noisy[:, 1], FS_CAP)
        with open(os.path.join(args.out, "capture.json"), "a", encoding="utf-8") as fh:
            fh.write(json.dumps({"id": cid, "reference": it.get("reference", ""),
                                 "play_dbfs": args.play_dbfs,
                                 "noise_dbfs": args.noise_dbfs}, ensure_ascii=False) + "\n")
        if k % 20 == 0:
            print(f"  {k}/{len(items)}", flush=True)

    stream.stop()
    stream.close()
    if aligned:
        rs = [r for _, r in aligned]
        print(f"  对齐相关: 中位 {np.median(rs):.3f}  最低 {min(rs):.3f}  中位偏移 {int(np.median([l for l, _ in aligned]))} 样本")
    print(f"  完成 {len(items)} 条 -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
