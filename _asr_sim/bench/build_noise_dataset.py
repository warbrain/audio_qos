"""噪声场景库 + 离线混合：把"干净采集"变成多场景、多信噪比的训练数据。

为什么用离线混合而不是每次录一遍
--------------------------------
* 采集一次干净数据（196 条，含真实箱体混响与麦克风响应）就够；
* 噪声类型、信噪比、方位可以任意组合，成本是 CPU 而不是硬件时间；
* 可复现：同一 seed 得到同一份数据。

噪声从哪来
----------
1. **合成场景**（默认）：babble 用真实语料多说话人叠加；另有键盘、风扇/空调、
   街道、咖啡厅、音乐。它们覆盖了 ASR 最常见的干扰类型。
2. **真实录音**：`--noise-dir DIR` 指定一个装噪声 wav 的目录（MUSAN/DEMAND/自录都行），
   有真实录音时优先用它。

空间真实性
----------
噪声按 (延迟, 左右电平差) 施加到两路，模拟来自某个方向：
  --azimuth 0（正前方）→ 时延 0；90（正侧方）→ 时延 = 间距/c
再叠加随机抖动，避免模型只记住一个方向。

用法
----
  # 生成噪声库（每类一段 30 s）
  python build_noise_dataset.py --make-library --out _asr_sim/noise_lib

  # 用干净采集 + 噪声库生成训练集
  python build_noise_dataset.py --clean _asr_sim/dataset_full/quiet \
      --out _asr_sim/train_v1 --snr -5,0,5,10 --scenarios all
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import wave

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

import bootstrap  # noqa: F401
import simcore as S

FS = 16000
CORPUS = r"D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5"
SCENARIOS = ("babble", "keyboard", "fan", "street", "cafe", "music")


# ---------------------------------------------------------------- 基础工具
def bandlimit(x: np.ndarray, lo: float = 80.0, hi: float = 7000.0) -> np.ndarray:
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1.0 / FS)
    spec[(f < lo) | (f > hi)] = 0.0
    return np.fft.irfft(spec, len(x))


def unit_rms(x: np.ndarray) -> np.ndarray:
    return x / (np.sqrt(np.mean(x.astype(np.float64) ** 2)) + 1e-12)


def read_mono(path: str) -> np.ndarray:
    with wave.open(path, "rb") as w:
        fs, ch, n = w.getframerate(), w.getnchannels(), w.getnframes()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    if fs != FS:
        x = S.resample_linear(x, fs, FS)
    return x


# ---------------------------------------------------------------- 合成场景
def _corpus_files(limit: int = 2000) -> list[str]:
    out = []
    for root, _dirs, files in os.walk(CORPUS):
        for f in files:
            if f.endswith(".wav"):
                out.append(os.path.join(root, f))
                if len(out) >= limit:
                    return out
    return out


def gen_babble(dur: float, rng: np.random.Generator, files: list[str],
               speakers: int = 8, density: float = 1.0) -> np.ndarray:
    """多说话人叠加：每层随机取一条语料、随机增益与偏移。"""
    n = int(dur * FS)
    acc = np.zeros(n)
    step = max(1, int(dur * FS / max(1, int(speakers * density))))
    for k in range(speakers):
        p = files[int(rng.integers(0, len(files)))]
        try:
            v = read_mono(p)
        except Exception:
            continue
        if len(v) < int(0.5 * FS):
            continue
        seg = v[: int(dur * FS)]
        off = (k * step + int(rng.integers(0, max(1, step)))) % n
        gain = 10 ** (rng.uniform(-6, 0) / 20)
        end = min(n, off + len(seg))
        acc[off:end] += seg[: end - off] * gain
    return bandlimit(acc, 150, 6000)


def gen_keyboard(dur: float, rng: np.random.Generator) -> np.ndarray:
    n = int(dur * FS)
    x = np.zeros(n)
    t = 0
    while t < n - 2000:
        t += int(rng.exponential(0.18) * FS)
        if t >= n - 2000:
            break
        L = int(rng.integers(400, 1600))
        click = rng.standard_normal(L) * np.exp(-np.arange(L) / (L / 5))
        x[t:t + L] += click * 10 ** (rng.uniform(-6, 0) / 20)
    return bandlimit(x, 200, 7000)


def gen_fan(dur: float, rng: np.random.Generator) -> np.ndarray:
    n = int(dur * FS)
    t = np.arange(n) / FS
    base = 60 + rng.uniform(0, 30)
    hum = sum(np.sin(2 * np.pi * base * k * t) / k for k in range(1, 7))
    hiss = bandlimit(rng.standard_normal(n), 200, 7000) * 0.6
    return hum * 0.4 + hiss


def gen_street(dur: float, rng: np.random.Generator) -> np.ndarray:
    n = int(dur * FS)
    rumble = bandlimit(rng.standard_normal(n), 30, 300) * 1.2
    air = bandlimit(rng.standard_normal(n), 300, 5000) * 0.35
    x = rumble + air
    for _ in range(int(dur * rng.uniform(0.5, 2.0))):
        t = int(rng.integers(0, max(1, n - 8000)))
        L = int(rng.integers(2000, 7000))
        burst = bandlimit(rng.standard_normal(L), 200, 4000) * np.exp(-np.arange(L) / (L / 4))
        x[t:t + L] += burst * 10 ** (rng.uniform(-8, -2) / 20)
    return x


def gen_cafe(dur: float, rng: np.random.Generator, files: list[str]) -> np.ndarray:
    x = gen_babble(dur, rng, files, speakers=14, density=1.6)
    # 杯盘碰撞：稀疏高频瞬态
    n = int(dur * FS)
    for _ in range(int(dur * rng.uniform(0.4, 1.2))):
        t = int(rng.integers(0, max(1, n - 4000)))
        L = int(rng.integers(600, 2500))
        clink = bandlimit(rng.standard_normal(L), 1500, 7000) * np.exp(-np.arange(L) / (L / 3))
        x[t:t + L] += clink * 10 ** (rng.uniform(-10, -3) / 20)
    return x


def gen_music(dur: float, rng: np.random.Generator) -> np.ndarray:
    n = int(dur * FS)
    t = np.arange(n) / FS
    root = 220 * 2 ** (rng.integers(-5, 5) / 12)
    x = np.zeros(n)
    for k, ratio in enumerate((1.0, 1.26, 1.5)):
        x += np.sin(2 * np.pi * root * ratio * t) / (k + 2)
    beat = (np.sin(2 * np.pi * rng.uniform(1.0, 2.5) * t) > 0.6).astype(float)
    x = x * (0.6 + 0.4 * beat)
    return bandlimit(x, 100, 5000)


def make_library(out_dir: str, seconds: float = 30.0, seed: int = 7) -> dict:
    os.makedirs(out_dir, exist_ok=True)
    rng = np.random.default_rng(seed)
    files = _corpus_files()
    print(f"  语料可用 {len(files)} 条，合成噪声库 -> {out_dir}")
    meta = {}
    for name in SCENARIOS:
        if name == "babble":
            x = gen_babble(seconds, rng, files)
        elif name == "keyboard":
            x = gen_keyboard(seconds, rng)
        elif name == "fan":
            x = gen_fan(seconds, rng)
        elif name == "street":
            x = gen_street(seconds, rng)
        elif name == "cafe":
            x = gen_cafe(seconds, rng, files)
        else:
            x = gen_music(seconds, rng)
        x = unit_rms(x) * 0.1          # 归一化到 -20 dBFS RMS 存储
        path = os.path.join(out_dir, f"{name}.wav")
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(FS)
            w.writeframes(np.clip(x * 32767, -32768, 32767).astype("<i2").tobytes())
        meta[name] = path
        spec = np.abs(np.fft.rfft(x))
        f = np.fft.rfftfreq(len(x), 1.0 / FS)
        centroid = float(np.sum(f * spec) / (np.sum(spec) + 1e-12))
        print(f"    {name:9s} {len(x)/FS:5.1f}s  频谱重心 {centroid:6.0f} Hz")
    json.dump(meta, open(os.path.join(out_dir, "library.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
    return meta


# ---------------------------------------------------------------- 空间化与混合
def frac_delay(x: np.ndarray, delay_samples: float) -> np.ndarray:
    """分数样本延迟（频域相位）。"""
    n = len(x)
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1.0 / FS)
    spec *= np.exp(-2j * np.pi * f * delay_samples / FS)
    return np.fft.irfft(spec, n)


def spatialize(noise: np.ndarray, azimuth_deg: float, spacing: float,
               rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """按方位给两路不同的延迟与电平，模拟噪声来自某个方向。"""
    max_delay = spacing / 343.0 * FS                  # 样本
    d = max_delay * math.sin(math.radians(azimuth_deg))
    d += rng.uniform(-0.2, 0.2)                       # 轻微抖动
    ild_db = float(np.clip(6.0 * math.sin(math.radians(azimuth_deg)), -6, 6))
    ild_db += rng.uniform(-1, 1)
    left = frac_delay(noise, 0.0)
    right = frac_delay(noise, d) * (10 ** (ild_db / 20.0))
    return left, right


def mix_pair(clean_l: np.ndarray, clean_r: np.ndarray,
             noise: np.ndarray, snr_db: float, azimuth: float,
             spacing: float, rng) -> tuple[np.ndarray, np.ndarray, dict]:
    nl, nr = spatialize(noise, azimuth, spacing, rng)
    n = min(len(clean_l), len(nl))
    cl, cr, nl, nr = clean_l[:n], clean_r[:n], nl[:n], nr[:n]

    a, b = int(0.15 * FS), int(min(len(cl), 0.6 * FS))
    if b <= a:
        a, b = 0, len(cl)
    sp_p = float(np.mean(cl[a:b] ** 2)) + 1e-20
    nz_p = float(np.mean(nl[a:b] ** 2)) + 1e-20
    scale = math.sqrt((sp_p / (10 ** (snr_db / 10.0))) / nz_p)

    l = cl + nl * scale
    r = cr + nr * scale
    peak = max(float(np.max(np.abs(l))), float(np.max(np.abs(r))), 1e-9)
    if peak > 0.99:
        l *= 0.99 / peak
        r *= 0.99 / peak
    actual = 10 * math.log10(sp_p / (float(np.mean((nl * scale) ** 2)) + 1e-20))
    return l, r, {"target_snr": snr_db, "actual_snr": round(actual, 1)}


# ---------------------------------------------------------------- 主流程
def build(clean_dir: str, out_dir: str, scenarios: list[str], snrs: list[float],
          azimuths: list[float], spacing: float, seed: int, noise_dir: str | None) -> int:
    lib_dir = noise_dir or os.path.join(os.path.dirname(out_dir), "noise_lib")
    if noise_dir is None and not os.path.exists(os.path.join(lib_dir, "library.json")):
        make_library(lib_dir, seed=seed)
    if noise_dir:
        lib = {os.path.splitext(f)[0]: os.path.join(noise_dir, f)
               for f in os.listdir(noise_dir) if f.endswith(".wav")}
        print(f"  使用真实噪声目录 {noise_dir}（{len(lib)} 个文件）")
    else:
        lib = json.load(open(os.path.join(lib_dir, "library.json"), encoding="utf-8"))

    chosen = list(lib) if "all" in scenarios else [s for s in scenarios if s in lib]
    if not chosen:
        print(f"  没有可用噪声；库里有 {list(lib)}", file=sys.stderr)
        return 2

    rng = np.random.default_rng(seed)
    clips = sorted(f for f in os.listdir(clean_dir) if f.endswith("_L.wav"))
    if not clips:
        print(f"  干净目录没有 *_L.wav: {clean_dir}", file=sys.stderr)
        return 2

    refs = {}
    cap = os.path.join(clean_dir, "capture.json")
    if os.path.exists(cap):
        for line in open(cap, encoding="utf-8"):
            if line.strip():
                r = json.loads(line)
                refs[r["id"]] = r.get("reference", "")

    manifest = []
    noisy_cache = {name: read_mono(path) for name, path in lib.items() if name in chosen}
    for name in chosen:
        noise = noisy_cache[name]
        for snr in snrs:
            for az in azimuths:
                tag = f"{name}_snr{snr:+.0f}_az{az:.0f}".replace("+", "p").replace("-", "m")
                d = os.path.join(out_dir, tag)
                os.makedirs(d, exist_ok=True)
                for f in clips:
                    cid = f[:-6]
                    cl, _ = S.load_wav_mono(os.path.join(clean_dir, f))
                    cr, _ = S.load_wav_mono(os.path.join(clean_dir, f"{cid}_R.wav"))
                    if len(noise) < len(cl):
                        noise_ext = np.tile(noise, int(np.ceil(len(cl) / len(noise))))
                    else:
                        noise_ext = noise
                    start = int(rng.integers(0, max(1, len(noise_ext) - len(cl))))
                    seg = noise_ext[start:start + len(cl)]
                    l, r, info = mix_pair(cl, cr, seg, snr, az, spacing, rng)
                    for suffix, ch in (("L", l), ("R", r)):
                        with wave.open(os.path.join(d, f"{cid}_{suffix}.wav"), "wb") as w:
                            w.setnchannels(1)
                            w.setsampwidth(2)
                            w.setframerate(FS)
                            w.writeframes(np.clip(ch * 32767, -32768, 32767).astype("<i2").tobytes())
                    manifest.append({"id": cid, "reference": refs.get(cid, ""),
                                     "clean_dir": clean_dir, "noisy_dir": d,
                                     "scenario": name, **info})
                print(f"    {tag:26s} {len(clips)} 条  SNR {info['actual_snr']:+.1f} dB", flush=True)

    with open(os.path.join(out_dir, "pairs.jsonl"), "w", encoding="utf-8") as f:
        for m in manifest:
            f.write(json.dumps(m, ensure_ascii=False) + "\n")
    print(f"\n  共 {len(manifest)} 对 -> {os.path.join(out_dir, 'pairs.jsonl')}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--make-library", action="store_true")
    ap.add_argument("--out", required=True)
    ap.add_argument("--clean", default=None, help="干净采集目录（含 *_L.wav/_R.wav）")
    ap.add_argument("--scenarios", nargs="+", default=["all"])
    ap.add_argument("--snr", nargs="+", type=float, default=[0.0])
    ap.add_argument("--azimuth", nargs="+", type=float, default=[90.0])
    ap.add_argument("--spacing", type=float, default=0.020)
    ap.add_argument("--noise-dir", default=None, help="真实噪声 wav 目录（优先于合成库）")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    if args.make_library:
        make_library(args.out, seed=args.seed)
        return 0
    if not args.clean:
        print("需要 --clean 或 --make-library", file=sys.stderr)
        return 2
    return build(args.clean, args.out, args.scenarios, args.snr, args.azimuth,
                 args.spacing, args.seed, args.noise_dir)


if __name__ == "__main__":
    raise SystemExit(main())
