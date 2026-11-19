"""信噪比阈值扫点：直通(0) vs 双路 Wiener(5)。

用 calibrate_snr.py 标定出的耦合，把目标 SNR 换算成噪声播放电平，
逐点采集、评分，输出 CER(SNR) 曲线，用于判定"何时启用 Wiener"。

前提（当前工位）：
  * 隔离箱门缝留开，仅 Out3 播放噪声，Out4 关闭；
  * 人工嘴到 WhisPrompt 麦克风 10 cm；
  * 噪声为单侧方向性，TDOA ≈ +1 样本（端射）。

用法：
  python snr_threshold_sweep.py --clips 24 --snr 10,5,0,-5,-10
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _ROOT)

import bootstrap  # noqa: F401
import numpy as np

import simcore as S
import score as SC

BOX = os.path.join(_ROOT, "box_capture.py")
OUT_BASE = os.path.join(_ROOT, "snr_threshold")
# 实测（当前工位：门缝开、仅 Out3、嘴距 10 cm）：
#   语音耦合 -6.3 dB（播放 -30 -> 麦克风带内 -36.3 dBFS）
#   噪声耦合 -32.3 dB（播放 -14 -> 麦克风带内 -46.3 dBFS）
# 粉噪峰值因子约 12 dB，超过 -14 dBFS RMS 就会被峰值钳位削顶，
# 因此噪声取 -14 dBFS 为"不削顶的最大值"，再通过降低语音来达成低 SNR。
NOISE_PLAY_DBFS = -14.0    # 不削顶的最大噪声（粉噪峰值因子 12 dB）
NOISE_MIC_DBFS = -38.5      # 实测：噪声 -14 dBFS 播放 -> 麦克风带内 -38.5 dBFS
SPEECH_COUPLING_DB = -2.7  # 实测：语音 -40 dBFS 播放 -> 麦克风带内 -42.7 dBFS
NAMES = {0: "bypass", 5: "stereo Wiener"}


def set_mode(mode: int, retries: int = 4) -> str:
    """写入模式并确认设备回读等于目标值；未确认就重试。

    设备静置会进入 light sleep，首次写入可能只拿到 status=0（未确认）。
    若直接继续，会把直通数据错标成模式 5，因此这里必须确认。
    """
    import time

    from set_spatial import DIR_WRITE, open_config, request

    h = open_config()
    try:
        last = None
        for _ in range(retries):
            got, err = request(h, 3, 18, mode, DIR_WRITE)
            last = got if got is not None else err
            if got == mode:
                return f"confirmed {got}"
            time.sleep(0.4)
        raise SystemExit(f"spatial mode {mode} not confirmed after {retries} attempts: {last}")
    finally:
        h.close()


def capture(tag: str, items: list[dict], play_dbfs: float, noise_dbfs: float) -> str:
    d = os.path.join(OUT_BASE, tag)
    if os.path.isdir(d):
        shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d, exist_ok=True)
    manifest = os.path.join(d, "_manifest.json")
    json.dump(items, open(manifest, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    cmd = [sys.executable, "-X", "utf8", BOX, "--manifest", manifest,
           "--play-dbfs", str(play_dbfs),
           "--noise-dbfs", str(noise_dbfs),
           "--noise-channel", "left",
           "--out", d]
    subprocess.run(cmd, check=False, env=bootstrap.child_env(),
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--snr", default="10,5,0,-5,-10", help="comma-separated target in-band SNR (dB)")
    ap.add_argument("--modes", default="0,5")
    ap.add_argument("--backend", default="vulkan")
    ap.add_argument("--out-tag", default="", help="输出目录后缀，避免覆盖既有数据")
    args = ap.parse_args()

    snrs = [float(x) for x in args.snr.split(",") if x.strip()]
    modes = [int(x) for x in args.modes.split(",") if x.strip()]
    bad = [m for m in modes if m not in (0, 5)]
    if bad:
        raise SystemExit(f"spatial mode(s) {bad} were removed in firmware 0.5.27; use 0 and/or 5")

    items = S.load_manifest(args.clips)
    refs = {it["id"]: it["reference"] for it in items}
    os.makedirs(OUT_BASE, exist_ok=True)
    eng = SC.open_asr(args.backend)
    rows = []
    try:
        for snr in snrs:
            # 噪声固定在最大不削顶电平；用语音电平达成目标 SNR
            speech_play = (NOISE_MIC_DBFS + snr) - SPEECH_COUPLING_DB
            for m in modes:
                tag = f"snr{snr:+.0f}_m{m}" + (f"_{args.out_tag}" if args.out_tag else "")
                print(f"\n--- SNR {snr:+.0f} dB (speech {speech_play:.1f} / noise {NOISE_PLAY_DBFS:.1f} dBFS) mode {m} ---", flush=True)
                print("  HID:", set_mode(m), flush=True)
                d = capture(tag, items, speech_play, NOISE_PLAY_DBFS)
                files = [f for f in os.listdir(d) if f.endswith("_L.wav")]
                silent = 0
                for f in files:
                    x, _ = S.load_wav_mono(os.path.join(d, f))
                    if len(x) == 0 or float(np.max(np.abs(x))) == 0:
                        silent += 1
                if silent:
                    print(f"  ⚠ {silent} 条静音，跳过该点", flush=True)
                    rows.append({"snr": snr, "mode": m, "cer": None, "note": f"silent {silent}"})
                    continue
                res = SC.score_dir(d, refs, eng)
                SC.write_report(d, res)
                rows.append({"snr": snr, "mode": m, "cer": res["cer"],
                             "cer_robust": res["cer_robust"],
                             "perfect": res["perfect"], "clips": res["clips"],
                             "speech_play_dbfs": speech_play, "noise_play_dbfs": NOISE_PLAY_DBFS})
                print(f"  mode {m}: CER {res['cer']:6.2f}%  全对 {res['perfect']}/{res['clips']}", flush=True)
    finally:
        eng.close()

    with open(os.path.join(OUT_BASE, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"speech_coupling_db": SPEECH_COUPLING_DB, "noise_mic_dbfs": NOISE_MIC_DBFS,
                   "results": rows}, f, ensure_ascii=False, indent=2)

    print("\n=== CER vs SNR ===")
    print("   SNR   bypass   stereoWiener   delta")
    for snr in snrs:
        a = next((r for r in rows if r["snr"] == snr and r["mode"] == 0), None)
        b = next((r for r in rows if r["snr"] == snr and r["mode"] == 5), None)
        ca = a["cer"] if (a and a.get("cer") is not None) else None
        cb = b["cer"] if (b and b.get("cer") is not None) else None
        sa = f"{ca:8.2f}%" if ca is not None else "       -"
        sb = f"{cb:14.2f}%" if cb is not None else "             -"
        sd = f"{cb - ca:+8.2f}" if (ca is not None and cb is not None) else "       -"
        print(f"  {snr:+4.0f}{sa}{sb}{sd}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
