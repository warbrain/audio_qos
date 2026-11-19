"""第 4 步：小声/耳语 + 环境噪声 的条件矩阵（噪声固定，只降语音电平）。

真实场景里"耳语"意味着语音变小声、噪声不变 → 信噪比下降。
本脚本用 (play_dbfs, noise_dbfs) 组合复现该场景，并给出 CER。

默认条件（噪声都走 Out3/Out4，即空间分离）：
  A  -30 / -30   SNR   0 dB   基准
  B  -36 / -30   SNR  -6 dB   小声说话
  C  -42 / -30   SNR -12 dB   噪声中的耳语
  D  -42 / -42   SNR   0 dB   对照：同样 SNR，只是整体更小

C 与 A 的差 = 信噪比代价；D 与 A 的差 = 纯电平代价。

用法：
  python step4_whisper_noise.py
  python step4_whisper_noise.py --conditions "-30:-30,-42:-30" --clips 24
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C  # noqa: E402

PY = sys.executable
# 子进程必须 UTF-8：box_capture 会打印中文，cp1252 控制台下会崩掉采集循环。
CHILD_ENV = dict(os.environ, PYTHONIOENCODING="utf-8")
OUT_BASE = os.path.join(C.ROOT, "whisper_noise")

DEFAULT_CONDITIONS = ["-30:-30", "-36:-30", "-42:-30", "-42:-42"]


def capture(tag: str, items: list[dict], play: float, noise: float, coherent: bool) -> str:
    d = os.path.join(OUT_BASE, tag)
    os.makedirs(d, exist_ok=True)
    manifest = os.path.join(d, "_manifest.json")
    json.dump(items, open(manifest, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    cmd = [PY, "-X", "utf8", os.path.join(C.ROOT, "box_capture.py"), "--manifest", manifest,
           "--play-dbfs", str(play), "--noise-dbfs", str(noise), "--out", d]
    if coherent:
        cmd.append("--noise-coherent")
    subprocess.run(cmd, check=False, env=CHILD_ENV,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--conditions", default=",".join(DEFAULT_CONDITIONS),
                    help="逗号分隔的 play:noise 电平对，例如 -30:-30,-42:-30")
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--noise-coherent", action="store_true")
    ap.add_argument("--backend", default="vulkan")
    args = ap.parse_args()

    items = C.load_manifest(args.clips, args.start)
    eng = C.open_asr(args.backend)
    summary = []

    for cond in args.conditions.split(","):
        cond = cond.strip()
        if not cond:
            continue
        play_s, noise_s = cond.split(":")
        play, noise = float(play_s), float(noise_s)
        tag = f"p{int(play)}_n{int(noise)}".replace("-", "m")
        print(f"\n--- play {play:+.0f} / noise {noise:+.0f} dBFS  (SNR {play - noise:+.0f} dB) ---", flush=True)
        d = capture(tag, items, play, noise, args.noise_coherent)

        info = C.check_capture_dir(d)
        print(f"  采集校验: {info['good']}/{info['total']} 有效, 静音 {info['silent']}", flush=True)
        if info["silent"] > 0:
            print("  ❌ 检出全零音频，本轮作废（链路断开）。请重启 Scepter 后重跑。", flush=True)
            summary.append({"play": play, "noise": noise, "cer": None,
                            "note": f"silent {info['silent']}"})
            continue

        res = C.score_capture_dir(d, items, eng)
        C.write_jsonl(os.path.join(d, "asr.jsonl"), res["rows"])
        summary.append({"play": play, "noise": noise, "snr": play - noise,
                        "clips": res["clips"], "cer": res["cer"], "perfect": res["perfect"], "dir": d})
        print(f"  CER {res['cer']:6.2f}%  全对 {res['perfect']}/{res['clips']}", flush=True)

    eng.close()
    os.makedirs(OUT_BASE, exist_ok=True)
    with open(os.path.join(OUT_BASE, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print("\n=== 汇总 ===")
    print(f"{'语音':>8s} {'噪声':>8s} {'SNR':>7s} {'CER':>8s} {'全对':>9s}")
    for e in summary:
        cer = f"{e['cer']:7.2f}%" if e.get("cer") is not None else "    —  "
        pf = f"{e['perfect']}/{e['clips']}" if e.get("perfect") is not None else "—"
        print(f"{e['play']:8.0f} {e['noise']:8.0f} {e.get('snr', 0):+7.0f} {cer} {pf:>9s} {e.get('note', '')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
