"""空间算法对比：方向性噪声（单侧端射）+ 固定 SNR。

与之前测试的区别：
  * 噪声只从左侧音箱播放（Out3），且位置已使 TDOA ≈ +1 样本（端射）→ 阵列有方向信息；
  * 噪声按峰值归一化，不削顶；
  * 语音电平按实测耦合设定，使 SNR ≈ 0 dB。

用法：
  python run_spatial_directional.py --modes 0,1,4,6 --clips 24
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

import bootstrap  # noqa: F401
import numpy as np

import simcore as S
import score as SC

BOX = os.path.join(os.path.dirname(_HERE), "box_capture.py")
OUT_BASE = os.path.join(os.path.dirname(_HERE), "spatial_dir")
NAMES = {0: "直通", 5: "双路Wiener",
         1: "DAS(已下线)", 2: "NLMS(已下线)", 3: "NLMS立体声(已下线)",
         4: "DAS+Wiener(已下线)", 6: "MVDR(已下线)"}


def set_mode(mode: int) -> str:
    from set_spatial import DIR_WRITE, open_config, request

    h = open_config()
    try:
        got, err = request(h, 3, 18, mode, DIR_WRITE)
        return str(got if got is not None else err)
    finally:
        h.close()


def capture(tag: str, items: list[dict], args, *, spatial_mode: int | None = None) -> str:
    d = os.path.join(OUT_BASE, tag)
    os.makedirs(d, exist_ok=True)
    manifest = os.path.join(d, "_manifest.json")
    json.dump(items, open(manifest, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    cmd = [sys.executable, "-X", "utf8", BOX, "--manifest", manifest,
           "--play-dbfs", str(args.play_dbfs),
           "--noise-dbfs", str(args.noise_dbfs),
           "--noise-channel", "left",
           "--out", d]
    if spatial_mode is not None:
        cmd.extend(["--spatial-mode", str(spatial_mode)])
    subprocess.run(cmd, check=False, env=bootstrap.child_env(),
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--modes", default="0,5",
                    help="firmware 0.5.27 exposes only 0 (bypass) and 5 (stereo Wiener)")
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--play-dbfs", type=float, default=-35.0)
    ap.add_argument("--noise-dbfs", type=float, default=-12.0)
    ap.add_argument("--backend", default="vulkan")
    ap.add_argument("--tag", default="")
    ap.add_argument("--manifest", default=None,
                    help="自定义清单 json（列表或含 recordings 的对象）；省略则用留出集")
    args = ap.parse_args()

    modes = [int(m) for m in args.modes.split(",") if m.strip()]
    removed = [m for m in modes if m not in (0, 5)]
    if removed:
        raise SystemExit(
            f"spatial mode(s) {removed} were removed in firmware 0.5.27; "
            f"supported modes are 0 (bypass) and 5 (stereo Wiener)"
        )
    if args.manifest:
        raw = json.load(open(args.manifest, encoding="utf-8"))
        items = raw if isinstance(raw, list) else raw.get("recordings", [])
        items = items[: args.clips]
        print(f"  清单 {os.path.basename(args.manifest)}: {len(items)} 条", flush=True)
    else:
        items = S.load_manifest(args.clips, args.start)
    refs = {it["id"]: it["reference"] for it in items}
    os.makedirs(OUT_BASE, exist_ok=True)
    eng = SC.open_asr(args.backend)
    summary = []
    try:
        for m in modes:
            tag = f"m{m}" + (f"_{args.tag}" if args.tag else "")
            print(f"\n--- 模式 {m} ({NAMES.get(m,'?')}) ---", flush=True)
            print("  HID:", set_mode(m), flush=True)
            d = capture(tag, items, args, spatial_mode=m)
            files = [f for f in os.listdir(d) if f.endswith("_L.wav")]
            silent = 0
            for f in files:
                x, _ = S.load_wav_mono(os.path.join(d, f))
                if len(x) == 0 or float(np.max(np.abs(x))) == 0:
                    silent += 1
            print(f"  采集 {len(files)} 条，静音 {silent}", flush=True)
            if silent:
                summary.append({"mode": m, "cer": None, "note": f"silent {silent}"})
                continue
            res = SC.score_dir(d, refs, eng)
            SC.write_report(d, res)
            summary.append({"mode": m, "cer": res["cer"], "cer_robust": res["cer_robust"],
                            "perfect": res["perfect"], "clips": res["clips"],
                            "degenerate": res["degenerate"]})
            print(f"  模式 {m}: CER {res['cer']:6.2f}%  全对 {res['perfect']}/{res['clips']}", flush=True)
    finally:
        eng.close()

    with open(os.path.join(OUT_BASE, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": summary}, f, ensure_ascii=False, indent=2)

    print("\n=== 汇总（方向性噪声，端射）===")
    print(f"{'模式':<12s} {'CER':>8s} {'稳健':>8s} {'全对':>9s}")
    for e in summary:
        if e.get("cer") is None:
            print(f"{NAMES.get(e['mode'],'?'):<12s} {'—':>8s} {'—':>8s} {e.get('note',''):>9s}")
        else:
            print(f"{NAMES.get(e['mode'],'?'):<12s} {e['cer']:7.2f}% {e['cer_robust']:7.2f}% "
                  f"{e['perfect']:4d}/{e['clips']:<4d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
