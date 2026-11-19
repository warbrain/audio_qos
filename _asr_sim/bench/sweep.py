"""仿真参数扫描：一次跑一组条件并给出 CER 对比表。

每个条件 = 一个 (layout, t60, snr, noise_azimuth) 组合，渲染到独立目录后评分。
这是纯 pyroomacoustics 的路径，不需要音箱/麦克风/链路。

用法：
  python sweep.py --clips 24 --matrix quick
  python sweep.py --clips 24 --layout broadside endfire --snr -6 0 6 --t60 0.2 0.5
"""
from __future__ import annotations

import argparse
import itertools
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
import score as SC
import simcore as S

OUT_BASE = os.path.join(S.__dict__.get("ROOT", os.path.dirname(_HERE)), "sim_sweep")

PRESETS = {
    # 快速验证：几何对比 + 噪声有无
    "quick": {
        "layout": ["broadside"],
        "t60": [0.30],
        "snr": [None, 0.0],
        "azimuth": [90.0],
    },
    # 几何对比：同一 SNR 下端射 vs 侧向
    "geometry": {
        "layout": ["broadside", "endfire"],
        "t60": [0.30],
        "snr": [None, 0.0, -6.0],
        "azimuth": [90.0],
    },
    # 噪声方位：正前方(0) / 侧前方(45) / 正侧方(90) / 后方(180)
    "azimuth": {
        "layout": ["broadside"],
        "t60": [0.30],
        "snr": [0.0],
        "azimuth": [0.0, 45.0, 90.0, 180.0],
    },
    # 嘴距：固定噪声源电平，SNR 由距离决定（10->5 cm 理论 +6 dB）
    "distance": {
        "layout": ["broadside"],
        "t60": [0.30],
        "distance": [0.05, 0.075, 0.10, 0.15, 0.20],
        "noise_level": [-30.0],
        "azimuth": [90.0],
    },
    # 嘴距 × 混响
    "distance_reverb": {
        "layout": ["broadside"],
        "t60": [0.2, 0.5],
        "distance": [0.05, 0.10],
        "noise_level": [-30.0],
        "azimuth": [90.0],
    },
    # 混响
    "reverb": {
        "layout": ["broadside"],
        "t60": [0.2, 0.5, 0.8],
        "snr": [0.0],
        "azimuth": [90.0],
    },
}


def condition_tag(cfg: S.SimConfig) -> str:
    if cfg.snr_db is not None:
        snr = f"snr{cfg.snr_db:+.0f}".replace("+", "p").replace("-", "m")
    elif cfg.noise_level_dbfs is not None:
        snr = f"nl{cfg.noise_level_dbfs:+.0f}".replace("+", "p").replace("-", "m")
    else:
        snr = "quiet"
    az = "noaz" if cfg.noise_azimuth_deg is None else f"az{cfg.noise_azimuth_deg:.0f}"
    return f"{cfg.layout}_d{cfg.distance*100:.0f}cm_t60{cfg.t60:.1f}_{snr}_{az}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--matrix", choices=list(PRESETS), default=None,
                    help="使用预设矩阵；省略时用下面的显式参数")
    ap.add_argument("--layout", nargs="+", default=["broadside"])
    ap.add_argument("--t60", nargs="+", type=float, default=[0.30])
    ap.add_argument("--snr", nargs="+", type=float, default=[0.0],
                    help="用 'none' 表示安静条件")
    ap.add_argument("--azimuth", nargs="+", type=float, default=[90.0])
    ap.add_argument("--spacing", type=float, default=0.020)
    ap.add_argument("--distance-default", type=float, default=0.10)
    ap.add_argument("--mic-mismatch-db", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--backend", default="vulkan")
    ap.add_argument("--no-score", action="store_true", help="只渲染不评分")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    distances = [args.distance_default]
    noise_levels = [None]
    if args.matrix:
        m = PRESETS[args.matrix]
        layouts, t60s, azs = m["layout"], m["t60"], m["azimuth"]
        snrs = m.get("snr", [None])
        distances = m.get("distance", [args.distance_default])
        noise_levels = m.get("noise_level", [None])
    else:
        layouts, t60s, snrs, azs = args.layout, args.t60, args.snr, args.azimuth

    out_base = args.out or os.path.join(_HERE, "..", "sim_sweep")
    out_base = os.path.abspath(out_base)
    os.makedirs(out_base, exist_ok=True)

    items = S.load_manifest(args.clips, args.start)
    refs = {it["id"]: it["reference"] for it in items}
    eng = None if args.no_score else SC.open_asr(args.backend)

    results = []
    try:
        for layout, t60, snr, az, dist, nlev in itertools.product(
                layouts, t60s, snrs, azs, distances, noise_levels):
            cfg = S.SimConfig(layout=layout, spacing=args.spacing, distance=dist,
                              t60=t60, snr_db=snr, noise_level_dbfs=nlev,
                              noise_azimuth_deg=az, seed=args.seed,
                              mic_gain_db=(args.mic_mismatch_db, 0.0))
            cfg.validate()
            tag = condition_tag(cfg)
            d = os.path.join(out_base, tag)
            print(f"\n=== {tag} ===", flush=True)
            S.render_dataset(items, d, cfg, verbose=False)

            entry = {"tag": tag, "config": {"layout": layout, "t60": t60,
                                            "snr_db": snr, "noise_level_dbfs": nlev,
                                            "distance_m": dist, "noise_azimuth_deg": az},
                     "dir": d}
            if eng is not None:
                res = SC.score_dir(d, refs, eng)
                SC.write_report(d, res)
                entry.update({"cer": res["cer"], "cer_robust": res["cer_robust"],
                              "perfect": res["perfect"],
                              "degenerate": res["degenerate"]})
                print(f"    CER {res['cer']:6.2f}%  稳健 {res['cer_robust']:6.2f}%  "
                      f"全对 {res['perfect']}/{res['clips']}", flush=True)
            results.append(entry)
    finally:
        if eng is not None:
            eng.close()

    with open(os.path.join(out_base, "sweep_summary.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)

    if eng is not None:
        print(f"\n{'条件':<34s} {'CER':>8s} {'稳健':>8s} {'全对':>7s}")
        print("-" * 62)
        for e in results:
            print(f"{e['tag']:<34s} {e['cer']:7.2f}% {e['cer_robust']:7.2f}% "
                  f"{e['perfect']:3d}/{args.clips:<3d}")
    print(f"\n输出目录: {out_base}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
