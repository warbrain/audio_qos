"""渲染一个仿真条件（pyroomacoustics）。

两种噪声控制方式，二选一：
  --snr N          把噪声归一化到目标信噪比（固定 SNR 的对比实验）
  --noise-level L  固定噪声源电平，SNR 由声源到麦克风的距离决定
                   （嘴距实验必须用这个，否则归一化会把 1/r 效应抹掉）

用法示例
--------
python sim_room.py --clips 24 --out _asr_sim/sim_quiet
python sim_room.py --clips 24 --snr 0 --noise-azimuth 90 --out _asr_sim/sim_snr0
python sim_room.py --clips 24 --distance 0.05 --noise-level -30 --out _asr_sim/sim_d5
python sim_room.py --clips 24 --layout endfire --out _asr_sim/sim_endfire
"""
from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import bootstrap  # noqa: F401
import simcore as S


def main() -> int:
    ap = argparse.ArgumentParser(description="pyroomacoustics 双麦仿真渲染")
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--layout", choices=list(S.LAYOUTS), default="broadside",
                    help="broadside=实测几何(时延约0)，endfire=旧假设(时延0.93样本)")
    ap.add_argument("--spacing", type=float, default=0.020)
    ap.add_argument("--distance", type=float, default=0.10,
                    help="人工嘴到阵列中心距离(m)")
    ap.add_argument("--t60", type=float, default=0.30)
    ap.add_argument("--snr", type=float, default=None,
                    help="目标信噪比(dB)；与 --noise-level 二选一，省略=安静")
    ap.add_argument("--noise-level", type=float, default=None,
                    help="固定噪声源电平(dBFS)；SNR 由距离决定，用于嘴距实验")
    ap.add_argument("--noise-azimuth", type=float, default=90.0,
                    help="噪声源方位角(度)，0=正前方，90=正侧方")
    ap.add_argument("--noise-distance", type=float, default=0.30)
    ap.add_argument("--mic-mismatch-db", type=float, default=0.0,
                    help="两麦灵敏度失配，正数表示 L 比 R 高(实测约 -1.54)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--quiet", action="store_true", help="不逐条打印")
    args = ap.parse_args()

    if args.snr is not None and args.noise_level is not None:
        print("--snr 与 --noise-level 只能选一个", file=sys.stderr)
        return 2

    cfg = S.SimConfig(
        layout=args.layout, spacing=args.spacing, distance=args.distance,
        t60=args.t60, snr_db=args.snr, noise_level_dbfs=args.noise_level,
        noise_azimuth_deg=args.noise_azimuth,
        noise_distance=args.noise_distance, seed=args.seed,
        mic_gain_db=(args.mic_mismatch_db, 0.0))
    cfg.validate()

    items = S.load_manifest(args.clips, args.start)
    mode = "quiet"
    if cfg.snr_db is not None:
        mode = f"snr={cfg.snr_db:+.0f}dB"
    elif cfg.noise_level_dbfs is not None:
        mode = f"noise-level={cfg.noise_level_dbfs:+.0f}dBFS"
    print(f"  渲染 {len(items)} 条 -> {args.out}")
    print(f"  几何={cfg.layout} 间距={cfg.spacing*1000:.0f}mm 嘴距={cfg.distance*100:.0f}cm "
          f"T60={cfg.t60}s {mode} 噪声方位={cfg.noise_azimuth_deg}")

    S.render_dataset(items, args.out, cfg, verbose=not args.quiet)

    theo = S.theoretical_delay_samples(cfg)
    print(f"\n=== 几何自检（理论时延 {theo:.3f} 样本）===")
    for r in items[:3]:
        l, _ = S.load_wav_mono(os.path.join(args.out, f"{r['id']}_L.wav"))
        rr, _ = S.load_wav_mono(os.path.join(args.out, f"{r['id']}_R.wav"))
        print(f"  {r['id'][:26]} 实测 lag {S.measure_delay(l, rr):+d} 样本")

    # 回报实测 SNR（若加了噪声）
    import json
    cap = os.path.join(args.out, "capture.json")
    if os.path.exists(cap):
        rows = [json.loads(l) for l in open(cap, encoding="utf-8") if l.strip()]
        snrs = [r["snr_db"] for r in rows if r.get("snr_db") is not None]
        if snrs:
            import statistics
            print(f"\n=== 实测 SNR（{len(snrs)} 条）中位 {statistics.median(snrs):+.1f} dB ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
