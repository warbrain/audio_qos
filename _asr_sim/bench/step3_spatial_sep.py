"""第 3 步：空间分离噪声下的空间滤波对比（本项目最关键的一次实验）。

噪声从 Out3/Out4（房间里的两只音箱）播放，语音从人工嘴播放 —— 这才是
空间滤波能够发挥作用的条件；此前所有测试都是同点噪声（--noise-mouth），
阵列在物理上没有方向信息，因此 6 种模式全都无增益。

流程：对每个模式 → HID 设置 → 采集 24 条 → 静音校验 → ASR 打分。
任何一轮检出全零音频立即中止，避免把废数据算进结论。

用法：
  python step3_spatial_sep.py --modes 0,1,2,4,6 --noise-dbfs -30
  python step3_spatial_sep.py --modes 0 --noise-dbfs -30 --noise-coherent
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
OUT_BASE = os.path.join(C.ROOT, "spatial_sep")
HID_VERSION_SPATIAL = 3
ITEM_SPATIAL_REQUESTED = 18


def set_mode(mode: int) -> None:
    """通过 HID 设置空间模式（0=直通 1=DAS 2=NLMS 3=NLMS立体声 4=DAS+Wiener 5=双路Wiener 6=MVDR）。"""
    sys.path.insert(0, C.ROOT)
    from set_spatial import DIR_WRITE, open_config, request

    h = open_config()
    try:
        got, err = request(h, HID_VERSION_SPATIAL, ITEM_SPATIAL_REQUESTED, mode, DIR_WRITE)
        print(f"  设置模式 {mode}: {got if got is not None else err}", flush=True)
    finally:
        h.close()


def capture(tag: str, items: list[dict], play: float, noise: float | None,
            coherent: bool, repeat: int) -> str:
    d = os.path.join(OUT_BASE, tag)
    os.makedirs(d, exist_ok=True)
    manifest = os.path.join(d, "_manifest.json")
    json.dump(items, open(manifest, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    cmd = [PY, "-X", "utf8", os.path.join(C.ROOT, "box_capture.py"), "--manifest", manifest,
           "--play-dbfs", str(play), "--out", d]
    if noise is not None:
        cmd += ["--noise-dbfs", str(noise)]      # 不加 --noise-mouth → 走 Out3/Out4
    if coherent:
        cmd += ["--noise-coherent"]
    if repeat > 1:
        cmd += ["--repeat", str(repeat)]
    print("  采集:", " ".join(cmd[-6:]), flush=True)
    subprocess.run(cmd, check=False, env=CHILD_ENV,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--modes", default="0,5",
                    help="firmware 0.5.27 exposes only 0 and 5")
    ap.add_argument("--clips", type=int, default=24)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--play-dbfs", type=float, default=-31.0,
                    help="实测：语音 -31 dBFS + 噪声 0 dBFS 时麦克风端 SNR 约 0 dB")
    ap.add_argument("--noise-dbfs", type=float, default=0.0,
                    help="房间音箱要开到最大才能到达麦克风（耦合约 -26 dB）")
    ap.add_argument("--noise-coherent", action="store_true")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--tag", default="")
    ap.add_argument("--backend", default="vulkan")
    args = ap.parse_args()

    modes = [int(m) for m in args.modes.split(",") if m.strip()]
    items = C.load_manifest(args.clips, args.start)
    eng = C.open_asr(args.backend)
    summary = []

    for mode in modes:
        suffix = f"_m{mode}" + (f"_{args.tag}" if args.tag else "")
        tag = f"snr{int(args.play_dbfs - args.noise_dbfs):+03d}{suffix}".replace("+", "p").replace("-", "m")
        print(f"\n--- 模式 {mode} ({tag}) ---", flush=True)
        set_mode(mode)
        d = capture(tag, items, args.play_dbfs, args.noise_dbfs, args.noise_coherent, args.repeat)

        info = C.check_capture_dir(d)
        print(f"  采集校验: {info['good']}/{info['total']} 有效, 静音 {info['silent']}", flush=True)
        if info["silent"] > 0:
            print("  ❌ 检出全零音频，本轮作废（多为 ESP-NOW 链路断开）。"
                  "请断电重启 Scepter 后重跑，不要使用这批数据。", flush=True)
            summary.append({"mode": mode, "tag": tag, "cer": None, "perfect": None,
                            "note": f"silent {info['silent']}/{info['total']}"})
            continue

        res = C.score_capture_dir(d, items, eng)
        C.write_jsonl(os.path.join(d, "asr.jsonl"), res["rows"])
        entry = {"mode": mode, "tag": tag, "clips": res["clips"],
                 "cer": res["cer"], "perfect": res["perfect"]}
        summary.append(entry)
        print(f"  模式 {mode}: CER {res['cer']:6.2f}%  全对 {res['perfect']}/{res['clips']}", flush=True)

    eng.close()
    os.makedirs(OUT_BASE, exist_ok=True)
    with open(os.path.join(OUT_BASE, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"config": vars(args), "results": summary}, f, ensure_ascii=False, indent=2)

    print("\n=== 汇总（空间分离噪声）===")
    print(f"{'模式':>4s} {'CER':>8s} {'全对':>9s}  备注")
    names = {0: "直通", 5: "双路Wiener",
             1: "DAS(已下线)", 2: "NLMS(已下线)", 3: "NLMS立体声(已下线)",
             4: "DAS+Wiener(已下线)", 6: "MVDR(已下线)"}
    for e in summary:
        cer = f"{e['cer']:7.2f}%" if e["cer"] is not None else "    —  "
        pf = f"{e['perfect']}/{e['clips']}" if e["perfect"] is not None else "—"
        print(f"{e['mode']:4d} {cer} {pf:>9s}  {names.get(e['mode'], ''):<10s} {e.get('note', '')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
