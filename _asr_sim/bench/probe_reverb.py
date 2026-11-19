"""检查混响时间如何决定"靠近麦克风"的收益。"""
import sys
import numpy as np

sys.path.insert(0, r"D:\App\cppApp\audio_qos\_asr_sim\bench")
import simcore as S

items = S.load_manifest(1)
x, _ = S.load_wav_mono(items[0]["audio"])
x = S.resample_linear(x, 16000, S.FS)
rms = float(np.sqrt(np.mean(x ** 2))) + 1e-12
speech = np.concatenate([np.zeros(int(0.6 * S.FS)),
                         x * (10 ** (-30 / 20) / rms),
                         np.zeros(int(1.0 * S.FS))])
a, b = int(0.6 * S.FS), int(1.6 * S.FS)

print(f"{'T60':>6s} {'5cm功率':>10s} {'20cm功率':>10s} {'功率提升':>9s} "
      f"{'5cm峰值':>9s} {'20cm峰值':>9s} {'峰值提升':>9s}")
print("-" * 72)
for t60 in (0.05, 0.10, 0.20, 0.30):
    out = {}
    for d in (0.05, 0.20):
        cfg = S.SimConfig(distance=d, t60=t60, snr_db=None, noise_azimuth_deg=None)
        room = S._make_room(cfg)
        w = S._render(room, {0: speech}, len(speech))[0][a:b]
        out[d] = (10 * np.log10(np.mean(w ** 2) + 1e-20), float(np.max(np.abs(w))))
    p5, pk5 = out[0.05]
    p20, pk20 = out[0.20]
    print(f"{t60:6.2f} {p5:9.1f}dB {p20:9.1f}dB {p5-p20:+8.1f}dB "
          f"{pk5:8.3f} {pk20:8.3f} {20*np.log10(pk5/pk20):+8.1f}dB")
print()
print("解读：功率提升 = 总能量（含弥散场）的变化；峰值提升 = 直达声的变化。")
print("      两者差得越大，说明混响场占比越高、靠近平移的收益被稀释。")
