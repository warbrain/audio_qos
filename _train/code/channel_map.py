"""逐通道送 1 kHz 音，用 WhisPrompt 麦克风录音，确认哪个 index 对应人工嘴。

人工嘴距麦克风仅 2 cm，响度会显著高于其他音箱，因此"最响的那一路"就是嘴。
"""
import time

import numpy as np
import sounddevice as sd

FS_PLAY = 48000
FS_CAP = 16000
TONE_DBFS = -20.0
DUR = 1.0


def main() -> int:
    dev_out = dev_in = None
    for d in sd.query_devices():
        n = "".join(c for c in d["name"].lower() if c.isalnum())
        if dev_in is None and "whisprompt" in n and d["max_input_channels"] >= 2:
            dev_in = d["index"]
        if dev_out is None and d["max_output_channels"] == 4 and "us4x4" in n:
            dev_out = d["index"]
    print(f"  in={dev_in} out={dev_out}")

    t = np.arange(int(DUR * FS_PLAY)) / FS_PLAY
    tone = (10 ** (TONE_DBFS / 20)) * np.sin(2 * np.pi * 1000 * t).astype(np.float32)

    for idx in range(4):
        audio = np.zeros((len(tone), 4), dtype=np.float32)
        audio[:, idx] = tone
        rec = []
        def cb(indata, frames, tt, status):
            rec.append(indata[:, :2].copy())
        stream = sd.InputStream(device=dev_in, channels=2, samplerate=FS_CAP,
                                blocksize=1024, callback=cb)
        stream.start()
        time.sleep(0.1)
        sd.play(audio, FS_PLAY, device=dev_out, blocking=True)
        time.sleep(0.3)
        stream.stop()
        stream.close()
        cap = np.concatenate(rec) if rec else np.zeros((0, 2))
        if len(cap) < 1600:
            print(f"  index {idx}: 采集不足")
            continue
        # 只取播放期间的中段
        a = int(0.3 * FS_CAP)
        b = min(int(1.0 * FS_CAP), len(cap))
        seg = cap[a:b]
        rl = 20 * np.log10(np.sqrt(np.mean(seg[:, 0] ** 2)) + 1e-12)
        rr = 20 * np.log10(np.sqrt(np.mean(seg[:, 1] ** 2)) + 1e-12)
        print(f"  index {idx}: L {rl:6.1f} dBFS   R {rr:6.1f} dBFS   峰值 {np.max(np.abs(seg)):.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
