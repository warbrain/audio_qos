"""测量模式 5（双路 Wiener）的实际 DSP 开销与调度健康度。

思路：打开串口会复位设备（ESP32-S3 USB-Serial-JTAG 的 DTR 行为），
因此必须"先开串口 -> 等启动 -> 设模式 -> 同连接读遥测"，
否则设置的模式会被复位清掉。
"""
from __future__ import annotations

import io
import re
import sys
import time

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
sys.path.insert(0, r"D:\App\cppApp\audio_qos\_asr_sim")

import serial  # noqa: E402
from set_spatial import DIR_WRITE, open_config, request  # noqa: E402

MODE = int(sys.argv[1]) if len(sys.argv) > 1 else 5
DURATION = int(sys.argv[2]) if len(sys.argv) > 2 else 25

s = serial.Serial("COM5", 115200, timeout=0.2)
s.setDTR(False)
s.setRTS(False)
time.sleep(3.0)  # 等复位后的固件启动完成
s.reset_input_buffer()

h = open_config()
got = None
for _ in range(3):
    got, err = request(h, 3, 18, MODE, DIR_WRITE)
    if got is not None:
        break
    time.sleep(0.4)
h.close()
print(f"  设置模式 {MODE}: {got if got is not None else err}")

t0 = time.time()
buf = b""
while time.time() - t0 < DURATION:
    try:
        d = s.read(16384)
    except Exception:
        break
    if d:
        buf += d
s.close()
txt = buf.decode("utf-8", "replace")


def last_int(pattern: str):
    m = re.findall(pattern, txt)
    return int(m[-1]) if m else None


gens = sorted({int(x) for x in re.findall(r'"generation":(\d+)', txt)})
cum_us = last_int(r'"processing_us":"?(\d+)"?')
frames = last_int(r'"processing_frames":(\d+)')
max_us = last_int(r'"processing_max_us":(\d+)')
misses = last_int(r"work_over_20ms=(\d+)")
overloads = last_int(r"overload_stops=(\d+)")
dma_ov = last_int(r"dma_overflows=(\d+)")
qhi = last_int(r"queue_highwater=(\d+)")

print(f"  generation 采样: {gens[:5]}")
print(f"  processing_frames 累计: {frames}")
print(f"  processing_us 累计:    {cum_us}")
if cum_us is not None and frames:
    avg = cum_us / frames
    print(f"  -> DSP 平均每帧 {avg:.0f} us = 20 ms 预算的 {100.0 * avg / 20000:.1f}%")
if max_us is not None:
    print(f"  -> DSP 单帧最大  {max_us} us = 20 ms 预算的 {100.0 * max_us / 20000:.1f}%")

enc = last_int(r"encode_max_us=(\d+)")
late = last_int(r"late_events=(\d+)")
qfull = last_int(r'"queue_full":(\d+)')
fails = last_int(r"failures=(\d+)")
print()
print("  编码与调度（末值）：")
print(f"    encode_max_us = {enc} us = 20 ms 预算的 {100.0 * enc / 20000:.1f}%" if enc else "    encode_max_us = 无")
print(f"    late_events   = {late}")
print(f"    queue_full    = {qfull}")
print(f"    failures      = {fails}")
if enc is not None and max_us is not None:
    print(f"    -> 若同核串行，最坏合计 {(enc + max_us) / 1000:.2f} ms；分核后每核最坏 {max(enc, max_us) / 1000:.2f} ms")
print()
print("  调度健康度（本次窗口末值）：")
print(f"    work_over_20ms  = {misses}")
print(f"    overload_stops  = {overloads}")
print(f"    dma_overflows   = {dma_ov}")
print(f"    queue_highwater = {qhi}")
