"""Set or read the dual-microphone spatial mode through the Dongle's HID config
channel.

Protocol (firmware/dongle/main/app_main.c:dongle_dsp_request, and
docs/WhisPromptå®¢æˆ·ç«¯é€šä¿¡åè®®_1028.md):
    host -> device   SET_REPORT(report_id 3) or SET_REPORT(FEATURE, 3), 16 bytes
    device -> host   GET_FEATURE(report_id 3) read with length 19

    byte 0    0x11                      DSP request command
    byte 1    version                   spatial items 17..22 require 3
    byte 2    direction                 1 = read, 2 = write
    byte 3    item                      17 caps, 18 requested, 19 applied,
                                        20 layout, 21 fallback, 22 generation
    bytes 4-7 token                    echoed back
    bytes 8-9 value                    little endian
    byte 10   status (reply)           1 = queued/ok, 4 = forward rejected
    bytes 11-15                        must be zero on request

Usage:
    python set_spatial.py --read
    python set_spatial.py --mode 4
"""
from __future__ import annotations

import argparse
import random
import sys
import time

import hid

VID, PID = 0x303A, 0x8001
USAGE_PAGE = 0xFF00
REPORT_ID = 3
REPORT_SIZE = 16
READ_LEN = 19
ITEM_CAPS, ITEM_REQUESTED, ITEM_APPLIED, ITEM_LAYOUT, ITEM_FALLBACK, ITEM_GENERATION = 17, 18, 19, 20, 21, 22
VERSION_SPATIAL = 3
DIR_READ, DIR_WRITE = 1, 2
# 固件 0.5.27 起只暴露 bypass(0) 与 stereo Wiener(5)。
# 其余模式保留名称仅用于解码历史日志；设置时会被本地拦截。
MODE_NAMES = {0: "bypass", 1: "DAS", 2: "NLMS", 3: "NLMS stereo",
              4: "DAS+Wiener", 5: "stereo Wiener", 6: "soft-mask MVDR"}
SUPPORTED_MODES = (0, 5)
REMOVED_MODES = (1, 2, 3, 4, 6)


def require_supported(mode: int) -> None:
    """在发往设备前拦住已下线的模式，避免只拿到一个 status 错误码。"""
    if mode in REMOVED_MODES:
        raise SystemExit(
            f"spatial mode {mode} ({MODE_NAMES.get(mode, '?')}) was removed in firmware 0.5.27; "
            f"supported modes are {SUPPORTED_MODES}"
        )


def open_config():
    for d in hid.enumerate(VID, PID):
        if d.get("usage_page") == USAGE_PAGE:
            h = hid.device()
            h.open_path(d["path"])
            h.set_nonblocking(False)
            return h
    raise SystemExit("vendor-defined HID interface (usage_page 0xFF00) not found")


def _send(h, payload: bytes) -> int:
    # Send as an OUTPUT report first: a FEATURE SET leaves the host-side feature
    # cache holding our own request, so the following GET_FEATURE would echo it
    # back instead of returning the device's reply.
    try:
        n = h.write(payload)
    except Exception:
        n = 0
    if n <= 0:
        try:
            n = h.send_feature_report(payload)
        except Exception:
            n = 0
    return n


def _read(h):
    try:
        raw = h.get_feature_report(REPORT_ID, READ_LEN)
    except Exception as exc:
        return None, f"get_feature_report failed: {exc}"
    if not raw:
        return None, "empty feature reply"
    reply = bytes(raw)
    # hidapi normally strips the report id; tolerate builds that keep it.
    if reply and reply[0] != 0x11:
        reply = reply[1:]
    return reply, None


def request(h, version: int, item: int, value: int = 0, direction: int = DIR_READ):
    """Send one DSP request and return (value, error).

    The reply lives in a single shared 16-byte buffer: a request rejected before
    dongle_dsp_request() leaves the previous content untouched, so we poll for a
    matching token and a real status instead of trusting the first read.
    """
    token = random.randrange(1, 0xFFFFFFFF)
    buf = bytearray(REPORT_SIZE)
    buf[0] = 0x11
    buf[1] = version
    buf[2] = direction
    buf[3] = item
    buf[4:8] = token.to_bytes(4, "little")
    buf[8:10] = (value & 0xFFFF).to_bytes(2, "little")
    payload = bytes([REPORT_ID]) + bytes(buf)

    if _send(h, payload) <= 0:
        return None, "send failed"
    time.sleep(0.03)
    last = "no reply"
    for attempt in range(10):
        reply, err = _read(h)
        if reply is not None and len(reply) >= REPORT_SIZE:
            if int.from_bytes(reply[4:8], "little") == token:
                status = reply[10]
                if status == 1:
                    return int.from_bytes(reply[8:10], "little"), None
                if status == 4:
                    last = "status=4 (forward rejected: link down or queue full)"
                else:
                    last = f"status={status}"
        if attempt in (3, 6):
            _send(h, payload)
        time.sleep(0.03)
    return None, last


ITEM_CODEC = 15
VERSION_CODEC = 2
# 固件 0.5.27 起只暴露 G.722；Opus 名称仅用于解码历史日志。
CODEC_NAMES = {4: "G.722 64k", 3: "Opus 32k", 5: "Opus 62k"}
SUPPORTED_CODECS = (4,)
REMOVED_CODECS = (3, 5)


def describe_caps(caps: int) -> str:
    """把能力位翻译成可读结论，避免沿用旧的 63/6 模式判定。"""
    has_bypass = True
    has_wiener = (caps & 0x10) == 0x10
    has_legacy = (caps & 0x0F) != 0 or (caps & 0x20) != 0
    parts = ["bypass"] + (["stereo Wiener"] if has_wiener else [])
    text = f"spatial=0x{caps:02x} -> " + " + ".join(parts)
    if has_legacy:
        text += "  (legacy bits also set; unexpected for 0.5.27+)"
    if not has_wiener:
        text += "  WARNING: stereo Wiener missing"
    return text


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--read", action="store_true")
    ap.add_argument("--mode", type=int,
                    help="spatial mode; firmware 0.5.27 accepts only 0 (bypass) and 5 (stereo Wiener)")
    ap.add_argument("--codec", type=int, help="4=G.722 (the only codec in firmware 0.5.27)")
    args = ap.parse_args()

    if args.mode is not None:
        require_supported(args.mode)
    if args.codec is not None and args.codec in REMOVED_CODECS:
        raise SystemExit(
            f"codec {args.codec} ({CODEC_NAMES.get(args.codec, '?')}) was removed in firmware 0.5.27; "
            f"supported codecs are {SUPPORTED_CODECS} (G.722)"
        )

    h = open_config()
    if args.codec is not None:
        got, err = request(h, VERSION_CODEC, ITEM_CODEC, args.codec, DIR_WRITE)
        print(f"set codec -> {args.codec} ({CODEC_NAMES.get(args.codec, '?')})"
              + ("" if got is not None else f"  [reply: {err}]"))
        time.sleep(0.5)
    caps, err = request(h, VERSION_SPATIAL, ITEM_CAPS)
    if caps is None:
        print(f"caps read failed: {err}")
    else:
        print("capabilities " + describe_caps(caps))

    if args.mode is not None:
        got, err = request(h, VERSION_SPATIAL, ITEM_REQUESTED, args.mode, DIR_WRITE)
        if got is None:
            print(f"set mode {args.mode} FAILED: {err}")
            h.close()
            return 1
        print(f"set spatial mode -> {got} ({MODE_NAMES.get(got, '?')})")
        time.sleep(0.3)

    for item, name in ((ITEM_REQUESTED, "requested"), (ITEM_APPLIED, "applied"),
                       (ITEM_LAYOUT, "layout"), (ITEM_FALLBACK, "fallback"),
                       (ITEM_GENERATION, "generation")):
        v, err = request(h, VERSION_SPATIAL, item)
        extra = ""
        if v is not None and item in (ITEM_REQUESTED, ITEM_APPLIED):
            extra = f"  ({MODE_NAMES.get(v, '?')})"
        print(f"  {name:11s} = {v}{extra}" if v is not None else f"  {name:11s} : {err}")
    h.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

