#!/usr/bin/env python3
"""Read/write Scepter DSP parameters through the Dongle HID feature interface.

Protocol (see firmware README):
  radio payload : [version, op, item, token32 LE, value16 LE, status, reserved x3]
  USB feature   : [0x11, <13-byte radio payload>, reserved x2] on HID report ID 3

Items:
  15 codec (4 = G.722, 3 = Opus-32, 5 = Opus-62)   version 2
  16 codec capabilities (read only)                version 2
  17 spatial capabilities (read only)              version 3
  18 spatial requested mode (0..5)                 version 3
  19 spatial applied mode (read only)              version 3
"""
from __future__ import annotations

import argparse
import struct
import time

import hid

VID, PID = 0x303A, 0x8001
VENDOR_USAGE_PAGE = 0xFF00
REPORT_ID = 0x03
OP_GET, OP_SET = 1, 2

ITEMS = {
    15: ("codec", 2),
    16: ("codec_caps", 2),
    17: ("spatial_caps", 3),
    18: ("spatial_mode", 3),
    19: ("spatial_applied", 3),
}

SPATIAL_MODES = {
    0: "bypass (stereo passthrough)",
    1: "DAS",
    2: "NLMS",
    3: "NLMS stereo",
    4: "DAS + Wiener (dual mono)",
    5: "independent stereo Wiener",
}

CODECS = {3: "Opus 32k", 4: "G.722 64k", 5: "Opus 62k"}


def open_device() -> hid.device:
    for info in hid.enumerate(VID, PID):
        if info.get("usage_page") == VENDOR_USAGE_PAGE:
            device = hid.device()
            device.open_path(info["path"])
            return device
    raise SystemExit("no vendor HID interface found (is the Dongle plugged in?)")


def xfer(device: hid.device, item: int, op: int, value: int = 0,
         token: int = 0) -> dict:
    version = ITEMS.get(item, ("", 1))[1]
    payload = (bytes([0x11, version, op, item])
               + struct.pack("<I", token)
               + struct.pack("<H", value)
               + bytes([0, 0, 0, 0])
               + bytes([0, 0]))
    assert len(payload) == 16, len(payload)
    device.send_feature_report(bytes([REPORT_ID]) + payload)
    time.sleep(0.08)
    raw = device.get_feature_report(REPORT_ID, 17)
    if len(raw) < 12:
        raise SystemExit(f"short response: {list(raw)}")
    return {
        "version": raw[2],
        "op": raw[3],
        "item": raw[4],
        "token": int.from_bytes(bytes(raw[5:9]), "little"),
        "value": int.from_bytes(bytes(raw[9:11]), "little"),
        "status": raw[11],
        "raw": list(raw),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--get", type=int, metavar="ITEM")
    parser.add_argument("--set", nargs=2, type=int, metavar=("ITEM", "VALUE"))
    parser.add_argument("--mode", type=int, help="shortcut for --set 18 MODE")
    parser.add_argument("--codec", type=int, help="shortcut for --set 15 CODEC")
    parser.add_argument("--all", action="store_true", help="read items 15..19")
    args = parser.parse_args()

    device = open_device()
    try:
        if args.all or (args.get is None and args.set is None
                        and args.mode is None and args.codec is None):
            for item in (15, 16, 17, 18, 19):
                result = xfer(device, item, OP_GET)
                name = ITEMS[item][0]
                extra = ""
                if item == 18 or item == 19:
                    extra = f"  ({SPATIAL_MODES.get(result['value'], '?')})"
                elif item == 15:
                    extra = f"  ({CODECS.get(result['value'], '?')})"
                print(f"{item:>2} {name:<18} = {result['value']:<6}"
                      f" status={result['status']}{extra}")
            return 0

        if args.mode is not None:
            args.set = [18, args.mode]
        if args.codec is not None:
            args.set = [15, args.codec]

        if args.get is not None:
            result = xfer(device, args.get, OP_GET)
            print(f"{args.get}: value={result['value']} status={result['status']}")
            return 0

        item, value = args.set
        result = xfer(device, item, OP_SET, value=value)
        print(f"SET item {item} = {value} -> status={result['status']} "
              f"reported={result['value']}")
        if result["status"] != 0:
            print("warning: device reported a non-zero status")
            return 1
        return 0
    finally:
        device.close()


if __name__ == "__main__":
    raise SystemExit(main())
