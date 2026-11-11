#!/usr/bin/env python3
"""Ten-minute scenario run across several spatial patterns.

For each pattern the script plays a scenario recording (whisper + noise, already
rendered by box_sim) through the artificial mouth, records the Dongle UAC stream
and writes one wav per round.  Firmware telemetry from both serial ports is
captured in the background so the summary can report link health, encode time
and log drops for every pattern.

Usage:
    python scene_run.py --minutes 10 --modes 0,1,4,5
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import wave
from pathlib import Path

import numpy as np
import serial
import sounddevice as sd

SR = 48000
OSR = 16000
MOUTH = 1
DSP = str(Path(__file__).resolve().with_name("dsp_param.py"))


def find(name: str, want_out: bool) -> int:
    for index, info in enumerate(sd.query_devices()):
        if name in info["name"]:
            if want_out and info["max_output_channels"] >= 4:
                return index
            if not want_out and info["max_input_channels"] >= 2:
                return index
    raise SystemExit(f"device {name} not found")


def load(path: str) -> np.ndarray:
    with wave.open(path, "rb") as handle:
        channels, rate = handle.getnchannels(), handle.getframerate()
        raw = handle.readframes(handle.getnframes())
    x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if channels > 1:
        x = x.reshape(-1, channels).mean(axis=1)
    k = int(len(x) * SR / rate)
    return np.interp(np.linspace(0, len(x) - 1, k), np.arange(len(x)), x).astype(np.float32)


def resample16(x: np.ndarray) -> np.ndarray:
    k = int(len(x) * OSR / SR)
    return np.interp(np.linspace(0, len(x) - 1, k), np.arange(len(x)), x).astype(np.float32)


def set_dsp(args: list[str]) -> str:
    result = subprocess.run([sys.executable, DSP] + args, capture_output=True, text=True, timeout=30)
    return result.stdout.strip()


class Telemetry:
    def __init__(self, port: str, path: str) -> None:
        self.port = port
        self.path = path
        self.stop = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        try:
            ser = serial.Serial(self.port, 115200, timeout=0.2)
        except Exception as exc:  # noqa: BLE001
            with open(self.path, "w", encoding="utf-8") as handle:
                handle.write(f"open failed: {exc}\n")
            return
        # The Dongle only mirrors ESP_LOG while its CDC log switch is on.
        if self.port.upper() == "COM4":
            try:
                ser.write(b"LOG 1\r\n")
                ser.flush()
            except Exception:
                pass
        with open(self.path, "w", encoding="utf-8") as handle:
            while not self.stop:
                line = ser.readline()
                if line:
                    handle.write(line.decode("utf-8", "replace"))
                    handle.flush()
        if self.port.upper() == "COM4":
            try:
                ser.write(b"LOG 0\r\n")
                ser.flush()
            except Exception:
                pass
        ser.close()

    def start(self) -> None:
        self.thread.start()

    def shutdown(self) -> None:
        self.stop = True
        self.thread.join(timeout=3)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--minutes", type=float, default=10.0)
    parser.add_argument("--modes", default="0,1,4,5")
    parser.add_argument("--scene", required=True, help="scenario WAV to play through the artificial mouth")
    parser.add_argument("--out", default="scene-run")
    args = parser.parse_args()

    modes = [int(v) for v in args.modes.split(",") if v.strip()]
    os.makedirs(args.out, exist_ok=True)
    speech = load(args.scene)
    out_dev, in_dev = find("US-4x4HR", True), find("Audio QoS", False)
    per_mode = args.minutes * 60.0 / max(len(modes), 1)
    summary = []

    tele_a = Telemetry("COM6", os.path.join(args.out, "telemetry-scepter.log"))
    tele_b = Telemetry("COM4", os.path.join(args.out, "telemetry-dongle.log"))
    tele_a.start()
    tele_b.start()
    try:
        for mode in modes:
            set_dsp(["--mode", str(mode)])
            time.sleep(1.0)
            applied = set_dsp(["--get", "19"])
            rounds = 0
            deadline = time.time() + per_mode
            peaks, rms = [], []
            while time.time() < deadline:
                n = len(speech) + int(0.6 * SR)
                buf = np.zeros((n, 4), dtype=np.float32)
                start = int(0.3 * SR)
                buf[start:start + len(speech), MOUTH] = speech * 0.5
                chunks: list[np.ndarray] = []

                def cb(indata, frames, time_info, status):  # noqa: ANN001
                    chunks.append(indata.copy())

                stream = sd.InputStream(device=in_dev, channels=2, samplerate=SR,
                                        dtype="float32", callback=cb)
                stream.start()
                time.sleep(0.1)
                try:
                    sd.play(buf, samplerate=SR, device=out_dev, blocking=True)
                    time.sleep(0.2)
                finally:
                    stream.stop()
                    stream.close()
                rec = np.concatenate(chunks) if chunks else np.zeros((1, 2), np.float32)
                stereo = np.stack([resample16(rec[:, 0]), resample16(rec[:, 1])], axis=1)
                peaks.append(float(np.max(np.abs(stereo))))
                rms.append(float(np.sqrt(np.mean(stereo.astype(np.float64) ** 2))))
                path = os.path.join(args.out, f"m{mode}_{rounds:03d}.wav")
                pcm = np.clip(stereo * 32767, -32768, 32767).astype("<i2")
                with wave.open(path, "wb") as handle:
                    handle.setnchannels(2)
                    handle.setsampwidth(2)
                    handle.setframerate(OSR)
                    handle.writeframes(pcm.tobytes())
                rounds += 1
            summary.append({
                "mode": mode, "applied": applied, "rounds": rounds,
                "peak_mean": round(float(np.mean(peaks)), 3),
                "rms_db_mean": round(20 * np.log10(float(np.mean(rms)) + 1e-12), 1),
            })
            print(f"mode {mode}: rounds={rounds} peak={summary[-1]['peak_mean']} "
                  f"rms={summary[-1]['rms_db_mean']} dBFS", flush=True)
    finally:
        set_dsp(["--mode", "0"])
        tele_a.shutdown()
        tele_b.shutdown()

    with open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    print("SUMMARY", json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
