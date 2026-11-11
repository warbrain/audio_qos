#!/usr/bin/env python3
"""Static scan for ESP_LOG records that can exceed the log transport limit.

The log transport rejects any formatted record whose text (prefix + body + LF)
does not fit DUEL_END_LOG_TRANSPORT_LINE_BYTES (256).  This scanner parses every
ESP_LOGx(...) call in the firmware sources, concatenates the format literals
(with width placeholders for PRI* macros), and reports calls whose estimated
length exceeds the limit.

Usage:
    python scan_logs.py [--root DIR] [--limit 256] [--top 0]
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

LOG_RE = re.compile(r"ESP_LOG[IWEDV]\s*\(")
PRI_RE = re.compile(r"PRI[a-zA-Z0-9_]+")
SKIP_DIRS = {"build", "build-local", "managed_components", ".git", "__pycache__"}


def extract_call(text: str, start: int) -> str:
    """Return the argument text of the call starting at the '(' index."""
    depth = 0
    in_str = False
    escape = False
    for index in range(start, len(text)):
        ch = text[index]
        if in_str:
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return text[start + 1:index]
    return text[start + 1:]


def literals_and_macros(args: str) -> tuple[str, str]:
    """Concatenate string literals, inserting width placeholders for PRI*."""
    out: list[str] = []
    index = 0
    while index < len(args):
        ch = args[index]
        if ch == '"':
            index += 1
            value = []
            while index < len(args):
                c = args[index]
                if c == "\\":
                    if index + 1 < len(args):
                        value.append(args[index + 1])
                        index += 2
                        continue
                elif c == '"':
                    index += 1
                    break
                value.append(c)
                index += 1
            out.append("".join(value))
            continue
        match = PRI_RE.match(args, index)
        if match:
            token = match.group(0)
            width = 12 if "64" in token else 8 if "32" in token else 5
            out.append("#" * width)
            index = match.end()
            continue
        index += 1
    return "".join(out), args


def scan_file(path: str, limit: int) -> list[tuple[int, str, int]]:
    text = open(path, encoding="utf-8", errors="replace").read()
    findings = []
    for match in LOG_RE.finditer(text):
        args = extract_call(text, match.end() - 1)
        body, _ = literals_and_macros(args)
        if len(body) + 1 > limit:
            line = text.count("\n", 0, match.start()) + 1
            findings.append((len(body) + 1, path, line, body[:80]))
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    default_root = Path(__file__).resolve().parents[2] / "firmware"
    parser.add_argument("--root", default=str(default_root))
    parser.add_argument("--limit", type=int, default=256)
    parser.add_argument("--top", type=int, default=0)
    args = parser.parse_args()

    rows = []
    for directory, dirs, files in os.walk(args.root):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for name in files:
            if name.endswith(".c"):
                rows.extend(scan_file(os.path.join(directory, name), args.limit))

    rows.sort(key=lambda r: -r[0])
    if args.top:
        rows = rows[: args.top]
    print(f"{len(rows)} ESP_LOG calls exceed {args.limit} bytes (estimated)\n")
    for length, path, line, body in rows:
        rel = os.path.relpath(path, args.root)
        print(f"{length:5d}  {rel}:{line}")
        print(f"       {body}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
