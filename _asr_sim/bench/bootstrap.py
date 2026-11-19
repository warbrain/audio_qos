"""统一入口引导：在任何输出之前把编码和子进程环境摆正。

必须最先 import。历史上踩过的坑：
  * 控制台是 cp1252/cp936 时，打印中文直接抛 UnicodeEncodeError，采集循环在第 1 条就崩；
  * 用 subprocess 调 box_capture 时没继承 PYTHONIOENCODING，子进程同样崩；
  * 这两个问题都表现为"只采到 1 条 / 没有输出"，很难回溯。
"""
from __future__ import annotations

import os
import sys

# 本进程的标准输出/错误一律 UTF-8
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# 子进程继承的环境：任何 python 子进程都按 UTF-8 运行
os.environ.setdefault("PYTHONIOENCODING", "utf-8")
os.environ.setdefault("PYTHONUTF8", "1")

# 子进程命令里始终带上 -X utf8（环境变量被父进程清理时的兜底）
PYTHON = sys.executable
PY_FLAGS = ["-X", "utf8"]


def child_env() -> dict:
    """给 subprocess 用的环境副本。"""
    return dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1")
