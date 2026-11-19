"""直接运行宿主测试函数（绕过 pytest-embedded 的收集约束）。

这些测试本质是普通函数：接收 tmp_path，内部编译 C 代码并跑子进程断言。
在 IDF 的 pytest 环境里无法收集（pytest-embedded 要求 parametrize 带 target），
所以这里直接 import 模块并调用函数。

约定：
  * 展开 pytest.mark.parametrize 的参数组合（仅位置参数）；
  * 带 pytest.mark.skip 的函数整体跳过；
  * 其余按签名传 tmp_path 或直接调用。

用法：
    python run_host_tests.py <module> [func ...]
"""
from __future__ import annotations

import importlib
import inspect
import pathlib
import sys
import tempfile
import traceback

TESTS = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS))

TMP_ARGS = ("tmp_path", "tmp_path_factory")


def skip_reason(fn) -> str | None:
    for mark in getattr(fn, "pytestmark", []) or []:
        if getattr(mark, "name", "") == "skip":
            kw = getattr(mark, "kwargs", {}) or {}
            return kw.get("reason") or "marked skip"
    return None


def cases(fn) -> list[dict]:
    """Return the parametrize combinations as kwargs dicts ([] when unparametrized)."""
    combos: list[dict] = []
    for mark in getattr(fn, "pytestmark", []) or []:
        if getattr(mark, "name", "") != "parametrize":
            continue
        args = getattr(mark, "args", ()) or ()
        if len(args) < 2:
            continue
        names = args[0]
        values = args[1]
        if isinstance(names, str):
            names = [n.strip() for n in names.split(",")]
        for row in values:
            vals = row if isinstance(row, (tuple, list)) else (row,)
            combos.append(dict(zip(names, vals)))
    return combos


def call(fn, kw: dict, tmp: pathlib.Path) -> None:
    params = inspect.signature(fn).parameters
    if "tmp_path" in params:
        kw = dict(kw, tmp_path=tmp)
    fn(**kw)


def run(module_name: str, func_names: list[str] | None) -> int:
    mod = importlib.import_module(module_name)
    names = func_names or [n for n in dir(mod) if n.startswith("test_")]
    ok = fail = skipped = 0
    for name in names:
        fn = getattr(mod, name, None)
        if fn is None or not callable(fn):
            print(f"  MISSING {module_name}.{name}")
            fail += 1
            continue
        reason = skip_reason(fn)
        if reason:
            print(f"  SKIP    {name}: {reason[:90]}")
            skipped += 1
            continue
        combos = cases(fn) or [{}]
        for kw in combos:
            label = f"{name}[{','.join(str(v) for v in kw.values())}]" if kw else name
            with tempfile.TemporaryDirectory(prefix="hosttest_") as td:
                try:
                    call(fn, kw, pathlib.Path(td))
                    print(f"  PASS    {label}")
                    ok += 1
                except Exception as exc:  # noqa: BLE001
                    print(f"  FAIL    {label}: {type(exc).__name__}: {str(exc)[:200]}")
                    traceback.print_exc(limit=3)
                    fail += 1
    print(f"\n  合计: {ok} 通过 / {fail} 失败 / {skipped} 跳过（跳过按函数计）")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: run_host_tests.py <module> [func ...]")
        raise SystemExit(2)
    raise SystemExit(run(sys.argv[1], sys.argv[2:] or None))
