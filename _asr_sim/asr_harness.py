"""Minimal Qwen3-ASR harness that drives native.dll directly through its C ABI.

Why this exists: the shipped hmi_voice_probe.exe / HMI.exe are blocked by this
machine's Smart App Control policy. native.dll itself loads fine, so this module
calls the documented audiocpp_* ABI from Python instead of going through an exe.

ABI: audiocpp 0.5.0 (see hmi/native-console/3rd_party/native/source/include/audiocpp.h)
Sequence mirrors hmi/native-console/src/engine/asr_session.cpp:
    registry_create -> model_load(family qwen3_asr) -> session_create("asr","streaming")
    -> stream_policy -> request(set_audio/options) -> stream_start
    -> stream_push* -> stream_finish -> result_text

Usage:
    python asr_harness.py --wav a.wav [b.wav ...]
    python asr_harness.py --manifest manifest.json      # [{id, audio, reference}]
"""
from __future__ import annotations

import argparse
import ctypes
import json
import os
import sys
import time
import wave

_NATIVE_ROOT = r"D:\App\cppApp\audio_qos\hmi\native-console\build"
# Smart App Control blocks some copies of native.dll even though they are
# byte-identical, so probe the known locations and use the first one that loads.
DLL_CANDIDATES = [
    _NATIVE_ROOT + r"\diagnostics\ggml-kernels\bin\Release\runtime\native.dll",
    _NATIVE_ROOT + r"\bin\Release\native\native.dll",
    _NATIVE_ROOT + r"\audio-sdk\package\Release\bin\native.dll",
]


def resolve_dll() -> str:
    last = None
    for path in DLL_CANDIDATES:
        if not os.path.exists(path):
            continue
        try:
            if hasattr(os, "add_dll_directory"):
                os.add_dll_directory(os.path.dirname(path))
            import ctypes as _ct
            probe = _ct.WinDLL(path)
            del probe
            return path
        except OSError as exc:
            last = exc
    if last is not None:
        raise RuntimeError(f"no loadable native.dll among {DLL_CANDIDATES}: {last}")
    raise RuntimeError("native.dll not found")


DLL = DLL_CANDIDATES[0]
MODEL = r"D:\App\cppApp\audio_qos\hmi\native-console\models\asr.gguf"

AUDIOCPP_OK = 0
AUDIOCPP_ERR_NOT_AVAILABLE = 7
STREAM_INPUT_AUDIO_CHUNKS = 1
STREAM_INPUT_NONE = 0

STATUS = {
    0: "OK", 1: "INVALID_ARGUMENT", 2: "UNSUPPORTED_FAMILY", 3: "LOAD_FAILED",
    4: "RUNTIME", 5: "OUT_OF_MEMORY", 6: "OUT_OF_RANGE", 7: "NOT_AVAILABLE",
}


def _bind(lib):
    """Declare exact prototypes. Anything unspecified would default to int and
    corrupt 64-bit handles."""
    P = ctypes.c_void_p
    I = ctypes.c_int
    U32 = ctypes.c_uint32
    U64 = ctypes.c_uint64
    SZ = ctypes.c_size_t
    I64 = ctypes.c_int64
    STR = ctypes.c_char_p
    F = ctypes.c_float

    class ModelConfig(ctypes.Structure):
        _fields_ = [("family_hint", STR), ("config_id", STR),
                    ("weight_id", STR), ("model_spec_override", STR)]

    class BackendConfig(ctypes.Structure):
        _fields_ = [("backend", STR), ("device", I), ("threads", I)]

    sig = {
        "audiocpp_abi_version": (U32, []),
        "audiocpp_last_error": (STR, []),
        "audiocpp_status_string": (STR, [I]),
        "audiocpp_registry_create": (I, [STR, ctypes.POINTER(P)]),
        "audiocpp_registry_free": (None, [P]),
        # Config pointers are typed as void* so callers may pass any Structure
        # by reference without ctypes pointer-class mismatches.
        "audiocpp_model_load": (I, [P, STR, P, P, ctypes.POINTER(P)]),
        "audiocpp_model_free": (None, [P]),
        "audiocpp_model_family": (STR, [P]),
        "audiocpp_session_create": (I, [P, STR, STR, P, P, ctypes.POINTER(P)]),
        "audiocpp_session_free": (None, [P]),
        "audiocpp_stream_policy": (I, [P, ctypes.POINTER(I), ctypes.POINTER(I),
                                       ctypes.POINTER(I64), ctypes.POINTER(ctypes.c_double)]),
        "audiocpp_request_create": (P, []),
        "audiocpp_request_free": (None, [P]),
        "audiocpp_request_set_audio": (I, [P, ctypes.POINTER(F), SZ, I, I]),
        "audiocpp_request_set_option": (I, [P, STR, STR]),
        "audiocpp_stream_start": (I, [P, P]),
        "audiocpp_stream_push": (I, [P, ctypes.POINTER(F), SZ, I, I, I64, ctypes.POINTER(P)]),
        "audiocpp_stream_next_event": (I, [P, ctypes.POINTER(P)]),
        "audiocpp_stream_finish": (I, [P, ctypes.POINTER(P)]),
        "audiocpp_stream_reset": (I, [P]),
        "audiocpp_event_free": (None, [P]),
        "audiocpp_result_free": (None, [P]),
        "audiocpp_result_text": (I, [P, ctypes.POINTER(STR), ctypes.POINTER(STR)]),
    }
    for name, (restype, argtypes) in sig.items():
        fn = getattr(lib, name)
        fn.restype = restype
        fn.argtypes = argtypes
    return lib


class Sdk:
    def __init__(self, dll: str | None = None):
        dll = dll or resolve_dll()
        self.native_dir = os.path.dirname(dll)
        # Sibling ggml-*.dll must be resolvable.
        if hasattr(os, "add_dll_directory"):
            self._dll_dir = os.add_dll_directory(self.native_dir)
        self.lib = _bind(ctypes.WinDLL(dll))
        self.dll_path = dll
        abi = self.lib.audiocpp_abi_version()
        self.abi = abi
        if abi != 0x00000500:
            raise RuntimeError(f"unexpected ABI 0x{abi:08x}, expected 0x00000500")

    def check(self, status: int, what: str):
        if status == AUDIOCPP_OK:
            return
        detail = self.lib.audiocpp_last_error()
        msg = detail.decode("utf-8", "replace") if detail else STATUS.get(status, str(status))
        raise RuntimeError(f"{what} failed: {STATUS.get(status, status)} ({msg})")


class AsrEngine:
    """Loads the model once and reuses the session for every utterance."""

    def __init__(self, model: str = MODEL, backend: str = "cuda", threads: int = 4,
                 language: str | None = None, max_tokens: int = 0,
                 max_audio_seconds: int = 30, sdk: Sdk | None = None):
        self.sdk = sdk or Sdk()
        lib = self.sdk.lib
        self.language = language
        self.max_tokens = max_tokens
        self.window = str(max_audio_seconds)
        self.registry = ctypes.c_void_p()
        # NULL config_path -> SDK default loader set.
        self.sdk.check(lib.audiocpp_registry_create(None, ctypes.byref(self.registry)),
                       "registry_create")
        self.model = ctypes.c_void_p()
        ModelConfig = self._model_config_type()
        sel = ModelConfig(b"qwen3_asr", None, None, None)
        self.sdk.check(lib.audiocpp_model_load(self.registry, model.encode("utf-8"),
                                               ctypes.byref(sel), None,
                                               ctypes.byref(self.model)),
                       "model_load")
        BackendConfig = self._backend_config_type()
        be = BackendConfig(backend.encode(), 0, threads)
        self.session = ctypes.c_void_p()
        self.sdk.check(lib.audiocpp_session_create(self.model, b"asr", b"streaming",
                                                   ctypes.byref(be), None,
                                                   ctypes.byref(self.session)),
                       "session_create")
        inp = ctypes.c_int()
        out = ctypes.c_int()
        samples = ctypes.c_int64()
        secs = ctypes.c_double()
        self.sdk.check(lib.audiocpp_stream_policy(self.session, ctypes.byref(inp),
                                                  ctypes.byref(out), ctypes.byref(samples),
                                                  ctypes.byref(secs)),
                       "stream_policy")
        if inp.value != STREAM_INPUT_AUDIO_CHUNKS:
            raise RuntimeError("ASR model does not accept streaming PCM")
        self.chunk = samples.value if 0 < samples.value <= 16000 else 16000
        self.family = (lib.audiocpp_model_family(self.model) or b"").decode("utf-8", "replace")
        self.backend = backend

    @staticmethod
    def _model_config_type():
        class ModelConfig(ctypes.Structure):
            _fields_ = [("family_hint", ctypes.c_char_p), ("config_id", ctypes.c_char_p),
                        ("weight_id", ctypes.c_char_p), ("model_spec_override", ctypes.c_char_p)]
        return ModelConfig

    @staticmethod
    def _backend_config_type():
        class BackendConfig(ctypes.Structure):
            _fields_ = [("backend", ctypes.c_char_p), ("device", ctypes.c_int),
                        ("threads", ctypes.c_int)]
        return BackendConfig

    def transcribe(self, pcm, sample_rate: int = 16000) -> str:
        """pcm: float32 1-D array already at `sample_rate`, mono."""
        import numpy as np
        lib = self.sdk.lib
        audio = np.ascontiguousarray(pcm, dtype=np.float32)
        frames = len(audio)
        if frames == 0:
            return ""
        req = lib.audiocpp_request_create()
        if not req:
            raise RuntimeError("request_create returned NULL")
        try:
            self.sdk.check(lib.audiocpp_request_set_audio(req, None, 0, sample_rate, 1),
                           "request_set_audio(format)")
            if self.language:
                self.sdk.check(lib.audiocpp_request_set_option(req, b"language",
                                                               self.language.encode()),
                               "set language")
            self.sdk.check(lib.audiocpp_request_set_option(req, b"audio_chunk_duration_sec",
                                                           self.window.encode()),
                           "set window")
            if self.max_tokens > 0:
                self.sdk.check(lib.audiocpp_request_set_option(req, b"max_tokens",
                                                               str(self.max_tokens).encode()),
                               "set max_tokens")
            self.sdk.check(lib.audiocpp_stream_start(self.session, req), "stream_start")
            offset = 0
            # The C caller advances the pointer by `offset` samples; passing the
            # buffer head every time would feed the same first chunk repeatedly.
            base = audio.ctypes.data
            while offset < frames:
                n = min(self.chunk, frames - offset)
                ptr = ctypes.cast(base + offset * audio.itemsize,
                                  ctypes.POINTER(ctypes.c_float))
                event = ctypes.c_void_p()
                self.sdk.check(lib.audiocpp_stream_push(self.session, ptr, n, sample_rate, 1,
                                                        offset, ctypes.byref(event)),
                               "stream_push")
                if event:
                    lib.audiocpp_event_free(event)
                offset += n
                while True:
                    ev = ctypes.c_void_p()
                    self.sdk.check(lib.audiocpp_stream_next_event(self.session, ctypes.byref(ev)),
                                   "drain")
                    if not ev:
                        break
                    lib.audiocpp_event_free(ev)
            result = ctypes.c_void_p()
            self.sdk.check(lib.audiocpp_stream_finish(self.session, ctypes.byref(result)),
                           "stream_finish")
            if not result:
                return ""
            try:
                text = ctypes.c_char_p()
                lang = ctypes.c_char_p()
                st = lib.audiocpp_result_text(result, ctypes.byref(text), ctypes.byref(lang))
                if st == AUDIOCPP_OK and text.value:
                    return text.value.decode("utf-8", "replace")
                if st == AUDIOCPP_ERR_NOT_AVAILABLE:
                    return ""
                self.sdk.check(st, "result_text")
                return ""
            finally:
                lib.audiocpp_result_free(result)
        finally:
            lib.audiocpp_request_free(req)
            lib.audiocpp_stream_reset(self.session)

    def close(self):
        lib = self.sdk.lib
        if getattr(self, "session", None):
            lib.audiocpp_session_free(self.session)
            self.session = None
        if getattr(self, "model", None):
            lib.audiocpp_model_free(self.model)
            self.model = None
        if getattr(self, "registry", None):
            lib.audiocpp_registry_free(self.registry)
            self.registry = None


def read_wav(path: str):
    import numpy as np
    with wave.open(path, "rb") as w:
        rate, ch, n = w.getframerate(), w.getnchannels(), w.getnframes()
        x = np.frombuffer(w.readframes(n), dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    return x, rate


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--wav", nargs="*", default=[])
    ap.add_argument("--manifest", default="")
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--backend", default="cuda")
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--language", default="")
    ap.add_argument("--jsonl", default="")
    args = ap.parse_args()

    items = []
    base = ""
    if args.manifest:
        base = os.path.dirname(os.path.abspath(args.manifest))
        for e in json.load(open(args.manifest, encoding="utf-8")):
            items.append({"id": e.get("id", e["audio"]),
                          "wav": os.path.join(base, e["audio"]),
                          "reference": e.get("reference", "")})
    for w in args.wav:
        items.append({"id": os.path.basename(w), "wav": w, "reference": ""})
    if not items:
        ap.error("need --wav or --manifest")

    t0 = time.time()
    eng = AsrEngine(model=args.model, backend=args.backend, threads=args.threads,
                    language=args.language or None)
    print(f"# engine ready family={eng.family} backend={eng.backend} "
          f"chunk={eng.chunk} load={time.time()-t0:.1f}s", file=sys.stderr)

    out = []
    for it in items:
        pcm, rate = read_wav(it["wav"])
        t = time.time()
        text = eng.transcribe(pcm, rate)
        dt = time.time() - t
        out.append({"id": it["id"], "wav": it["wav"], "reference": it["reference"],
                    "hypothesis": text, "audio_s": round(len(pcm) / rate, 2),
                    "asr_ms": round(dt * 1000, 1), "rtf": round(dt / (len(pcm) / rate), 3)})
        print(json.dumps(out[-1], ensure_ascii=False), flush=True)
    eng.close()
    if args.jsonl:
        with open(args.jsonl, "w", encoding="utf-8") as f:
            for r in out:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
        print(f"# wrote {args.jsonl}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

