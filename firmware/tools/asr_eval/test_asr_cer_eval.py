"""Synthetic tests for the offline HMI/Qwen pattern report contract."""
from __future__ import annotations

import argparse
import importlib.util
import sys
import tempfile
import unittest
import wave
from pathlib import Path


SCRIPT = Path(__file__).with_name("asr_cer_eval.py")
SPEC = importlib.util.spec_from_file_location("asr_cer_eval", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
EVAL = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = EVAL
SPEC.loader.exec_module(EVAL)


class PatternReportTests(unittest.TestCase):
    def test_report_scores_character_cer_even_when_legacy_unit_is_word(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            manifest = Path(temporary) / "manifest.tsv"
            manifest.write_text("id\twav\tref\n", encoding="utf-8")
            source = Path(temporary) / "noise_s0_babble_snr0.wav"
            with wave.open(str(source), "wb") as audio:
                audio.setnchannels(1)
                audio.setsampwidth(2)
                audio.setframerate(16000)
                audio.writeframes(b"\x00\x00" * 320)
            item = EVAL.Item("noise_s0_babble_snr0", source, "你好")
            conditions = [
                EVAL.Condition("m0_g1", 0, 1.0),
                EVAL.Condition("m4_g1", 4, 1.0),
            ]
            rows = [
                {"id": item.uid, "condition": "m0_g1", "ref": "你好", "hyp": "你好",
                 "dist": 0, "ref_len": 1, "cer": 0.0, "rms_dbfs": -20.0},
                {"id": item.uid, "condition": "m4_g1", "ref": "你好", "hyp": "你号",
                 "dist": 1, "ref_len": 1, "cer": 1.0, "rms_dbfs": -21.0},
            ]
            summary = {"conditions": {condition.label: {} for condition in conditions}}
            args = argparse.Namespace(
                keep_punct=False, unit="word", level_normalize="input",
                bootstrap=100, seed=5, manifest=manifest,
            )

            report = EVAL.pattern_report(
                summary, rows, conditions, "m0_g1", [item], args,
                {"recognizer": "synthetic"},
            )

        arm = report["arms"][1]
        self.assertEqual(arm["coverage"]["raw"]["cer"], {"errors": 1, "n": 2})
        self.assertEqual(arm["rows"][0]["raw_score"]["cer"], {"errors": 1, "n": 2})
        self.assertAlmostEqual(summary["conditions"]["m4_g1"]["micro_cer"], 0.5)
        self.assertEqual(arm["conditions"]["strata"][0]["pattern"], "babble")
        self.assertEqual(arm["conditions"]["strata"][0]["snr_db"], 0)
        self.assertIn("whitespace_tokens", arm["conditions"]["wer_policy"])
        self.assertEqual(report["conditions"]["inputs"][0]["channels"], 1)
        self.assertEqual(report["conditions"]["dsp_input_channels"], 1)
        self.assertFalse(report["conditions"]["physical_device_tested"])
        self.assertFalse(report["conditions"]["audio_capture_performed"])

    def test_noise_pattern_requires_stable_pattern_and_snr_fields(self) -> None:
        self.assertEqual(EVAL.noise_pattern("noise_s2_shape_snr-5_variantA"), ("shape", -5))
        self.assertIsNone(EVAL.noise_pattern("noise_shape_minus5"))

    def test_native_worker_frames_are_length_bounded_and_round_trip(self) -> None:
        value = {"type": "Commit", "text": "中文"}
        self.assertEqual(EVAL.native_worker_frames(EVAL.native_worker_frame(value)), [value])
        with self.assertRaises(RuntimeError):
            EVAL.native_worker_frames(b"\x00\x00")


if __name__ == "__main__":
    unittest.main()
