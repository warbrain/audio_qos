"""Check PCM/full state against unmodified 0.5.11 and count real predictions.

Only the prediction function entry is instrumented in temporary source copies;
the production implementation and frozen reference remain untouched. Host work
counts prove the optimization, not device timing or audio/ASR performance.
"""
from pathlib import Path

import pytest

from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
CODEC = ROOT / 'components/dual_end_audio_codec'


def _counted_source(source, counter):
    text = source.read_text(encoding='utf-8')
    marker = '    float matched=0.0f;\n    const size_t first=DUAL_END_SPATIAL_TAPS-head;'
    assert text.count(marker) == 1, 'Instrument the actual prediction function once'
    text = text.replace(marker, '    ++' + counter + ';\n' + marker)
    return ('#include <stdint.h>\nstatic uint64_t ' + counter + ';\n' + text +
            '\nuint64_t ' + counter + '_take(void) {\n'
            '    uint64_t result=' + counter + '; ' + counter + '=0; return result;\n}\n')


@pytest.fixture(scope='module')
def zero_mix_comparison(tmp_path_factory):
    build_dir = tmp_path_factory.mktemp('spatial_zero_mix_exact')
    actual = build_dir / 'actual.c'
    reference = build_dir / 'reference.c'
    actual.write_text(_counted_source(CODEC / 'dual_end_spatial.c', 'actual_predictions'), encoding='utf-8')
    reference.write_text(
        '#define dual_end_spatial_reset reference_reset\n'
        '#define dual_end_spatial_process reference_process\n' +
        _counted_source(ROOT / 'tests/reference/dual_end_spatial_0511.c', 'reference_predictions'),
        encoding='utf-8')
    executable, env = _build_host_executable(
        [ROOT / 'tests/test_spatial_zero_mix.c', reference, actual, CODEC / 'dual_end_wiener.c', CODEC / 'dual_end_mvdr.c'], build_dir,
        extra_includes=(CODEC / 'include',))
    return executable, env, build_dir


@pytest.mark.parametrize('scenario', [
    'quiet', 'transitions', 'fade', 'nonfinite', 'finite_overflow', 'modes',
])
def test_exact_0511_pcm_state_and_prediction_work(zero_mix_comparison, scenario):
    executable, env, build_dir = zero_mix_comparison
    result = _run([str(executable), scenario], cwd=build_dir, env=env)
    assert 'PASS:' in result.stdout

