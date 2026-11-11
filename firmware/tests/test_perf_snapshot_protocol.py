"""Contract tests for the additive immutable PERF_SNAPSHOT reader.

The test compiles the production parser against small host stubs.  It proves
that a 13-part transaction uses one cached window even if the live PERF cache
changes between parts, while the legacy PERF path remains live and unchanged.
"""

from pathlib import Path

from test_audio_recovery_042 import _build_host_executable, _run
from test_snapshot_consistency import runtime_prefix


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "components" / "dual_end_runtime"


def test_perf_snapshot_lease_is_immutable_and_legacy_perf_stays_live(tmp_path: Path):
    source = (RUNTIME / "dual_end_runtime.c").read_text(encoding="utf-8")
    harness = tmp_path / "perf_snapshot_contract.c"
    harness.write_text(runtime_prefix(source) + r'''
int main(void) {
    char reply[256];
    s_perf_ready[1] = true;
    s_perf_cache[1].window_seq = 101;
    assert(dual_end_runtime_command("PERF_SNAPSHOT 77 1 0", reply, sizeof(reply)));
    assert(strstr(reply, "OK PERF_SNAPSHOT token=77") && strstr(reply, "\"window\":101"));
    s_perf_cache[1].window_seq = 202;
    assert(dual_end_runtime_command("PERF_SNAPSHOT 77 1 12", reply, sizeof(reply)));
    assert(strstr(reply, "\"window\":101"));
    assert(dual_end_runtime_command("PERF_SNAPSHOT 77 2 1", reply, sizeof(reply)));
    assert(strstr(reply, "snapshot_lease_mismatch"));
    assert(dual_end_runtime_command("PERF_SNAPSHOT 78 1 1", reply, sizeof(reply)));
    assert(strstr(reply, "snapshot_lease_mismatch"));
    assert(dual_end_runtime_command("PERF_SNAPSHOT 77 1 0", reply, sizeof(reply)));
    assert(strstr(reply, "\"window\":101")); /* idempotent part zero */
    assert(dual_end_runtime_command("PERF 88 1 0", reply, sizeof(reply)));
    assert(strstr(reply, "OK PERF token=88") && strstr(reply, "\"window\":202"));
    assert(dual_end_runtime_command("PERF_SNAPSHOT 99 1 13", reply, sizeof(reply)));
    assert(strcmp(reply, "ERR PERF_SNAPSHOT arguments\n") == 0);
    puts("PERF_SNAPSHOT immutable lease and legacy PERF compatibility: PASS");
    return 0;
}
''', encoding="utf-8")
    executable, env = _build_host_executable(
        [harness], tmp_path, extra_includes=(RUNTIME / "include",))
    result = _run([str(executable)], cwd=tmp_path, env=env)
    assert "immutable lease and legacy PERF compatibility: PASS" in result.stdout
