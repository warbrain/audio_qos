"""Source-level guards for the protocol-2 secure pairing integration."""
from pathlib import Path
import re

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("role", ["scepter", "dongle"])
def test_espnow_peer_enables_ccmp_and_uses_transactional_store(role):
    source = (ROOT / role / "components/dual_end_espnow_link/dual_end_espnow_link.c").read_text(encoding="utf-8-sig")
    for required in (
        '#include "dual_end_secure_pairing.h"',
        "peer.encrypt = encrypted;",
        "dual_end_pairing_store_save_pending",
        "dual_end_pairing_store_save_active",
        "dual_end_pairing_store_erase_pending",
        "dual_end_pairing_records_equal",
        "restore_pairing_previous_locked",
    ):
        assert required in source


@pytest.mark.parametrize("role", ["scepter", "dongle"])
def test_usb_pairing_surface_exposes_full_transaction(role):
    source = (ROOT / role / "components/dual_end_pairing/dual_end_pairing.c").read_text(encoding="utf-8-sig")
    for command in ("PAIR PREPARE", "PAIR COMMIT", "PAIR CONFIRM", "PAIR ROLLBACK", "PAIR CLEAR"):
        assert command in source
    assert "pair_protocol=%u" in source
    assert "encryption=%s" in source


@pytest.mark.parametrize("role", ["scepter", "dongle"])
def test_application_protocol_version_matches_0_4_1_bootstrap(role):
    header = (ROOT / role / "components/dual_end_common/include/dual_end_link_config.h").read_text(encoding="utf-8-sig")
    match = re.search(r"#define\s+SCEPTER_PROTOCOL_VERSION\s+(\d+)", header)
    assert match and match.group(1) == "1"


def test_shared_store_component_is_built_by_both_targets():
    for role in ("scepter", "dongle"):
        cmake = (ROOT / role / "CMakeLists.txt").read_text(encoding="utf-8-sig")
        assert 'EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components"' in cmake
    assert (ROOT / "components/dual_end_secure_pairing/dual_end_secure_pairing.c").is_file()
