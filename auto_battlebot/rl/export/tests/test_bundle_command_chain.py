import re

import numpy as np

from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.rl.export.bundle import (
    ESPNOW_PROTOCOL_VERSION,
    MK2_ESPNOW_COMMAND_CHAIN,
    BundleSpec,
    write_bundle,
)

PROTOCOL_HEADER = REPO_ROOT / "firmware/shared/espnow_protocol/src/espnow_protocol.h"
CHAIN_SOURCE = REPO_ROOT / "src/health/command_chain.cpp"


def tiny_spec(chain: dict[str, str] | None = None) -> BundleSpec:
    obs = np.zeros((1, 3), dtype=np.float32)
    act = np.zeros((1, 2), dtype=np.float32)
    spec = BundleSpec(
        run_id="chain_test",
        obs_spec_version=1,
        obs_dim=3,
        action_dim=2,
        model={},
        selftest_obs=obs,
        selftest_actions=act,
    )
    if chain is not None:
        spec.command_chain = chain
    return spec


def test_exporter_speaks_the_firmware_protocol_version() -> None:
    match = re.search(r"PROTOCOL_VERSION\s*=\s*(\d+);", PROTOCOL_HEADER.read_text())
    assert match is not None
    assert int(match.group(1)) == ESPNOW_PROTOCOL_VERSION


def test_manifest_carries_the_command_chain_the_policy_was_trained_for(tmp_path) -> None:
    out = write_bundle(tiny_spec(), tmp_path / "bundle", REPO_ROOT)
    manifest = tomllib.loads((out / "manifest.toml").read_text())
    assert manifest["command_chain"] == MK2_ESPNOW_COMMAND_CHAIN
    assert manifest["command_chain"]["max_motor_rpm_per_s"] == "0"


def test_a_custom_chain_is_written_as_given(tmp_path) -> None:
    chain = {"transmitter": "OpenTxTransmitter", "max_motor_rpm_per_s": "2000"}
    out = write_bundle(tiny_spec(chain), tmp_path / "bundle", REPO_ROOT)
    assert tomllib.loads((out / "manifest.toml").read_text())["command_chain"] == chain


def test_chain_keys_are_the_keys_the_app_resolves() -> None:
    source = CHAIN_SOURCE.read_text()
    for key in MK2_ESPNOW_COMMAND_CHAIN:
        assert f'{{"{key}",' in source, key
