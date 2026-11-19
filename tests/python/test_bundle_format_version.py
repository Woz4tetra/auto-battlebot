import re
from pathlib import Path

from auto_battlebot.rl.export.bundle import FORMAT_VERSION

HEADER = Path(__file__).resolve().parents[2] / "include" / "rl_policy" / "policy_bundle.hpp"


def test_cpp_loader_reads_the_format_version_the_exporter_writes():
    match = re.search(r"kPolicyBundleFormatVersion\s*=\s*(\d+);", HEADER.read_text())
    assert match is not None
    assert int(match.group(1)) == FORMAT_VERSION
