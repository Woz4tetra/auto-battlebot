"""check_sysid_recordings thresholds track the code that consumes the same recordings."""

from __future__ import annotations

import re
from pathlib import Path

from auto_battlebot.perception.sysid_windows import WindowOptions
from playground.calibration import check_sysid_recordings as check

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_tag_coverage_matches_smoother() -> None:
    assert check.TAG_COVERAGE_PASS == WindowOptions().coverage_min


def test_telemetry_hz_matches_firmware() -> None:
    header = (REPO_ROOT / "firmware/mr_stabs_mk2/include/espnow_link.h").read_text()
    period_ms = int(re.search(r"TELEMETRY_PERIOD_MS\s*=\s*(\d+)", header).group(1))
    assert check.TELEMETRY_HZ == 1000.0 / period_ms


def _mcap_bytes() -> bytes:
    import io

    from mcap.writer import Writer

    buffer = io.BytesIO()
    writer = Writer(buffer)
    writer.start()
    schema = writer.register_schema(name="s", encoding="jsonschema", data=b"{}")
    channel = writer.register_channel(topic="/t", message_encoding="json", schema_id=schema)
    for i in range(200):
        writer.add_message(channel, i, b"{}" * 50, i)
    writer.finish()
    return buffer.getvalue()


def _torn_variants() -> list[bytes]:
    import random

    data = _mcap_bytes()
    variants = [data[:n] for n in range(0, len(data), 97)]
    rng = random.Random(0)
    for _ in range(300):
        corrupt = bytearray(data)
        for _ in range(5):
            corrupt[rng.randrange(len(corrupt))] = rng.randrange(256)
        variants.append(bytes(corrupt))
    return variants


def test_torn_files_fail_the_check_instead_of_crashing(tmp_path: Path) -> None:
    raised: set[type] = set()
    from mcap.exceptions import McapError
    from mcap.reader import make_reader

    for index, variant in enumerate(_torn_variants()):
        path = tmp_path / f"{index}.mcap"
        path.write_bytes(variant)
        try:
            with open(path, "rb") as handle:
                make_reader(handle).get_summary()
        except Exception as exc:
            raised.add(type(exc))
            report = check.Report(path)
            assert check.check_file(path, report) is None
            assert any(c.level is check.Level.FAIL for c in report.checks)
    assert len(raised) >= 4
    assert not all(issubclass(kind, McapError) for kind in raised)
