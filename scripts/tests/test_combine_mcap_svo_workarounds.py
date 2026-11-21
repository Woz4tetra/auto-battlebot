import importlib.util
import json
from pathlib import Path

from mcap.exceptions import McapError
from mcap.reader import StreamReader
from mcap.writer import CompressionType, Writer

from auto_battlebot.paths import REPO_ROOT

SCRIPT = REPO_ROOT / "scripts" / "combine_mcap_svo.py"
SPEC = importlib.util.spec_from_file_location("combine_mcap_svo_workarounds", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
combine = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(combine)


def write_mcap(path: Path, topic: str, payloads: list[bytes]) -> None:
    with open(path, "wb") as f:
        writer = Writer(f, compression=CompressionType.ZSTD)
        writer.start()
        schema = writer.register_schema(name="s", encoding="jsonschema", data=b"{}")
        channel = writer.register_channel(topic=topic, message_encoding="json", schema_id=schema)
        for i, payload in enumerate(payloads):
            writer.add_message(channel, i, payload, i)
        writer.finish()


def corrupt_first_chunk(path: Path) -> None:
    raw = bytearray(path.read_bytes())
    for i in range(100, 400):
        raw[i] ^= 0xFF
    path.write_bytes(bytes(raw))


def test_corrupt_compressed_chunk_raises_a_non_mcap_error_that_the_damage_probe_reports(
    tmp_path: Path,
) -> None:
    path = tmp_path / "damaged.mcap"
    write_mcap(path, "/t", [b'{"a": 1}' * 20] * 50)
    corrupt_first_chunk(path)
    raised: Exception | None = None
    try:
        with open(path, "rb") as f:
            for _record in StreamReader(f).records:
                pass
    except Exception as exc:
        raised = exc
    assert raised is not None
    assert not isinstance(raised, McapError)
    description = combine._describe_damage(path)
    assert description is not None
    assert description.startswith(type(raised).__name__)


def test_clean_recording_has_no_damage_description(tmp_path: Path) -> None:
    path = tmp_path / "clean.mcap"
    write_mcap(path, "/t", [b'{"a": 1}'] * 5)
    assert combine._describe_damage(path) is None


def test_svo_path_is_read_from_raw_frame_meta_json_of_old_recordings(tmp_path: Path) -> None:
    path = tmp_path / "old.mcap"
    meta = json.dumps({"svo_path": "/data/svo/run1.svo2", "frame": 1}).encode()
    write_mcap(path, combine.FRAME_META_TOPIC, [meta, meta, b'{"frame": 2}'])
    assert combine.extract_svo_paths(path) == [Path("/data/svo/run1.svo2")]
