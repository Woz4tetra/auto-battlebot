import io
import subprocess
from pathlib import Path

import cv2
import numpy as np
import pytest
from mcap.writer import Writer

from auto_battlebot.recording import svo2, video_import

FRAMES = 12
TOPIC = "zed/side_by_side"


def _encode_clip(path: Path) -> None:
    subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-f", "lavfi", "-i", "testsrc=size=128x48:rate=30"]
        + ["-frames:v", str(FRAMES), "-c:v", "libx264", str(path)],
        check=True,
    )


@pytest.fixture(scope="module")
def svo(tmp_path_factory: pytest.TempPathFactory) -> Path:
    directory = tmp_path_factory.mktemp("svo")
    clip = directory / "clip.mp4"
    _encode_clip(clip)
    path = directory / "test.svo2"
    with open(path, "wb") as handle:
        writer = Writer(handle)
        writer.start()
        schema = writer.register_schema(name="raw", encoding="", data=b"")
        channel = writer.register_channel(topic=TOPIC, message_encoding="raw", schema_id=schema)
        units = video_import.iter_access_units(clip, 30.0, preset="ultrafast")
        for i, unit in enumerate(units):
            writer.add_message(
                channel, log_time=i * 33_000_000, data=bytes(8) + unit, publish_time=0
            )
        writer.finish()
    return path


def test_left_jpegs_one_per_message(svo: Path) -> None:
    jpegs = list(svo2.iter_left_jpegs(svo, TOPIC, 30.0))
    assert len(jpegs) == FRAMES
    first = cv2.imdecode(np.frombuffer(jpegs[0], dtype=np.uint8), cv2.IMREAD_COLOR)
    assert first.shape[:2] == (48, 64)


def _jpeg(seed: int) -> bytes:
    rng = np.random.default_rng(seed)
    ok, encoded = cv2.imencode(".jpg", rng.integers(0, 255, (32, 48, 3), dtype=np.uint8))
    assert ok
    return encoded.tobytes()


def test_iter_jpegs_splits_on_soi_only() -> None:
    images = [_jpeg(seed) for seed in range(3)]
    stream = io.BytesIO(b"".join(images))
    assert list(svo2._iter_jpegs(stream)) == images


def test_iter_jpegs_survives_tiny_reads(monkeypatch: pytest.MonkeyPatch) -> None:
    images = [_jpeg(seed) for seed in range(3)]
    monkeypatch.setattr(svo2, "_READ_CHUNK", 7)
    assert list(svo2._iter_jpegs(io.BytesIO(b"".join(images)))) == images
