import subprocess
from pathlib import Path

import av
import pytest

from auto_battlebot.recording import video_import

FPS = 30.0
FRAMES = 75


@pytest.fixture(scope="module")
def clip(tmp_path_factory: pytest.TempPathFactory) -> Path:
    path = tmp_path_factory.mktemp("clip") / "clip.mp4"
    subprocess.run(
        [
            "ffmpeg",
            "-loglevel",
            "error",
            "-f",
            "lavfi",
            "-i",
            f"testsrc=size=64x48:rate={FPS}",
            "-frames:v",
            str(FRAMES),
            "-c:v",
            "libx264",
            "-bf",
            "2",
            "-g",
            "10",
            str(path),
        ],
        check=True,
    )
    return path


def test_reencoded_stream_has_no_bframes_and_fixed_gop(clip: Path) -> None:
    units = list(video_import.iter_access_units(clip, FPS, preset="ultrafast"))
    assert len(units) == FRAMES
    codec = av.CodecContext.create("h264", "r")
    frames = [frame for unit in units for frame in codec.decode(av.packet.Packet(unit))]
    frames += codec.decode(None)
    assert len(frames) == FRAMES
    assert all(frame.pict_type != av.video.frame.PictureType.B for frame in frames)
    keyframes = [i for i, frame in enumerate(frames) if frame.key_frame]
    assert keyframes == list(range(0, FRAMES, video_import.CAMERA_GOP_FRAMES))


def test_next_stamp_leaves_gap() -> None:
    previous = 5_000_000_000
    clip_end = previous + int(FRAMES / FPS * 1e9)
    assert video_import.next_start_stamp_ns(previous, FRAMES, FPS) - clip_end == 10_000_000_000
