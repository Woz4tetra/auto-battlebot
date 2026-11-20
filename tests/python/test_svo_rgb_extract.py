"""training/svo_rgb_extract.py: the ffmpeg command that decodes a raw side-by-side GDR stream."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def extractor(load_training):
    return load_training("training/svo_rgb_extract.py")


def _flag_value(command: list[str], flag: str) -> str:
    return command[command.index(flag) + 1]


def test_command_pins_constant_frame_rate(extractor, tmp_path: Path) -> None:
    command = extractor.ffmpeg_command(29.97, 0, tmp_path / "o.mp4", 23, "medium")
    assert "-err_detect" not in command
    assert _flag_value(command, "-r") == "29.970000"
    assert _flag_value(command, "-vsync") == "cfr"
    assert command.index("-r") < command.index("-i")
    assert _flag_value(command, "-f") == "h264"


def test_each_eye_crops_its_own_half(extractor, tmp_path: Path) -> None:
    left = extractor.ffmpeg_command(30.0, 0, tmp_path / "l.mp4", 23, "fast")
    right = extractor.ffmpeg_command(30.0, 1, tmp_path / "r.mp4", 23, "fast")
    assert _flag_value(left, "-vf") == "crop=iw/2:ih:0:0"
    assert _flag_value(right, "-vf") == "crop=iw/2:ih:iw/2:0"


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="needs ffmpeg")
def test_intra_refresh_stream_decodes_every_frame(extractor, tmp_path: Path) -> None:
    stream = tmp_path / "gdr.h264"
    subprocess.run(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "lavfi",
            "-i",
            "testsrc=size=128x64:rate=10",
            "-frames:v",
            "30",
            "-c:v",
            "libx264",
            "-x264-params",
            "intra-refresh=1:keyint=10",
            "-f",
            "h264",
            str(stream),
        ],
        check=True,
    )
    output = tmp_path / "right.mp4"
    command = extractor.ffmpeg_command(10.0, 1, output, 23, "ultrafast")
    subprocess.run(command, input=stream.read_bytes(), check=True)
    probe = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-count_frames",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=nb_read_frames,width,height",
            "-of",
            "csv=p=0",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert probe.stdout.strip() == "64,64,30"
