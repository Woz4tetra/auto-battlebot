"""Build the engines the playback clips need on this box, cached by source hash and TensorRT."""

from __future__ import annotations

import argparse
import hashlib
import os
import platform
import subprocess
import sys
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from auto_battlebot.config_profile import load_profile_toml
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.verify.playback import TIERS, gpu_tag, load_clips, tensorrt_tag

DEFAULT_CACHE = Path.home() / "ci-data" / "engines"
YOLO_CLI = "training/yolo/convert_to_tensorrt.py"
DEEPLAB_CLI = "training/deeplab/convert_to_tensorrt.py"
HASH_CHUNK = 1 << 20


@dataclass(frozen=True)
class EngineTarget:
    """One engine a profile lists for this box, and the files it is built from."""

    engine: Path
    sources: tuple[Path, ...]
    cli: str

    @property
    def stem(self) -> str:
        return self.sources[0].stem


def platform_tag() -> str:
    return f"{platform.machine()}_{gpu_tag()}"


def engine_candidates(profile: dict[str, Any]) -> Iterator[str]:
    """Every `engine.candidates` entry anywhere in a resolved profile."""
    for key, value in profile.items():
        if not isinstance(value, dict):
            continue
        if key == "engine" and isinstance(value.get("candidates"), list):
            yield from value["candidates"]
        else:
            yield from engine_candidates(value)


def source_for(engine: Path, tag: str, root: Path) -> EngineTarget | None:
    """The DeepLab checkpoint or YOLO ONNX an engine name comes from, if it is on disk."""
    suffix = f"_{tag}.engine"
    if not engine.name.endswith(suffix):
        return None
    base = root / engine.parent / engine.name.removesuffix(suffix)
    checkpoint, metadata, onnx = (base.with_suffix(s) for s in (".pth", ".toml", ".onnx"))
    if checkpoint.exists() and metadata.exists():
        return EngineTarget(root / engine, (checkpoint, metadata), DEEPLAB_CLI)
    if onnx.exists():
        return EngineTarget(root / engine, (onnx,), YOLO_CLI)
    return None


def required_engines(tag: str, root: Path = REPO_ROOT) -> tuple[list[EngineTarget], list[str]]:
    """Engines the playback clips' profiles list for `tag`, and the ones with no source."""
    names: dict[str, None] = {}
    for clip in load_clips():
        for tier in TIERS:
            for candidate in engine_candidates(load_profile_toml(clip.profile(tier))):
                if candidate.endswith(f"_{tag}.engine"):
                    names[candidate] = None
    found, missing = [], []
    for name in names:
        target = source_for(Path(name), tag, root)
        if target is None:
            missing.append(name)
        else:
            found.append(target)
    return found, missing


def sha256_of(paths: tuple[Path, ...]) -> str:
    digest = hashlib.sha256()
    for path in paths:
        with path.open("rb") as handle:
            while chunk := handle.read(HASH_CHUNK):
                digest.update(chunk)
    return digest.hexdigest()


def cache_dir(target: EngineTarget, cache: Path, trt: str) -> Path:
    return cache / f"{sha256_of(target.sources)[:16]}_{trt}"


def timing_cache(cache: Path, trt: str) -> Path:
    """Tactic timings shared by every build on this box, one file per TensorRT version."""
    return cache / f"timing_{trt}.cache"


def build_command(target: EngineTarget, out_dir: Path, root: Path, timing: Path) -> list[str]:
    return [
        sys.executable,
        str(root / target.cli),
        str(target.sources[0]),
        "-o",
        str(out_dir / f"{target.stem}.engine"),
        "--timing-cache",
        str(timing),
    ]


def link(engine: Path, cached: Path) -> None:
    """Point the name the config expects at the cached engine, replacing what was there."""
    if engine.is_symlink() or engine.exists():
        if engine.is_symlink() and engine.resolve() == cached.resolve():
            return
        engine.unlink()
    os.symlink(cached, engine)


def build(cache: Path, root: Path = REPO_ROOT) -> int:
    tag, trt = platform_tag(), tensorrt_tag()
    targets, missing = required_engines(tag, root)
    for name in missing:
        print(f"no source for {name}: sync the ONNX or checkpoint into data/models", flush=True)
    for target in targets:
        out_dir = cache_dir(target, cache, trt)
        cached = out_dir / target.engine.name
        if cached.exists():
            print(f"hit  {target.engine.name} ({out_dir.name})", flush=True)
        else:
            print(f"miss {target.engine.name}: building with {trt}", flush=True)
            out_dir.mkdir(parents=True, exist_ok=True)
            command = build_command(target, out_dir, root, timing_cache(cache, trt))
            code = subprocess.run(command, cwd=root).returncode
            if code != 0 or not cached.exists():
                print(f"build failed for {target.engine.name}", flush=True)
                return 1
        link(target.engine, cached)
    return 1 if missing else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("build", "list"))
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--tag", help="Platform tag to list for (default: this box)")
    args = parser.parse_args(argv)
    if args.action == "build":
        return build(args.cache)
    targets, missing = required_engines(args.tag or platform_tag())
    for target in targets:
        print(f"{target.engine.relative_to(REPO_ROOT)} <- {target.cli}")
        for source in target.sources:
            print(f"    {source.relative_to(REPO_ROOT)}")
    for name in missing:
        print(f"{name} <- no source on disk")
    return 0


if __name__ == "__main__":
    sys.exit(main())
