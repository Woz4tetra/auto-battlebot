"""Manifest of the recordings and model sources CI needs, checked by sha256 at job start."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.verify.ci_engines import required_engines
from auto_battlebot.verify.playback import load_clips

MANIFEST = Path(__file__).with_name("ci_clips.json")
HASH_CACHE = Path.home() / ".cache" / "auto_battlebot" / "ci_clips_hashes.json"
JETSON_TAG = "aarch64_sm87"
CI_DATA = "/home/ci/ci-data"
HASH_CHUNK = 1 << 20


def needed_files(root: Path = REPO_ROOT) -> list[str]:
    """Repo-relative paths of every clip recording and the Jetson engines' sources."""
    paths = {str(clip.svo_path.relative_to(root)) for clip in load_clips()}
    targets, missing = required_engines(JETSON_TAG, root)
    if missing:
        raise FileNotFoundError(f"no source on disk for {', '.join(missing)}")
    for target in targets:
        paths.update(str(source.relative_to(root)) for source in target.sources)
    return sorted(paths)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(HASH_CHUNK):
            digest.update(chunk)
    return digest.hexdigest()


class HashCache:
    """sha256 by resolved path, reused while size and mtime stay the same."""

    def __init__(self, path: Path) -> None:
        self.path = path
        try:
            self.entries: dict[str, list] = json.loads(path.read_text())
        except (OSError, ValueError):
            self.entries = {}

    def sha256(self, file: Path) -> str:
        stat = file.stat()
        key = str(file.resolve())
        entry = self.entries.get(key)
        if entry and entry[0] == stat.st_size and entry[1] == stat.st_mtime_ns:
            return str(entry[2])
        digest = sha256_file(file)
        self.entries[key] = [stat.st_size, stat.st_mtime_ns, digest]
        return digest

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.entries, indent=1, sort_keys=True))


def write(root: Path, manifest: Path, cache: HashCache) -> int:
    entries = {}
    for rel in needed_files(root):
        file = root / rel
        entries[rel] = {"bytes": file.stat().st_size, "sha256": cache.sha256(file)}
    manifest.write_text(json.dumps(entries, indent=2, sort_keys=True) + "\n")
    cache.save()
    print(f"wrote {len(entries)} entries to {manifest}")
    return 0


def problems(root: Path, manifest: Path, cache: HashCache) -> list[str]:
    """Every manifest entry that is missing or differs on this box."""
    found = []
    for rel, entry in json.loads(manifest.read_text()).items():
        file = root / rel
        if not file.exists():
            found.append(f"missing {rel}")
        elif file.stat().st_size != entry["bytes"]:
            found.append(f"size differs {rel}")
        elif cache.sha256(file) != entry["sha256"]:
            found.append(f"sha256 differs {rel}")
    cache.save()
    return found


def check(root: Path, manifest: Path, cache: HashCache) -> int:
    found = problems(root, manifest, cache)
    for problem in found:
        print(problem)
    if found:
        print("run `python -m auto_battlebot.verify.ci_clips sync <host>` from the dev box")
        return 1
    print(f"ci clips: {len(json.loads(manifest.read_text()))} files match the manifest")
    return 0


def sync_commands(host: str, manifest: Path) -> list[list[str]]:
    """One rsync per manifest file into the `ci` user's data dir, written as `ci`."""
    commands = []
    for rel in json.loads(manifest.read_text()):
        destination = f"{host}:{CI_DATA}/{Path(rel).relative_to('data')}"
        commands.append(
            [
                "rsync",
                "-a",
                "--mkpath",
                "--info=progress2",
                "--rsync-path=sudo -u ci rsync",
                str(REPO_ROOT / rel),
                destination,
            ]
        )
    return commands


def sync(host: str, manifest: Path) -> int:
    for command in sync_commands(host, manifest):
        print(" ".join(command[-2:]), flush=True)
        if subprocess.run(command).returncode != 0:
            return 1
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("write", "check", "sync"))
    parser.add_argument("host", nargs="?", help="sync target, e.g. ben@auto-battlebot-compute-1")
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    args = parser.parse_args(argv)
    cache = HashCache(HASH_CACHE)
    if args.action == "write":
        return write(REPO_ROOT, args.manifest, cache)
    if args.action == "check":
        return check(REPO_ROOT, args.manifest, cache)
    if not args.host:
        parser.error("sync needs a host")
    return sync(args.host, args.manifest)


if __name__ == "__main__":
    sys.exit(main())
