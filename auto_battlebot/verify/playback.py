"""Playback regression tiers: lockstep clips hashed against a baseline, real-time clips scored."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import re
import shutil
import subprocess
import sys
import time
from collections import defaultdict
from collections.abc import Iterable, Iterator
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.recording import mcap_io
from auto_battlebot.recording.recording_catalog import load_catalog

CLIPS_FILE = Path(__file__).with_name("playback_clips.toml")
BASELINE_FILE = Path(__file__).with_name("playback_baselines.json")
RUNS_DIR = REPO_ROOT / "data" / "verify" / "playback"
BINARY = REPO_ROOT / "build" / "auto_battlebot"
TIERS = ("lockstep", "realtime")
PROFILE_SUFFIX = {"lockstep": "lockstep", "realtime": "playback"}
CAMERA_FPS = 30.0
RUNS_KEPT = 5
CLIP_TIMEOUT_S = 900.0
AA_RUNS = 5

WALL_TIME_TOPICS = frozenset({"/log", "/diagnostics/health", "/diagnostics/foxglove_publisher"})
WALL_TIME_PREFIXES = ("/status/",)
WALL_TIME_ENTRIES = frozenset({"relay_publish_link"})
WALL_TIME_VALUES = frozenset({"rate", "uptime_s", "viz_dropped"})
WALL_TIME_VALUE_SUFFIX = "elapsed_ms"

JUMP_M = 0.3
MOVING_THRESHOLD = 0.05
RED_LEVEL = 2
LIVENESS_IGNORED = {"lockstep": frozenset({"relay_publish_link"}), "realtime": frozenset()}

CROSS_CLASS_TOLERANCE = {
    "opponent_coverage": 0.05,
    "our_coverage": 0.05,
    "opponent_jumps_per_min": 2.0,
    "our_jumps_per_min": 2.0,
    "linear_mean_abs": 0.05,
    "angular_mean_abs": 0.05,
    "moving_fraction": 0.05,
}
REALTIME_METRICS = ("latency_p95_ms", "frame_drop_rate")
REALTIME_FLOOR = {"latency_p95_ms": 5.0, "frame_drop_rate": 0.02}


@dataclass(frozen=True)
class Clip:
    """One recording segment the playback tiers replay."""

    name: str
    recording: str
    robot: str
    frames: int
    start_frame: int
    svo_path: Path

    def profile(self, tier: str) -> str:
        return f"playback/{self.robot}_{PROFILE_SUFFIX[tier]}"


def load_clips(path: Path = CLIPS_FILE, catalog_path: Path | None = None) -> list[Clip]:
    """Clips from the clip table, with paths and start frames from the recording catalog."""
    catalog = load_catalog(catalog_path) if catalog_path else load_catalog()
    clips = []
    for name, entry in tomllib.loads(path.read_text()).get("clips", {}).items():
        recording = catalog.get(entry["recording"])
        if recording is None:
            raise ValueError(
                f"{path}: clip {name!r} names unknown recording {entry['recording']!r}"
            )
        svo_path = recording.path if recording.path.is_absolute() else REPO_ROOT / recording.path
        clips.append(
            Clip(
                name=name,
                recording=entry["recording"],
                robot=entry["robot"],
                frames=int(entry["frames"]),
                start_frame=int(entry.get("start_frame", recording.start_frame)),
                svo_path=svo_path,
            )
        )
    return clips


def machine_class() -> str:
    """Architecture, GPU compute capability, TensorRT and ZED SDK versions."""
    return "_".join([platform.machine(), gpu_tag(), tensorrt_tag(), zed_tag()])


def gpu_tag() -> str:
    if shutil.which("nvidia-smi"):
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=compute_cap", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.split()
        if out:
            return "sm" + out[0].replace(".", "")
    compatible = Path("/proc/device-tree/compatible")
    if compatible.exists() and b"tegra234" in compatible.read_bytes():
        return "sm87"
    return "smunknown"


def tensorrt_tag() -> str:
    for lib_dir in (Path("/usr/lib/x86_64-linux-gnu"), Path("/usr/lib/aarch64-linux-gnu")):
        link = lib_dir / "libnvinfer.so.10"
        if link.exists():
            match = re.search(r"\.so\.(\d+)\.(\d+)", link.resolve().name)
            if match:
                return f"trt{match.group(1)}.{match.group(2)}"
    return "trtunknown"


def zed_tag() -> str:
    header = Path("/usr/local/zed/include/sl/Camera.hpp")
    if not header.exists():
        return "zedunknown"
    text = header.read_text(errors="replace")
    major = re.search(r"#define ZED_SDK_MAJOR_VERSION (\d+)", text)
    minor = re.search(r"#define ZED_SDK_MINOR_VERSION (\d+)", text)
    if not major or not minor:
        return "zedunknown"
    return f"zed{major.group(1)}.{minor.group(1)}"


def clip_profile_text(clip: Clip, tier: str, out_dir: Path) -> str:
    return "\n".join(
        [
            f'extends = "{clip.profile(tier)}"',
            "",
            "[rgbd_camera]",
            f"svo_file_path = {json.dumps(str(clip.svo_path))}",
            f"svo_start_frame = {clip.start_frame}",
            f"svo_max_frames = {clip.frames}",
            "",
            "[mcap]",
            "enable = true",
            f"recordings_dir = {json.dumps(str(out_dir))}",
            "",
        ]
    )


def run_clip(clip: Clip, tier: str, out_dir: Path, binary: Path = BINARY) -> Path:
    """Replay one clip through the app and return its output MCAP."""
    out_dir.mkdir(parents=True, exist_ok=True)
    profile = out_dir / "clip.toml"
    profile.write_text(clip_profile_text(clip, tier, out_dir))
    with (out_dir / "app.log").open("w") as log:
        code = subprocess.run(
            [str(binary), "-c", str(profile)],
            cwd=REPO_ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            timeout=CLIP_TIMEOUT_S,
            check=False,
        ).returncode
    mcaps = sorted(out_dir.glob("*.mcap"))
    if code != 0 or len(mcaps) != 1:
        raise RuntimeError(
            f"{clip.name}: app exited {code} with {len(mcaps)} MCAPs; see {out_dir / 'app.log'}"
        )
    return mcaps[0]


def canonical_diagnostics(data: bytes) -> bytes:
    """Diagnostics JSON with every wall-time field removed."""
    entries = json.loads(data)
    kept = {}
    for name, entry in sorted(entries.items()):
        if name in WALL_TIME_ENTRIES or entry.get("held"):
            continue
        values = {
            key: value
            for key, value in entry.get("values", {}).items()
            if key not in WALL_TIME_VALUES and not key.endswith(WALL_TIME_VALUE_SUFFIX)
        }
        kept[name] = {
            "level": entry.get("level"),
            "message": entry.get("message"),
            "values": values,
        }
    return json.dumps(kept, sort_keys=True).encode()


def iter_messages(mcap_path: Path) -> Iterator[tuple[str, bytes]]:
    for topic, _, data in mcap_io.iter_messages(mcap_path):
        yield topic, data


def topic_hashes(messages: Iterable[tuple[str, bytes]]) -> dict[str, str]:
    """SHA-256 of each deterministic topic's payloads in log order."""
    hashes: dict[str, Any] = defaultdict(hashlib.sha256)
    for topic, data in messages:
        if topic in WALL_TIME_TOPICS or topic.startswith(WALL_TIME_PREFIXES):
            continue
        payload = canonical_diagnostics(data) if topic.startswith("/diagnostics/") else data
        digest = hashes[topic]
        digest.update(len(payload).to_bytes(8, "little"))
        digest.update(payload)
    return {topic: digest.hexdigest() for topic, digest in sorted(hashes.items())}


@dataclass
class Tracks:
    """Per-frame robot positions keyed by frame id, from /robot_markers."""

    frames: list[dict[str, tuple[float, float]]] = field(default_factory=list)


def decode_robot_frame(data: bytes) -> dict[str, tuple[float, float]]:
    names: dict[str, str] = {}
    positions: dict[str, tuple[float, float]] = {}
    for entity in mcap_io.decode_scene_update(data).entities:
        kind, _, index = entity.id.partition("/")
        if kind == "robot_labels" and entity.texts:
            names[index] = entity.texts[0].text.split(" ", 1)[0]
        elif kind == "robot_bounds" and entity.cubes:
            x, y, _ = entity.cubes[0].pose.position
            positions[index] = (x, y)
    return {names[i]: positions[i] for i in positions if i in names}


def coverage(tracks: Tracks, prefix: str) -> float:
    if not tracks.frames:
        return 0.0
    hits = sum(any(name.startswith(prefix) for name in frame) for frame in tracks.frames)
    return hits / len(tracks.frames)


def jumps_per_min(tracks: Tracks, prefix: str, frames: int) -> float:
    jumps = 0
    for before, after in zip(tracks.frames, tracks.frames[1:]):
        for name, (x, y) in after.items():
            if name.startswith(prefix) and name in before:
                bx, by = before[name]
                if math.hypot(x - bx, y - by) > JUMP_M:
                    jumps += 1
    minutes = frames / CAMERA_FPS / 60.0
    return jumps / minutes if minutes > 0 else 0.0


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = q * (len(ordered) - 1)
    low = math.floor(rank)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (rank - low)


@dataclass
class Digest:
    """What one clip run produced: topic hashes, metrics, and red liveness checks."""

    hashes: dict[str, str]
    metrics: dict[str, float]
    red_checks: list[str]


@dataclass
class ClipReadings:
    """Per-message samples a clip's metrics are computed from."""

    tier: str
    tracks: Tracks = field(default_factory=Tracks)
    frame_indices: list[int] = field(default_factory=list)
    linear: list[float] = field(default_factory=list)
    angular: list[float] = field(default_factory=list)
    latencies: list[float] = field(default_factory=list)
    worst: dict[str, int] = field(default_factory=dict)

    def read(self, topic: str, data: bytes) -> None:
        reader = TOPIC_READERS.get(topic)
        if reader is not None:
            reader(self, data)

    def robot_markers(self, data: bytes) -> None:
        self.tracks.frames.append(decode_robot_frame(data))

    def frame_meta(self, data: bytes) -> None:
        index = json.loads(data).get("video_frame_index")
        if isinstance(index, int) and index >= 0:
            self.frame_indices.append(index)

    def transmitter(self, data: bytes) -> None:
        values = json.loads(data).get("process", {}).get("values", {})
        if "linear_out" in values:
            self.linear.append(float(values["linear_out"]))
            self.angular.append(float(values["angular_out"]))

    def runner(self, data: bytes) -> None:
        values = json.loads(data).get("pipeline", {}).get("values", {})
        if "latency_ms" in values:
            self.latencies.append(float(values["latency_ms"]))

    def monitor(self, data: bytes) -> None:
        for name, entry in json.loads(data).items():
            if name not in LIVENESS_IGNORED[self.tier]:
                self.worst[name] = max(self.worst.get(name, 0), int(entry.get("level", 0)))

    def metrics(self) -> dict[str, float]:
        frames = len(self.frame_indices)
        span = self.frame_indices[-1] - self.frame_indices[0] + 1 if frames else 0
        commands = list(zip(self.linear, self.angular))
        moving = sum(abs(a) > MOVING_THRESHOLD or abs(b) > MOVING_THRESHOLD for a, b in commands)
        count = len(commands)
        return {
            "frames": float(frames),
            "opponent_coverage": coverage(self.tracks, "their_robot"),
            "our_coverage": coverage(self.tracks, "our_robot"),
            "opponent_jumps_per_min": jumps_per_min(self.tracks, "their_robot", frames),
            "our_jumps_per_min": jumps_per_min(self.tracks, "our_robot", frames),
            "linear_mean_abs": sum(map(abs, self.linear)) / count if count else 0.0,
            "angular_mean_abs": sum(map(abs, self.angular)) / count if count else 0.0,
            "moving_fraction": moving / count if count else 0.0,
            "latency_p95_ms": percentile(self.latencies, 0.95),
            "frame_drop_rate": (span - frames) / span if span else 0.0,
        }

    def red_checks(self) -> list[str]:
        return sorted(name for name, level in self.worst.items() if level >= RED_LEVEL)


TOPIC_READERS = {
    "/robot_markers": ClipReadings.robot_markers,
    "/camera/frame_meta": ClipReadings.frame_meta,
    "/diagnostics/playback_transmitter": ClipReadings.transmitter,
    "/diagnostics/runner": ClipReadings.runner,
    "/diagnostics/performance_monitor": ClipReadings.monitor,
}


def summarize(messages: Iterable[tuple[str, bytes]], tier: str) -> Digest:
    """Hashes, track and command metrics, and liveness from a clip's output messages."""
    collected: list[tuple[str, bytes]] = []
    readings = ClipReadings(tier)
    for topic, data in messages:
        collected.append((topic, data))
        readings.read(topic, data)
    return Digest(topic_hashes(collected), readings.metrics(), readings.red_checks())


def digest_mcap(mcap_path: Path, tier: str) -> Digest:
    return summarize(iter_messages(mcap_path), tier)


def load_baselines(path: Path = BASELINE_FILE) -> dict[str, Any]:
    if not path.exists():
        return {tier: {} for tier in TIERS}
    data: dict[str, Any] = json.loads(path.read_text())
    for tier in TIERS:
        data.setdefault(tier, {})
    return data


def save_baselines(data: dict[str, Any], path: Path = BASELINE_FILE) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def compare_lockstep(
    clip: str, got: Digest, baselines: dict[str, Any], machine: str
) -> tuple[list[str], list[str]]:
    """Problems and notes for one lockstep clip against the stored baselines."""
    by_class = baselines.get("lockstep", {})
    own = by_class.get(machine, {}).get(clip)
    if own is not None:
        problems = []
        differing = sorted(
            topic
            for topic in set(own["hashes"]) | set(got.hashes)
            if own["hashes"].get(topic) != got.hashes.get(topic)
        )
        if differing:
            problems.append(f"{clip}: {len(differing)} topics differ: {', '.join(differing)}")
            problems += [f"{clip}:   {line}" for line in metric_deltas(own["metrics"], got.metrics)]
        if sorted(own["red_checks"]) != got.red_checks:
            problems.append(f"{clip}: red checks {got.red_checks}, baseline {own['red_checks']}")
        return problems, []
    others = {name: clips[clip] for name, clips in by_class.items() if clip in clips}
    if not others:
        return [f"{clip}: no lockstep baseline for any machine class"], []
    reference_class, reference = sorted(others.items())[0]
    notes = [f"{clip}: no {machine} baseline; metrics checked against {reference_class}"]
    problems = []
    for metric, tolerance in CROSS_CLASS_TOLERANCE.items():
        delta = got.metrics[metric] - reference["metrics"][metric]
        if abs(delta) > tolerance:
            problems.append(
                f"{clip}: {metric} {got.metrics[metric]:.4f} vs {reference['metrics'][metric]:.4f}"
                f" on {reference_class} (tolerance {tolerance})"
            )
    new_red = sorted(set(got.red_checks) - set(reference["red_checks"]))
    if new_red:
        problems.append(f"{clip}: new red checks {new_red}")
    return problems, notes


def metric_deltas(before: dict[str, float], after: dict[str, float]) -> list[str]:
    return [
        f"{name}: {before[name]:.6f} -> {after[name]:.6f}"
        for name in sorted(before)
        if name in after and before[name] != after[name]
    ]


def realtime_bounds(digests: list[Digest]) -> dict[str, Any]:
    """Per-metric min and max over A/A runs, and the red checks any run saw."""
    return {
        "metrics": {
            name: [min(d.metrics[name] for d in digests), max(d.metrics[name] for d in digests)]
            for name in REALTIME_METRICS
        },
        "red_checks": sorted({check for d in digests for check in d.red_checks}),
        "runs": len(digests),
    }


def compare_realtime(clip: str, got: Digest, baselines: dict[str, Any], machine: str) -> list[str]:
    """Problems for one real-time clip against its A/A spread on this machine class."""
    own = baselines.get("realtime", {}).get(machine, {}).get(clip)
    if own is None:
        return [f"{clip}: no realtime baseline for {machine}"]
    problems = []
    for name in REALTIME_METRICS:
        low, high = own["metrics"][name]
        margin = max(high - low, REALTIME_FLOOR[name])
        value = got.metrics[name]
        if not low - margin <= value <= high + margin:
            problems.append(
                f"{clip}: {name} {value:.4f} outside [{low - margin:.4f}, {high + margin:.4f}]"
            )
    new_red = sorted(set(got.red_checks) - set(own["red_checks"]))
    if new_red:
        problems.append(f"{clip}: new red checks {new_red}")
    return problems


def new_run_dir(tier: str, runs_dir: Path = RUNS_DIR) -> Path:
    stamp = time.strftime("%Y-%m-%dT%H-%M-%S")
    run_dir = runs_dir / f"{stamp}_{tier}"
    run_dir.mkdir(parents=True, exist_ok=False)
    prune_runs(runs_dir)
    return run_dir


def prune_runs(runs_dir: Path, keep: int = RUNS_KEPT) -> None:
    runs = sorted(path for path in runs_dir.iterdir() if path.is_dir())
    for old in runs[:-keep]:
        shutil.rmtree(old)


def selected_clips(names: list[str] | None) -> list[Clip]:
    clips = load_clips()
    if not names:
        return clips
    unknown = sorted(set(names) - {clip.name for clip in clips})
    if unknown:
        raise ValueError(f"unknown clips {unknown}; known: {[clip.name for clip in clips]}")
    return [clip for clip in clips if clip.name in names]


def lockstep_with_retry(
    clip: Clip, run_dir: Path, baselines: dict[str, Any], machine: str
) -> tuple[Digest, list[str], list[str]]:
    """One lockstep run, rerun once when it differs; both runs must differ to fail."""
    got = digest_mcap(run_clip(clip, "lockstep", run_dir / clip.name), "lockstep")
    problems, notes = compare_lockstep(clip.name, got, baselines, machine)
    if not problems or clip.name not in baselines["lockstep"].get(machine, {}):
        return got, problems, notes
    retry = digest_mcap(run_clip(clip, "lockstep", run_dir / f"{clip.name}_retry"), "lockstep")
    retry_problems, _ = compare_lockstep(clip.name, retry, baselines, machine)
    if retry_problems:
        return retry, problems + [f"{clip.name}: the rerun differs too"], notes
    flake = f"{clip.name}: first run differed and the rerun matched: {problems[0]}"
    return retry, [], notes + [flake]


def check(tier: str, names: list[str] | None = None) -> int:
    """Replay each clip and compare it with the baseline for this machine class."""
    machine = machine_class()
    baselines = load_baselines()
    run_dir = new_run_dir(tier)
    print(f"playback {tier} on {machine}; output in {run_dir}", flush=True)
    problems: list[str] = []
    results = {}
    for clip in selected_clips(names):
        start = time.monotonic()
        if tier == "lockstep":
            got, clip_problems, notes = lockstep_with_retry(clip, run_dir, baselines, machine)
        else:
            got = digest_mcap(run_clip(clip, tier, run_dir / clip.name), tier)
            clip_problems, notes = compare_realtime(clip.name, got, baselines, machine), []
        results[clip.name] = asdict(got)
        mark = "FAIL" if clip_problems else "ok  "
        print(f"{mark} {clip.name} ({time.monotonic() - start:.0f} s)", flush=True)
        for line in notes + clip_problems:
            print(f"     {line}", flush=True)
        problems += clip_problems
    (run_dir / "results.json").write_text(json.dumps(results, indent=2, sort_keys=True) + "\n")
    print(f"playback {tier}: " + ("FAIL" if problems else "PASS"))
    return 1 if problems else 0


def record_baseline(tier: str, names: list[str] | None = None, runs: int = AA_RUNS) -> int:
    """Replay clips and write this machine class's baseline for a tier."""
    machine = machine_class()
    baselines = load_baselines()
    run_dir = new_run_dir(tier)
    repeats = 2 if tier == "lockstep" else runs
    entry = baselines[tier].setdefault(machine, {})
    failed = False
    for clip in selected_clips(names):
        digests = [
            digest_mcap(run_clip(clip, tier, run_dir / f"{clip.name}_{i}"), tier)
            for i in range(repeats)
        ]
        if tier == "lockstep":
            if any(d.hashes != digests[0].hashes for d in digests[1:]):
                print(f"FAIL {clip.name}: two lockstep runs of the same code differ", flush=True)
                failed = True
                continue
            entry[clip.name] = asdict(digests[0])
        else:
            entry[clip.name] = realtime_bounds(digests)
        print(f"ok   {clip.name}: {json.dumps(entry[clip.name].get('metrics'))}", flush=True)
    save_baselines(baselines)
    print(f"wrote {tier} baseline for {machine} to {BASELINE_FILE}")
    return 1 if failed else 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="python -m auto_battlebot.verify.playback")
    parser.add_argument("action", choices=("check", "baseline", "machine"))
    parser.add_argument("--tier", choices=TIERS, default="lockstep")
    parser.add_argument("--clip", action="append", help="Limit to these clips")
    parser.add_argument(
        "--runs", type=int, default=AA_RUNS, help="A/A runs for a realtime baseline"
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.action == "machine":
        print(machine_class())
        return 0
    if not BINARY.exists():
        print(f"{BINARY} is missing; run scripts/build.sh", file=sys.stderr)
        return 2
    if args.action == "baseline":
        return record_baseline(args.tier, args.clip, args.runs)
    return check(args.tier, args.clip)


if __name__ == "__main__":
    sys.exit(main())
