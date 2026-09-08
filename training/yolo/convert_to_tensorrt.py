"""Convert a YOLO pose model (.pt or .onnx) to TensorRT engine format.

Engines are GPU- and TensorRT-version specific: an engine built on x86 cannot
run on Jetson (or vice versa). For Jetson deployment, copy the .pt or .onnx
to the Jetson and run this script there (e.g. --from-onnx if you have .onnx)
to produce the .engine used by the C++ app.

The C++ YoloKeypointModel expects:
- Input: single tensor, shape [1, 3, H, W] (NCHW, float32), e.g. [1, 3, 640, 640].
- Output: single tensor, shape [1, num_features, num_predictions], e.g. [1, 56, 8400]
  (features = 4 bbox + num_classes + num_keypoints*3).

Export from .pt uses Ultralytics model.export(format="engine"). Export from .onnx
uses TensorRT Builder + OnnxParser (no Ultralytics required).

For C++ YoloKeypointModel compatibility, prefer building from ONNX (--from-onnx):
  python training/yolo/convert_to_onnx.py model.pt
  python training/yolo/convert_to_tensorrt.py model.onnx --from-onnx -o data/models/model.engine
Engines built from .pt via Ultralytics may use a different plan format and fail to load in the
C++ runtime.

Output filenames include a platform tag (e.g. _x86_64_sm89, _aarch64_sm72) that
encodes both CPU architecture and GPU compute capability so incompatible engines
are not loaded by accident. --int8 adds a precision tag ahead of it
(model_int8_x86_64_sm86.engine), leaving FP16 filenames byte-identical.

INT8 needs calibration: activation ranges depend on the data, so real frames are run
through the network to measure them.

  python training/yolo/convert_to_tensorrt.py model.onnx --int8 \
      --calib-dir training/data/nhrl_robots_bbox_2class/val/images --calib-count 1000
"""

import argparse
import time
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import tensorrt as trt
import torch

from auto_battlebot.perception.trt_yolo import CPP_LETTERBOX_PADDING, preprocess_frame
from auto_battlebot.tensorrt_build import (
    DEFAULT_TIMING_CACHE,
    REPO_ROOT,
    engine_path_with_platform_tag,
    has_lean_runtime,
    load_timing_cache,
    save_timing_cache,
)

# Every engine in a sweep is the same architecture with different weights, so TensorRT's tactic
# timings are reusable. Persisting them turns each build after the first from a kernel-autotuning
# search into little more than weight serialisation: measured 396.4 s cold vs 4.2 s warm on
# yolo26n at 640x640, a 94x difference, with 7,175 timing entries reused.
#
# Builds are deliberately sequential and in-process. Fanning them across GPUs was tried and gives
# only ~3x, because each concurrent builder pays the full tactic search independently -- and three
# cold searches cost far more than one cold search plus N warm ones. Sequential also keeps the
# cache warm in memory between models within a single invocation.
# Repo-local so the cache travels with the checkout and is obviously disposable; `.cache/` is
# already gitignored. Not in $HOME, where it would silently outlive the project and be easy to
# forget when a TensorRT or driver upgrade invalidates it.

# The calibration cache holds one scale per tensor plus a version header, and nothing about the
# GPU: no architecture, no sm tag, no CPU arch. So it transports to another machine, which is
# what keeps a Jetson follow-up short. TensorRT does write its own version string and the
# algorithm name into it and rejects a mismatch, so a JetPack shipping a different TensorRT
# recalibrates from frames instead.
DEFAULT_INT8_CACHE_DIR = REPO_ROOT / ".cache" / "tensorrt" / "int8"
CALIB_ALGOS = ("entropy2", "minmax")
# First line of a calibration cache, e.g. ``TRT-101401-EntropyCalibration2``. TensorRT writes it
# and rejects a cache whose header does not match the running build, which is the whole reason a
# cache can be carried to another machine safely.
CALIB_CACHE_ALGO_NAMES = {"entropy2": "EntropyCalibration2", "minmax": "MinMaxCalibration"}
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")


def _scene_of(path: Path) -> str:
    """Recording a corpus frame came from. Corpus filenames are ``<scene>__frame_<n>.jpg``."""
    stem = path.stem
    return stem.split("__", 1)[0] if "__" in stem else stem


def fair_scene_allocation(scene_sizes: dict[str, int], total: int) -> dict[str, int]:
    """Max-min fair split of `total` frames across scenes. Deterministic, no RNG.

    The val split is heavily imbalanced: its three largest cage cameras hold a quarter of the
    frames, so a plain stride over the directory would calibrate mostly on those. Water-filling
    hands every scene an equal share, and re-divides what a scene cannot use among the scenes
    that still have frames left.
    """
    remaining = total
    allocation: dict[str, int] = {}
    ordered = sorted(scene_sizes.items(), key=lambda item: (item[1], item[0]))
    for index, (scene, size) in enumerate(ordered):
        take = min(size, remaining // (len(ordered) - index))
        allocation[scene] = take
        remaining -= take
    if remaining:
        raise SystemExit(f"could not allocate {total} calibration frames: {remaining} unplaced")
    return allocation


def expected_cache_header(algo: str) -> str:
    """The header line TensorRT writes for this build and calibrator."""
    major, minor, patch = (int(field) for field in trt.__version__.split(".")[:3])
    return f"TRT-{major}{minor:02d}{patch:02d}-{CALIB_CACHE_ALGO_NAMES[algo]}"


def check_cache_header(cache_path: Path, algo: str, have_frames: bool) -> None:
    """Refuse a cache this TensorRT will not accept when there are no frames to fall back on.

    A cache built by a different TensorRT is rejected by the builder, which then asks the
    calibrator for batches. With no --calib-dir there are none, so the build would finish having
    calibrated on nothing -- and the batch-count check cannot catch it, because a cache was read.
    Carrying caches to a Jetson is exactly the case this protects.
    """
    header = cache_path.read_text(encoding="utf-8", errors="replace").splitlines()[0].strip()
    expected = expected_cache_header(algo)
    if header == expected:
        return
    message = f"calibration cache {cache_path} has header {header!r}, this build wants {expected!r}"
    if not have_frames:
        raise SystemExit(f"{message}. Pass --calib-dir so it can recalibrate.")
    print(f"    warning: {message}. Recalibrating from frames.")


def parse_partition(text: str) -> tuple[int, int]:
    """Parse an ``I/N`` partition selector into (index, count)."""
    try:
        index, parts = (int(field) for field in text.split("/", 1))
    except ValueError:
        raise SystemExit(f"--calib-partition wants I/N, got {text!r}") from None
    if parts < 1 or not 0 <= index < parts:
        raise SystemExit(f"--calib-partition {text!r}: need 0 <= I < N and N >= 1")
    return index, parts


def select_calibration_frames(
    calib_dir: Path, count: int, partition: tuple[int, int] | None = None
) -> list[Path]:
    """`count` frames from `calib_dir`, stratified across scenes and evenly spaced within each.

    `partition` (index, parts) first thins each scene to every Nth frame, so two calls with
    different indices cannot return a frame in common however small the scene is. Shifting the
    picks by half a stride instead is not enough: the val split's smallest scenes are consumed
    whole, which left 162 of 1000 frames shared. That is how arm S gets a genuinely disjoint
    second sample out of the same directory, with no RNG anywhere.
    """
    images = sorted(path for path in calib_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
    if not images:
        raise SystemExit(f"no images found in --calib-dir {calib_dir}")
    if count > len(images):
        raise SystemExit(f"--calib-count {count} exceeds the {len(images)} images in {calib_dir}")
    by_scene: dict[str, list[Path]] = {}
    for image in images:
        by_scene.setdefault(_scene_of(image), []).append(image)
    if partition is not None:
        index, parts = partition
        by_scene = {scene: frames[index::parts] for scene, frames in by_scene.items()}
        by_scene = {scene: frames for scene, frames in by_scene.items() if frames}
    allocation = fair_scene_allocation({s: len(f) for s, f in by_scene.items()}, count)
    picked: list[Path] = []
    for scene, take in allocation.items():
        frames = by_scene[scene]
        stride = len(frames) / take if take else 0.0
        for i in range(take):
            picked.append(frames[min(len(frames) - 1, int(i * stride))])
    unique = sorted(set(picked))
    if len(unique) != count:
        raise SystemExit(f"frame selection produced {len(unique)} distinct frames, wanted {count}")
    return unique


@dataclass
class Int8Config:
    """What the INT8 path needs and the FP16 path does not."""

    frames: list[Path]
    cache_path: Path
    algo: str


def _make_calibrator(int8: Int8Config, input_h: int, input_w: int) -> Any:
    """Feed preprocessed corpus frames to the builder, one at a time.

    Preprocessing goes through `preprocess_frame`, the same letterbox `score.py` and the C++
    pipeline use. Calibration is a measurement of activation ranges, so a preprocessing mismatch
    between calibration and inference poisons every scale factor in the network. Two details:
    `letterbox_padding` has to be passed explicitly, because `preprocess_frame` defaults it to
    0.0 while `TrtYoloModel` and the deployed engine use 0.1; and the blob comes back transposed
    and non-contiguous, so it needs the same `ascontiguousarray` that `TrtYoloModel._run` does.
    """
    base = trt.IInt8EntropyCalibrator2 if int8.algo == "entropy2" else trt.IInt8MinMaxCalibrator

    class _FrameCalibrator(base):  # type: ignore[misc, valid-type]
        def __init__(self) -> None:
            super().__init__()
            self.index = 0
            self.batches_served = 0
            self.cache_hit = False
            # TensorRT receives a raw device address and does nothing to keep the allocation
            # alive, so the buffer is held on the instance. torch already owns a CUDA context
            # here; pycuda would be a second dependency for the same pointer.
            self._buffer = torch.empty((1, 3, input_h, input_w), dtype=torch.float32, device="cuda")

        def get_batch_size(self) -> int:
            return 1

        def get_batch(self, names: list[str]) -> list[int] | None:
            del names  # single-input network; the address order is the network's own
            if self.index >= len(int8.frames):
                return None
            path = int8.frames[self.index]
            self.index += 1
            image = cv2.imread(str(path))
            if image is None:
                raise RuntimeError(f"unreadable calibration frame: {path}")
            blob, *_ = preprocess_frame(
                image, input_h, input_w, letterbox_padding=CPP_LETTERBOX_PADDING
            )
            self._buffer.copy_(torch.from_numpy(np.ascontiguousarray(blob, dtype=np.float32)))
            self.batches_served += 1
            return [int(self._buffer.data_ptr())]

        def read_calibration_cache(self) -> bytes | None:
            if int8.cache_path.is_file():
                self.cache_hit = True
                return int8.cache_path.read_bytes()
            return None

        def write_calibration_cache(self, cache: memoryview) -> None:
            int8.cache_path.parent.mkdir(parents=True, exist_ok=True)
            int8.cache_path.write_bytes(bytes(cache))

    return _FrameCalibrator()


def _configure_int8(
    config: "trt.IBuilderConfig", network: "trt.INetworkDefinition", int8: Int8Config | None
) -> Any:
    """Turn on INT8 and attach the calibrator. Returns the calibrator, or None for FP16."""
    if int8 is None:
        return None
    # FP16 stays set alongside INT8 so layers the builder will not quantize fall back to FP16
    # rather than to FP32.
    config.set_flag(trt.BuilderFlag.INT8)
    # The calibrator has to feed the network its real geometry, so the size comes from the parsed
    # input shape rather than from --imgsz. That is what makes the 384x640 arms work with no
    # second flag.
    _, _, input_h, input_w = (int(dim) for dim in network.get_input(0).shape)
    calibrator = _make_calibrator(int8, input_h, input_w)
    print(
        f"    INT8 {int8.algo}: {len(int8.frames)} frames at {input_h}x{input_w}, "
        f"cache {int8.cache_path}"
    )
    with warnings.catch_warnings():
        # Implicit calibration is deprecated in favour of explicit Q/DQ, a different export
        # pipeline and out of scope. The warning fires on this one assignment.
        warnings.simplefilter("ignore", DeprecationWarning)
        config.int8_calibrator = calibrator
    return calibrator


def _check_calibration_ran(calibrator: Any, int8: Int8Config | None) -> None:
    """Fail a build whose calibrator did not consume every frame it was given.

    TensorRT catches and swallows every exception raised inside `get_batch`, then finishes the
    build with whatever batches it already had. Without this check one corrupt JPEG becomes a
    quietly miscalibrated engine that loads and runs normally.
    """
    if calibrator is None or int8 is None or calibrator.cache_hit:
        return
    if calibrator.batches_served != len(int8.frames):
        raise RuntimeError(
            f"calibration consumed {calibrator.batches_served} of {len(int8.frames)} frames. "
            "TensorRT swallowed an error inside get_batch; the engine is miscalibrated"
        )


def build_engine_from_onnx(
    onnx_path: Path,
    engine_path: Path,
    *,
    fp16: bool = True,
    workspace_gib: int = 4,
    version_compatible: bool = False,
    logger: trt.ILogger | None = None,
    timing_cache: Path | None = None,
    int8: Int8Config | None = None,
) -> None:
    """Build a TensorRT engine from an ONNX file (fixed input shape)."""
    if logger is None:
        logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)

    onnx_path = Path(onnx_path)
    if not parser.parse_from_file(str(onnx_path)):
        for i in range(parser.num_errors):
            print(parser.get_error(i))
        raise RuntimeError("Failed to parse ONNX file")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_gib << 30)
    load_timing_cache(config, timing_cache)
    if fp16 and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    calibrator = _configure_int8(config, network, int8)
    if version_compatible:
        if hasattr(trt.BuilderFlag, "VERSION_COMPATIBLE") and has_lean_runtime():
            config.set_flag(trt.BuilderFlag.VERSION_COMPATIBLE)
        else:
            print(
                "Warning: --version-compatible requested, but TensorRT VERSION_COMPATIBLE "
                "or lean runtime is unavailable. Building without VERSION_COMPATIBLE."
            )

    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("Failed to build TensorRT engine")
    _check_calibration_ran(calibrator, int8)
    save_timing_cache(config, timing_cache)
    engine_path = Path(engine_path)
    engine_path.parent.mkdir(parents=True, exist_ok=True)
    with open(engine_path, "wb") as f:
        f.write(serialized)


def resolve_output(model_path: Path, output: str | None, precision_tag: str | None = None) -> Path:
    """Output engine path for one model, with the platform + compute-capability tag applied."""
    if output:
        return engine_path_with_platform_tag(Path(output), precision_tag)
    return engine_path_with_platform_tag(
        model_path.parent / f"{model_path.stem}.engine", precision_tag
    )


def resolve_int8_config(model_path: Path, args: argparse.Namespace) -> Int8Config | None:
    """Calibration frames and cache path for one model, or None when building FP16."""
    if not args.int8:
        return None
    partition = parse_partition(args.calib_partition) if args.calib_partition else None
    # The sample is part of the cache key, not just the model. TensorRT reads whatever cache it
    # finds and skips calibration entirely, so a name keyed on the ONNX stem alone would hand a
    # rebuild with a different sample the old scales and report success. That is precisely the
    # arm S comparison, silently answered with a copy of the arm it was meant to check.
    sample_tag = f"_{args.calib_count}" + (f"_p{partition[0]}-{partition[1]}" if partition else "")
    cache_path = (
        Path(args.calib_cache)
        if args.calib_cache
        else DEFAULT_INT8_CACHE_DIR / f"{model_path.stem}{sample_tag}.calib"
    )
    frames: list[Path] = []
    if args.calib_dir:
        frames = select_calibration_frames(Path(args.calib_dir), args.calib_count, partition)
        # An audit trail for which frames produced which scales, and the only way to check that
        # a second sample really is disjoint from the first.
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        manifest = cache_path.with_suffix(".frames.txt")
        # Explicit utf-8: 86 of the corpus filenames carry non-ASCII characters (a fullwidth
        # question mark, from the YouTube titles the scenes are named after), and write_text
        # otherwise encodes with whatever locale the process has by then. Building on the Orin,
        # the second model of a multi-model run wrote a 0-byte manifest and died on
        # UnicodeEncodeError where the first had succeeded in the same process.
        manifest.write_text("".join(f"{path}\n" for path in frames), encoding="utf-8")
    elif not cache_path.is_file():
        raise SystemExit(
            f"--int8 needs --calib-dir or an existing calibration cache at {cache_path}"
        )
    if cache_path.is_file():
        check_cache_header(cache_path, args.calib_algo, bool(frames))
    return Int8Config(frames=frames, cache_path=cache_path, algo=args.calib_algo)


def build_one(model_path: Path, output_path: Path, args: argparse.Namespace) -> float:
    """Build a single engine in this process. Returns elapsed seconds."""
    started = time.perf_counter()
    build_engine_from_onnx(
        model_path,
        output_path,
        fp16=not args.no_fp16,
        workspace_gib=args.workspace,
        version_compatible=args.version_compatible,
        timing_cache=None if args.no_timing_cache else Path(args.timing_cache),
        int8=resolve_int8_config(model_path, args),
    )
    return time.perf_counter() - started


def main() -> None:
    """Convert one or more ONNX models to TensorRT engines."""
    parser = argparse.ArgumentParser(
        description="Convert YOLO models to TensorRT engines for the C++ runtime"
    )
    parser.add_argument("model", type=str, nargs="+", help="One or more ONNX files (.onnx)")
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help=(
            "Output path for the engine; platform+GPU tag (e.g. _x86_64_sm89) is appended to the "
            "stem. Only valid with a single input model (default: alongside each input)"
        ),
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Image size H=W for export (default: 640). Must match C++ config image_size.",
    )
    parser.add_argument("--no-fp16", action="store_true", help="Disable FP16; build FP32 only")
    parser.add_argument(
        "--int8",
        action="store_true",
        help="Build an INT8 engine, calibrated on --calib-dir frames. FP16 stays enabled so "
        "layers TensorRT will not quantize fall back to FP16 rather than FP32. The output "
        "filename gains an _int8 tag ahead of the platform tag.",
    )
    parser.add_argument(
        "--calib-dir",
        type=str,
        help="Directory of calibration images, stratified across scenes by --calib-count. "
        "Use the val split of the training corpus, never the eval set being scored.",
    )
    parser.add_argument(
        "--calib-count", type=int, default=1000, help="Calibration frames to use (default: 1000)"
    )
    parser.add_argument(
        "--calib-partition",
        type=str,
        metavar="I/N",
        help="Draw only from every Nth frame of each scene, starting at I (e.g. 1/2). Two "
        "partitions of the same N share no frames, which is how a disjoint second "
        "calibration sample is built from one directory.",
    )
    parser.add_argument(
        "--calib-cache",
        type=str,
        help=f"Calibration cache file (default: {DEFAULT_INT8_CACHE_DIR}/<onnx-stem>.calib). "
        "It is platform independent but keyed to the TensorRT version and algorithm name.",
    )
    parser.add_argument(
        "--calib-algo",
        choices=CALIB_ALGOS,
        default="entropy2",
        help="entropy2 clips outliers to keep resolution on the common case; minmax never "
        "clips (default: entropy2)",
    )
    parser.add_argument(
        "--workspace", type=int, default=4, metavar="GIB", help="Workspace size in GiB (default: 4)"
    )
    parser.add_argument(
        "--timing-cache",
        type=str,
        default=str(DEFAULT_TIMING_CACHE),
        help=f"TensorRT timing cache file (default: {DEFAULT_TIMING_CACHE}). Tactic timings are "
        "reusable across models of the same architecture, so the first build pays the autotuning "
        "cost and the rest do not.",
    )
    parser.add_argument(
        "--no-timing-cache",
        action="store_true",
        help="Disable the timing cache and re-run the full tactic search for every build.",
    )
    parser.add_argument(
        "--version-compatible",
        action="store_true",
        help=(
            "Enable TensorRT VERSION_COMPATIBLE (requires lean runtime). Disabled by "
            "default to avoid host-code deserialization requirements in some loaders."
        ),
    )
    args = parser.parse_args()

    models = [Path(m) for m in args.model]
    for model_path in models:
        if not model_path.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")
        if model_path.suffix.lower() != ".onnx":
            raise ValueError(f"Expected an .onnx file, got {model_path}")
    if args.output and len(models) > 1:
        raise SystemExit("--output is only valid with a single input model")
    if args.int8 and args.no_fp16:
        raise SystemExit("--int8 --no-fp16 would send unquantizable layers to FP32 instead of FP16")
    if args.calib_cache and len(models) > 1:
        raise SystemExit("--calib-cache is only valid with a single input model")
    if not args.int8 and (args.calib_dir or args.calib_cache):
        raise SystemExit("--calib-dir / --calib-cache have no effect without --int8")

    print(f"TensorRT version: {trt.__version__}")
    cache_note = "disabled" if args.no_timing_cache else args.timing_cache
    print(f"{len(models)} model(s), timing cache: {cache_note}")

    total = 0.0
    for index, model_path in enumerate(models, 1):
        output_path = resolve_output(model_path, args.output, "int8" if args.int8 else None)
        print(f"[{index}/{len(models)}] {model_path.name} -> {output_path.name}")
        elapsed = build_one(model_path, output_path, args)
        total += elapsed
        print(f"    built in {elapsed:.1f}s")
    if len(models) > 1:
        print(f"Done. {len(models)} engines in {total:.1f}s ({total / len(models):.1f}s each)")


if __name__ == "__main__":
    main()
