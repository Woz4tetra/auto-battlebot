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
are not loaded by accident.
"""

import argparse
import ctypes
import platform
import time
from pathlib import Path

import tensorrt as trt
import torch

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
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TIMING_CACHE = REPO_ROOT / ".cache" / "tensorrt" / "timing.cache"


def _has_lean_runtime() -> bool:
    """Check if the TensorRT lean runtime is available (needed for VERSION_COMPATIBLE)."""
    try:
        ctypes.CDLL("libnvinfer_lean.so.10")
        return True
    except OSError:
        return False


def _get_compute_capability() -> tuple[int, int]:
    """Query GPU compute capability of device 0."""
    if torch.cuda.is_available():
        return torch.cuda.get_device_capability(0)
    raise RuntimeError("CUDA not available — cannot determine GPU compute capability")


def engine_path_with_platform_tag(path: Path) -> Path:
    """Append platform + GPU compute capability tag so incompatible engines are distinct.

    Produces filenames like ``model_x86_64_sm89.engine`` — the ``sm`` tag
    prevents silently loading an engine built for a different GPU architecture.
    """
    arch = platform.machine()
    major, minor = _get_compute_capability()
    tag = f"{arch}_sm{major}{minor}"
    suffix = path.suffix if path.suffix else ".engine"
    return path.parent / f"{path.stem}_{tag}{suffix}"


def _load_timing_cache(config: "trt.IBuilderConfig", path: Path | None) -> None:
    """Seed the builder with previously measured tactic timings, if any."""
    if path is None:
        return
    blob = path.read_bytes() if path.is_file() else b""
    cache = config.create_timing_cache(blob)
    if cache is not None:
        config.set_timing_cache(cache, ignore_mismatch=False)


def _save_timing_cache(config: "trt.IBuilderConfig", path: Path | None) -> None:
    """Persist tactic timings so the next build can skip the autotuning search."""
    if path is None:
        return
    cache = config.get_timing_cache()
    if cache is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    # Last writer wins. Concurrent builders may race here; the cache is advisory, and a lost
    # update only costs the next build some re-timing.
    path.write_bytes(memoryview(cache.serialize()))


def build_engine_from_onnx(
    onnx_path: Path,
    engine_path: Path,
    *,
    fp16: bool = True,
    workspace_gib: int = 4,
    version_compatible: bool = False,
    logger: trt.ILogger | None = None,
    timing_cache: Path | None = None,
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
    _load_timing_cache(config, timing_cache)
    if fp16 and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    if version_compatible:
        if hasattr(trt.BuilderFlag, "VERSION_COMPATIBLE") and _has_lean_runtime():
            config.set_flag(trt.BuilderFlag.VERSION_COMPATIBLE)
        else:
            print(
                "Warning: --version-compatible requested, but TensorRT VERSION_COMPATIBLE "
                "or lean runtime is unavailable. Building without VERSION_COMPATIBLE."
            )

    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("Failed to build TensorRT engine")
    _save_timing_cache(config, timing_cache)
    engine_path = Path(engine_path)
    engine_path.parent.mkdir(parents=True, exist_ok=True)
    with open(engine_path, "wb") as f:
        f.write(serialized)


def resolve_output(model_path: Path, output: str | None) -> Path:
    """Output engine path for one model, with the platform + compute-capability tag applied."""
    if output:
        return engine_path_with_platform_tag(Path(output))
    return engine_path_with_platform_tag(model_path.parent / f"{model_path.stem}.engine")


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

    print(f"TensorRT version: {trt.__version__}")
    cache_note = "disabled" if args.no_timing_cache else args.timing_cache
    print(f"{len(models)} model(s), timing cache: {cache_note}")

    total = 0.0
    for index, model_path in enumerate(models, 1):
        output_path = resolve_output(model_path, args.output)
        print(f"[{index}/{len(models)}] {model_path.name} -> {output_path.name}")
        elapsed = build_one(model_path, output_path, args)
        total += elapsed
        print(f"    built in {elapsed:.1f}s")
    if len(models) > 1:
        print(f"Done. {len(models)} engines in {total:.1f}s ({total / len(models):.1f}s each)")


if __name__ == "__main__":
    main()
