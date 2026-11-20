import argparse
import os
from datetime import datetime
from pathlib import Path

os.environ["MPLBACKEND"] = "Agg"

import yaml
from clear_image_cache import sweep
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_ROOT = Path(BASE_DIR).resolve().parents[0] / "data"
CACHE_MAX_AGE_DAYS = 7.0
DEFAULT_CACHE = "disk"


def reclaim_stale_caches(dataset_yaml: str, max_age_days: float) -> None:
    """Delete ultralytics disk caches unused for ``max_age_days``, before training starts."""
    protect: set[Path] = set()
    try:
        meta = yaml.safe_load(Path(dataset_yaml).read_text())
        if isinstance(meta, dict) and meta.get("path"):
            protect.add(Path(str(meta["path"])).resolve())
    except (OSError, yaml.YAMLError):
        pass
    protect.add(Path(dataset_yaml).resolve().parent)

    try:
        freed, removed = sweep(DATA_ROOT, max_age_days, protect=protect)
    except Exception as exc:  # noqa: BLE001
        print(f"cache cleanup skipped: {exc}")
        return
    if removed:
        print(f"reclaimed {freed / 1e9:.1f} GB of stale image cache ({removed} files)")


def resolve_dataset(dataset: str) -> str:
    """Return the yaml inside a dataset directory, or the path unchanged."""
    path = Path(dataset)
    if not path.is_dir():
        return dataset
    for name in ("data.yml", "data.yaml"):
        candidate = path / name
        if candidate.exists():
            return str(candidate)
    raise SystemExit(f"No data.yml or data.yaml in dataset directory {path}")


def resolve_model_settings(
    config: dict, epochs: int, batch: int, imgsz: int, cache_override: str | bool | None
) -> tuple[str | bool, dict]:
    """Split a model's config into its cache and the rest; --cache beats the config's cache."""
    settings = dict(config)
    if epochs > 0:
        settings["epochs"] = epochs
    if batch > 0:
        settings["batch"] = batch
    if imgsz > 0:
        settings["imgsz"] = imgsz
    model_cache = settings.pop("cache", DEFAULT_CACHE)
    return (model_cache if cache_override is None else cache_override), settings


def main() -> None:
    configs = {
        "yolo11n-pose": {
            "batch": 32,
            "epochs": 500,
            "imgsz": 640,
        },
        "yolo11l-pose": {
            "batch": 16,
            "epochs": 300,
            "imgsz": 640,
        },
        "yolo26x-pose": {
            "batch": 32,
            "epochs": 500,
            "imgsz": 640,
        },
        "yolo26n-pose": {
            "batch": 96,
            "epochs": 500,
            "imgsz": 640,
        },
        "yolo26s-pose": {
            "batch": 64,
            "epochs": 500,
            "imgsz": 640,
        },
        "yolo26m-pose": {
            "batch": 64,
            "epochs": 500,
            "imgsz": 640,
        },
        "yolo26l-pose": {
            "batch": 64,
            "epochs": 500,
            "imgsz": 640,
        },
        "yolo26n-seg": {
            "batch": 32,
            "epochs": 500,
            "imgsz": 640,
        },
        "yolo26n": {
            "batch": 128,
            "epochs": 100,
            "imgsz": 640,
            "cache": "ram",
        },
        "yolo26s": {"batch": 64, "epochs": 100, "imgsz": 640, "cache": "ram"},
        "yolo26m": {"batch": 64, "epochs": 100, "imgsz": 640, "cache": "ram"},
        "yolo26l": {"batch": 64, "epochs": 100, "imgsz": 640, "cache": "ram"},
        "yolo26x": {"batch": 64, "epochs": 100, "imgsz": 640, "cache": "ram"},
    }

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "dataset",
        type=str,
        help="Path to dataset yaml",
    )
    parser.add_argument(
        "models",
        nargs="+",
        type=str,
        help="Model key to train. ex: yolov11l-pose",
    )
    parser.add_argument(
        "-c",
        "--checkpoint",
        default="",
        type=str,
        help="Resume from checkpoint",
    )
    parser.add_argument(
        "-e",
        "--epochs",
        default=0,
        type=int,
        help="Overwrite number of epochs",
    )
    parser.add_argument(
        "-d",
        "--devices",
        nargs="+",
        default=[0, 1, 2],
        type=int,
        help="List of indexed GPUs to use for training",
    )
    parser.add_argument(
        "-w",
        "--workers",
        default=12,
        type=int,
        help="Number of worker threads",
    )
    parser.add_argument(
        "-b",
        "--batch",
        default=0,
        type=int,
        help="Override the model's batch size (total across GPUs). 0 keeps the per-model config.",
    )
    parser.add_argument(
        "--imgsz",
        default=0,
        type=int,
        help="Override the model's input size (a single int). 0 keeps the per-model config.",
    )
    parser.add_argument(
        "--rect",
        action="store_true",
        help="Rectangular training: size batches to the images' aspect ratio, no letterbox.",
    )
    parser.add_argument(
        "--cache",
        default=None,
        choices=["ram", "disk", "false"],
        help=f"Image cache, overrides the model's setting (default '{DEFAULT_CACHE}').",
    )
    parser.add_argument(
        "--save-period",
        default=0,
        type=int,
        help="Save a checkpoint every N epochs besides last.pt/best.pt (0 disables).",
    )
    parser.add_argument(
        "--fraction",
        default=1.0,
        type=float,
        help="Train on the first FRACTION of the shuffled train split (default 1.0).",
    )
    parser.add_argument(
        "--seed",
        default=0,
        type=int,
        help="Training RNG seed (0 = Ultralytics default).",
    )
    parser.add_argument(
        "--lrf",
        default=0.1,
        type=float,
        help="Final LR as a fraction of lr0 (final = lr0 * lrf), default 0.1.",
    )
    parser.add_argument(
        "--cos-lr",
        action="store_true",
        help="Cosine LR decay instead of linear.",
    )
    parser.add_argument(
        "--close-mosaic",
        default=0,
        type=int,
        help="Disable mosaic for the final N epochs (0 = never disable).",
    )
    parser.add_argument(
        "--patience",
        default=100,
        type=int,
        help="Early-stop patience on val fitness.",
    )
    parser.add_argument(
        "--degrees",
        default=45.0,
        type=float,
        help="Rotation augmentation, +/- degrees.",
    )
    parser.add_argument(
        "--cache-max-age",
        default=CACHE_MAX_AGE_DAYS,
        type=float,
        metavar="DAYS",
        help=f"Before training, delete ultralytics disk caches unused for this many days "
        f"(default: {CACHE_MAX_AGE_DAYS:g}). The dataset being trained on is always spared.",
    )
    parser.add_argument(
        "--no-clear-cache",
        action="store_true",
        help="Skip the pre-training cache sweep entirely.",
    )
    parser.add_argument(
        "--flipud",
        default=0.0,
        type=float,
        help="Vertical-flip probability; default 0.0, Ultralytics uses 0.5.",
    )
    args = parser.parse_args()

    dataset = resolve_dataset(args.dataset)
    models = args.models
    epochs = args.epochs
    checkpoint_path = args.checkpoint
    devices = tuple(args.devices)
    workers = args.workers
    cache_override = False if args.cache == "false" else args.cache
    save_period = args.save_period
    fraction = args.fraction
    seed = args.seed

    if len(devices) == 0:
        devices_filtered = None
    elif len(devices) == 1:
        devices_filtered = devices[0]
    else:
        devices_filtered = devices

    if not args.no_clear_cache:
        reclaim_stale_caches(dataset, args.cache_max_age)

    session_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    hyper_params = dict(
        lr0=0.01,
        lrf=args.lrf,
        cos_lr=args.cos_lr,
        close_mosaic=args.close_mosaic,
        patience=args.patience,
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=3.0,
        warmup_momentum=0.8,
        warmup_bias_lr=0.1,
        box=7.5,
        cls=0.5,
        dfl=1.5,
        pose=12.0,
        kobj=1.0,
        label_smoothing=0.0,
        nbs=64,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=args.degrees,
        translate=0.5,
        scale=0.5,
        shear=10.0,
        flipud=args.flipud,
        perspective=0.001,
        fliplr=0.5,
        bgr=0.0,
        mosaic=0.4,
        mixup=0.1,
        copy_paste=0.2,
        copy_paste_mode="flip",
    )

    for model_key in models:
        cache, settings = resolve_model_settings(
            configs[model_key], epochs, args.batch, args.imgsz, cache_override
        )

        model = YOLO(checkpoint_path if checkpoint_path else model_key)

        run_name = f"auto_battlebots_{session_date}_{model_key}"
        model.train(
            data=dataset,
            name=run_name,
            project="../projects",
            device=devices_filtered,
            workers=workers,
            cache=cache,
            save_period=save_period,
            fraction=fraction,
            seed=seed,
            rect=args.rect,
            **hyper_params,
            **settings,
        )


if __name__ == "__main__":
    main()
