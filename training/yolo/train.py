import argparse
import os
from datetime import datetime
from pathlib import Path

# Training is headless. Force a non-interactive matplotlib backend before ultralytics
# imports matplotlib: an inherited (e.g. SSH-forwarded) DISPLAY otherwise makes it pick
# TkAgg, which ultralytics fails to restore after plotting metrics and crashes the run.
os.environ["MPLBACKEND"] = "Agg"

import yaml
from clear_image_cache import sweep
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_ROOT = Path(BASE_DIR).resolve().parents[0] / "data"
CACHE_MAX_AGE_DAYS = 7.0


def reclaim_stale_caches(dataset_yaml: str, max_age_days: float) -> None:
    """Delete ultralytics disk caches unused for ``max_age_days``, before training starts.

    ``cache="disk"`` writes one ``.npy`` per image and never prunes them: on megamind a single
    corpus grew to 196 GB of cache against 12 GB of images, and a stale set from an abandoned
    dataset can fill the NVMe and stall an unrelated run. Deleting is safe -- ultralytics rebuilds
    what it needs -- so the only cost of being wrong is re-caching time.

    The dataset about to be trained on is protected explicitly. ``busy_datasets`` skips this process
    and its ancestors (so that a shell mentioning a path does not pin it), which means it cannot see
    that *we* are the ones about to use this data.

    Never fatal: a failure here must not take down a multi-hour training run.
    """
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
    except Exception as exc:  # noqa: BLE001 - never block training on cache cleanup
        print(f"cache cleanup skipped: {exc}")
        return
    if removed:
        print(f"reclaimed {freed / 1e9:.1f} GB of stale image cache ({removed} files)")


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
        "yolo26n-seg": {
            "batch": 32,
            "epochs": 500,
            "imgsz": 640,
        },
        # Detect head; settings match yolo26n-seg so the only variable in a seg-vs-bbox
        # box-quality comparison is the head type, not batch/epochs/imgsz.
        "yolo26n": {
            "batch": 128,
            "epochs": 500,
            "imgsz": 640,
        },
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
        "--cache",
        default="disk",
        choices=["ram", "disk", "false"],
        help="Image cache: 'ram' avoids per-epoch disk IO if the resized cache fits in RAM. "
        "'disk' (default) reads full-res .npy each epoch, starving GPUs if it exceeds RAM.",
    )
    parser.add_argument(
        "--save-period",
        default=0,
        type=int,
        help="Save a checkpoint every N epochs (weights/epoch{0,N,2N,...}.pt) in addition to "
        "last.pt/best.pt. Default 0 disables periodic saving (current behavior). Turns one long "
        "run into an epoch-vs-metric ladder for the data_epoch_min experiment.",
    )
    parser.add_argument(
        "--fraction",
        default=1.0,
        type=float,
        help="Train on the first FRACTION of the (shuffled) train split; val untouched. Default "
        "1.0 uses all data. Cheap single-source real-data lever for the data-floor sweeps. Note: "
        "this subsamples one dataset and cannot set a real:synthetic ratio (use pool_datasets.py).",
    )
    parser.add_argument(
        "--seed",
        default=0,
        type=int,
        help="Training RNG seed (default 0 = Ultralytics default). Vary it to measure run-to-run "
        "variance, which data_epoch_min Exp 1 found (~0.05 recall) can exceed the parity margin.",
    )
    parser.add_argument(
        "--lrf",
        default=0.1,
        type=float,
        help="Final LR as a fraction of lr0 (final = lr0 * lrf). Default 0.1, raised from "
        "Ultralytics' 0.01: category_addition_2026-07-25 found every fully-annealed 150-epoch "
        "endpoint failed the parity gate while ep75-100 cleared it, so a less-annealed endpoint "
        "is what generalizes on the robot-camera eval.",
    )
    parser.add_argument(
        "--cos-lr",
        action="store_true",
        help="Cosine LR decay instead of linear. Untested on this data; use to A/B the schedule "
        "shape against the default linear decay.",
    )
    parser.add_argument(
        "--close-mosaic",
        default=0,
        type=int,
        help="Disable mosaic for the final N epochs (0 = never disable). Ultralytics' default 10 "
        "overlaps the steepest part of the late external-eval decline; pass 0 to test whether it "
        "contributes.",
    )
    parser.add_argument(
        "--patience",
        default=100,
        type=int,
        help="Early-stop patience on val fitness. Only meaningful once val represents the "
        "deployment camera: training data is NHRL overhead cage footage while the eval is the "
        "robot's own ZED, so val fitness keeps improving while eval recall collapses and this "
        "never fires. See category_addition_2026-07-25.",
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
        help="Vertical-flip probability. Default 0.0, lowered from 0.5: the deployment ZED has a "
        "fixed up-vector and never sees the arena inverted, so those batches train on an "
        "orientation that cannot occur at inference.",
    )
    args = parser.parse_args()

    dataset = args.dataset
    models = args.models
    epochs = args.epochs
    checkpoint_path = args.checkpoint
    devices = tuple(args.devices)
    workers = args.workers
    cache = False if args.cache == "false" else args.cache
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

    # One timestamp per script run so all models in this session share the same date.
    session_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    hyper_params = dict(
        lr0=0.01,  # (float) initial learning rate (i.e. SGD=1E-2, Adam=1E-3)
        lrf=args.lrf,  # (float) final learning rate (lr0 * lrf); see --lrf
        cos_lr=args.cos_lr,  # (bool) cosine LR schedule instead of linear
        close_mosaic=args.close_mosaic,  # (int) disable mosaic for the last N epochs
        patience=args.patience,  # (int) early-stop patience on val fitness
        momentum=0.937,  # (float) SGD momentum/Adam beta1
        weight_decay=0.0005,  # (float) optimizer weight decay 5e-4
        warmup_epochs=3.0,  # (float) warmup epochs (fractions ok)
        warmup_momentum=0.8,  # (float) warmup initial momentum
        warmup_bias_lr=0.1,  # (float) warmup initial bias lr
        box=7.5,  # (float) box loss gain
        cls=0.5,  # (float) cls loss gain (scale with pixels)
        dfl=1.5,  # (float) dfl loss gain
        pose=12.0,  # (float) pose loss gain
        kobj=1.0,  # (float) keypoint obj loss gain
        label_smoothing=0.0,  # (float) label smoothing (fraction)
        nbs=64,  # (int) nominal batch size
        hsv_h=0.015,  # (float) image HSV-Hue augmentation (fraction)
        hsv_s=0.7,  # (float) image HSV-Saturation augmentation (fraction)
        hsv_v=0.4,  # (float) image HSV-Value augmentation (fraction)
        degrees=args.degrees,  # (float) image rotation (+/- deg); see --degrees
        translate=0.5,  # (float) image translation (+/- fraction)
        scale=0.5,  # (float) image scale (+/- gain)
        shear=10.0,  # (float) image shear (+/- deg)
        flipud=args.flipud,  # (float) image flip up-down (probability); see --flipud
        perspective=0.001,  # (float) image perspective (+/- fraction), range 0-0.001
        fliplr=0.5,  # (float) image flip left-right (probability)
        bgr=0.0,  # (float) image channel BGR (probability)
        mosaic=0.4,  # (float) image mosaic (probability)
        mixup=0.1,  # (float) image mixup (probability)
        copy_paste=0.2,  # (float) segment copy-paste (probability)
        copy_paste_mode="flip",  # (str) the method to do copy_paste augmentation (flip, mixup)
        # `auto_augment`, `erasing` and `crop_fraction` are deliberately absent: Ultralytics only
        # reads them in ClassificationDataset / classify_transforms (data/dataset.py:753), so they
        # are inert for detect and pose and were only making this config look more tuned than it
        # was. `crop_fraction` is additionally deprecated upstream.
    )

    for model_key in models:
        settings = configs[model_key]
        if epochs > 0:
            settings["epochs"] = epochs

        # Load the model.
        model = YOLO(checkpoint_path if checkpoint_path else model_key)

        # Training in a new folder per session (date + time so same-day runs don't overwrite).
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
            **hyper_params,
            **settings,
        )


if __name__ == "__main__":
    main()
