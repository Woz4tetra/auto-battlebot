import argparse
import os
from datetime import datetime

# Training is headless. Force a non-interactive matplotlib backend before ultralytics
# imports matplotlib: an inherited (e.g. SSH-forwarded) DISPLAY otherwise makes it pick
# TkAgg, which ultralytics fails to restore after plotting metrics and crashes the run.
os.environ["MPLBACKEND"] = "Agg"

from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


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
            "batch": 32,
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
    args = parser.parse_args()

    dataset = args.dataset
    models = args.models
    epochs = args.epochs
    checkpoint_path = args.checkpoint
    devices = tuple(args.devices)
    workers = args.workers
    cache = False if args.cache == "false" else args.cache

    if len(devices) == 0:
        devices_filtered = None
    elif len(devices) == 1:
        devices_filtered = devices[0]
    else:
        devices_filtered = devices

    # One timestamp per script run so all models in this session share the same date.
    session_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    hyper_params = dict(
        lr0=0.01,  # (float) initial learning rate (i.e. SGD=1E-2, Adam=1E-3)
        lrf=0.01,  # (float) final learning rate (lr0 * lrf)
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
        degrees=180.0,  # (float) image rotation (+/- deg)
        translate=0.5,  # (float) image translation (+/- fraction)
        scale=0.5,  # (float) image scale (+/- gain)
        shear=10.0,  # (float) image shear (+/- deg)
        perspective=0.001,  # (float) image perspective (+/- fraction), range 0-0.001
        flipud=0.5,  # (float) image flip up-down (probability)
        fliplr=0.5,  # (float) image flip left-right (probability)
        bgr=0.0,  # (float) image channel BGR (probability)
        mosaic=0.4,  # (float) image mosaic (probability)
        mixup=0.1,  # (float) image mixup (probability)
        copy_paste=0.2,  # (float) segment copy-paste (probability)
        copy_paste_mode="flip",  # (str) the method to do copy_paste augmentation (flip, mixup)
        # (str) auto augmentation policy for classification (randaugment, autoaugment, augmix)
        auto_augment="randaugment",
        # (float) probability of random erasing during classification training (0-0.9),
        # 0 means no erasing, must be less than 1.0.
        erasing=0.4,
        # (float) image crop fraction for classification (0.1-1), 1.0 means no crop,
        # must be greater than 0.
        crop_fraction=0.2,
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
            **hyper_params,
            **settings,
        )


if __name__ == "__main__":
    main()
