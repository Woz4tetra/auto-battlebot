"""Fine-tune a YOLO checkpoint on a dataset."""

import argparse
import os
from datetime import datetime

from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    configs = {
        "yolo26n-pose": {
            "batch": 64,
            "epochs": 150,
            "imgsz": 640,
        },
        "yolo26n": {
            "batch": 128,
            "epochs": 50,
            "imgsz": 640,
        },
    }

    parser = argparse.ArgumentParser(
        description=(
            "Fine-tune a YOLO pose model from an existing checkpoint with a reduced learning rate."
        ),
    )
    parser.add_argument(
        "dataset",
        type=str,
        help="Path to dataset yaml",
    )
    parser.add_argument(
        "models",
        nargs="+",
        type=str,
        help="Model key to train. ex: yolo26n-pose",
    )
    parser.add_argument(
        "-c",
        "--checkpoint",
        required=True,
        type=str,
        help="Path to .pt checkpoint to fine-tune from",
    )
    parser.add_argument(
        "-e",
        "--epochs",
        default=0,
        type=int,
        help="Overwrite number of epochs",
    )
    parser.add_argument(
        "--lr0",
        default=0.001,
        type=float,
        help="Initial learning rate (default: 0.001, 10x lower than from-scratch)",
    )
    parser.add_argument(
        "--devices",
        default="0,1,2",
        type=str,
        help="Comma-separated GPU device IDs (default: 0,1,2)",
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
        help="Fine-tune on the first FRACTION of the shuffled train split (default 1.0).",
    )
    args = parser.parse_args()

    dataset = args.dataset
    models = args.models
    epochs = args.epochs
    checkpoint_path = args.checkpoint
    lr0 = args.lr0
    devices = tuple(int(d) for d in args.devices.split(","))
    save_period = args.save_period
    fraction = args.fraction

    session_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    hyper_params = dict(
        lr0=lr0,
        lrf=0.01,
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=3.0,
        warmup_momentum=0.8,
        warmup_bias_lr=0.01,
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
        degrees=180.0,
        translate=0.5,
        scale=0.5,
        shear=10.0,
        perspective=0.001,
        flipud=0.5,
        fliplr=0.5,
        bgr=0.0,
        mosaic=0.4,
        mixup=0.1,
        copy_paste=0.2,
        copy_paste_mode="flip",
        auto_augment="randaugment",
        erasing=0.4,
        crop_fraction=0.2,
    )

    for model_key in models:
        settings = configs[model_key]
        if epochs > 0:
            settings["epochs"] = epochs

        model = YOLO(checkpoint_path)

        run_name = f"auto_battlebots_keypoints_finetune_{session_date}_{model_key}"
        model.train(
            data=dataset,
            name=run_name,
            project="../projects",
            device=devices,
            workers=24,
            save_period=save_period,
            fraction=fraction,
            **hyper_params,
            **settings,
        )


if __name__ == "__main__":
    main()
