"""
Example usage:
python fine_tune_train.py /path/to/merged/data.yaml yolo26n-pose \
    -c ../../models/yolo26n-pose_mr_stabs_mk2_2026-03-08.pt \
    -e 200 --lr0 0.002
"""

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
    }

    parser = argparse.ArgumentParser(
        description="Fine-tune a YOLO pose model from an existing checkpoint with a reduced learning rate.",
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
    args = parser.parse_args()

    dataset = args.dataset
    models = args.models
    epochs = args.epochs
    checkpoint_path = args.checkpoint
    lr0 = args.lr0
    devices = tuple(int(d) for d in args.devices.split(","))

    session_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    hyper_params = dict(
        lr0=lr0,
        lrf=0.01,  # (float) final learning rate (lr0 * lrf)
        momentum=0.937,  # (float) SGD momentum/Adam beta1
        weight_decay=0.0005,  # (float) optimizer weight decay 5e-4
        warmup_epochs=3.0,  # (float) warmup epochs (fractions ok)
        warmup_momentum=0.8,  # (float) warmup initial momentum
        warmup_bias_lr=0.01,  # (float) warmup initial bias lr (lowered for fine-tuning)
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
            **hyper_params,
            **settings,
        )


if __name__ == "__main__":
    main()
