import argparse
import csv
import json
import random as pyrandom
import warnings
from contextlib import nullcontext
from datetime import datetime
from pathlib import Path
from typing import Any, ContextManager, Generator

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
from constants import IMAGE_SIZE, NUM_CLASSES, PAD_SIZE
from livelossplot import PlotLosses
from livelossplot.outputs.matplotlib_plot import MatplotlibPlot
from load_deeplabv3 import (
    VALID_DECODERS,
    build_model,
    common_transforms,
    seed_everything,
)
from model_config import ModelConfig, load_model_config, save_model_config
from PIL import Image
from torch.nn import functional
from torch.utils.data import DataLoader, Dataset
from torchmetrics import MeanMetric
from torchvision.transforms import functional as tv_functional
from tqdm import tqdm

matplotlib.use("agg")

# Field-mask class layout: channel 0 is background, channel 1 is the field.
FIELD_CLASS = 1
FIELD_INDEX_NAME = "field_index.json"
AMP_DTYPES = {"off": None, "bf16": torch.bfloat16}


def get_default_device() -> torch.device:
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def seed_worker(_worker_id: int) -> None:
    """Give every dataloader worker its own augmentation stream.

    SegDataset._augment draws from Python's global `random`, which PyTorch does not
    reseed per worker -- forked workers would otherwise share one sequence and apply
    correlated augmentation. That understates run-to-run variance, and Phase A's whole
    job is to measure run-to-run variance.
    """
    seed = torch.initial_seed() % (2**32)
    pyrandom.seed(seed)
    np.random.seed(seed)


class SegDataset(Dataset):
    def __init__(
        self,
        *,
        img_paths: list[Path],
        mask_paths: list[Path],
        image_size: tuple[int, int] = (IMAGE_SIZE, IMAGE_SIZE),
        pad_size: int = PAD_SIZE,
        data_type: str = "train",
    ) -> None:
        self.data_type = data_type
        self.img_paths = img_paths
        self.mask_paths = mask_paths
        self.image_size = image_size
        self.pad_size = pad_size

        self.transforms = common_transforms(pad_size=pad_size)

    def read_file(self, path: Path) -> np.ndarray:
        image = cv2.imread(str(path))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, self.image_size, interpolation=cv2.INTER_NEAREST)
        return np.array(image)

    def __len__(self) -> int:
        return len(self.img_paths)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        image_path = self.img_paths[index]
        image = self.read_file(image_path)

        mask_path = self.mask_paths[index]
        gt_mask = self.read_file(mask_path).astype(np.int32)

        image_pil = Image.fromarray(image)
        mask_pil = Image.fromarray(gt_mask[:, :, 0].astype(np.uint8))

        if self.data_type == "train":
            image_pil, mask_pil = self._augment(image_pil, mask_pil)

        image_tensor = self.transforms(np.array(image_pil))

        gt_arr = np.array(mask_pil)
        mask = np.zeros((*self.image_size, 2), dtype=np.float32)
        mask[:, :, 0] = np.where(gt_arr == 0, 1.0, 0.0)
        mask[:, :, 1] = np.where(gt_arr > 0, 1.0, 0.0)

        mask_bg = np.pad(
            mask[:, :, 0:1],
            ((self.pad_size, self.pad_size), (self.pad_size, self.pad_size), (0, 0)),
            mode="constant",
            constant_values=1,
        )
        mask_fg = np.pad(
            mask[:, :, 1:],
            ((self.pad_size, self.pad_size), (self.pad_size, self.pad_size), (0, 0)),
            mode="constant",
            constant_values=0,
        )
        mask = np.concatenate([mask_bg, mask_fg], axis=2)

        reordered_mask = torch.from_numpy(mask).permute(2, 0, 1)

        return image_tensor, reordered_mask

    @staticmethod
    def _augment(image: Image.Image, mask: Image.Image) -> tuple[Image.Image, Image.Image]:
        # Photometric (image only)
        image = tv_functional.adjust_brightness(image, pyrandom.uniform(0.7, 1.3))
        image = tv_functional.adjust_contrast(image, pyrandom.uniform(0.7, 1.3))
        image = tv_functional.adjust_saturation(image, pyrandom.uniform(0.7, 1.3))
        image = tv_functional.adjust_hue(image, pyrandom.uniform(-0.05, 0.05))
        if pyrandom.random() < 0.3:
            image = tv_functional.gaussian_blur(image, kernel_size=5)

        # Geometric (image + mask together)
        if pyrandom.random() < 0.5:
            image = tv_functional.hflip(image)
            mask = tv_functional.hflip(mask)

        if pyrandom.random() < 0.5:
            image = tv_functional.vflip(image)
            mask = tv_functional.vflip(mask)

        return image, mask


def pair_masks(image_paths: list[Path]) -> tuple[list[Path], list[Path]]:
    """Keep only images that have a `_mask.png` beside them."""
    filtered_image_paths = []
    annotation_paths = []
    for path in image_paths:
        anno_name = path.with_name(f"{path.stem}_mask.png")
        if not anno_name.exists():
            warnings.warn(f"Missing mask for image {path}")
            continue
        annotation_paths.append(anno_name)
        filtered_image_paths.append(path)
    return filtered_image_paths, annotation_paths


def get_training_set_paths(data_directory: Path) -> tuple[list[Path], list[Path]]:
    return pair_masks(sorted(p for p in data_directory.iterdir() if p.suffix == ".jpg"))


def paths_from_list(list_path: Path, dataset_root: Path) -> tuple[list[Path], list[Path]]:
    """Read an arm's image list. Relative entries resolve against the corpus root.

    Arms are lists rather than directories of copies because the Phase-B ladder is 18
    arms over ~29 k images; physical copies would be ~350 GB against 282 GB free.
    """
    if not list_path.exists():
        raise SystemExit(f"Image list not found: {list_path}")
    paths = []
    for line in list_path.read_text().split():
        path = Path(line)
        paths.append(path if path.is_absolute() else dataset_root / path)
    if not paths:
        raise SystemExit(f"Image list is empty: {list_path}")
    return pair_masks(paths)


def load_field_index(dataset_root: Path, override: Path | None) -> dict[str, str]:
    """Image name -> field type, written by make_field_splits.py.

    Absent, training still works but model selection falls back to the global mean IoU,
    which on this corpus is roughly a Cage-6 score.
    """
    path = override or (dataset_root / FIELD_INDEX_NAME)
    if not path.exists():
        return {}
    index = json.loads(path.read_text())
    return {name: entry["field"] for name, entry in index.items()}


def get_dataset(
    data_directory: Path,
    batch_size: int,
    num_workers: int,
    image_size: int = IMAGE_SIZE,
    pad_size: int = PAD_SIZE,
    train_list: Path | None = None,
    val_list: Path | None = None,
) -> tuple[DataLoader, DataLoader]:
    if train_list is not None:
        train_img_paths, train_msk_paths = paths_from_list(train_list, data_directory)
    else:
        train_img_paths, train_msk_paths = get_training_set_paths(data_directory / "train")
    if val_list is not None:
        valid_img_paths, valid_msk_paths = paths_from_list(val_list, data_directory)
    else:
        valid_img_paths, valid_msk_paths = get_training_set_paths(data_directory / "val")

    size_tuple = (image_size, image_size)
    train_ds = SegDataset(
        img_paths=train_img_paths,
        mask_paths=train_msk_paths,
        image_size=size_tuple,
        pad_size=pad_size,
        data_type="train",
    )
    valid_ds = SegDataset(
        img_paths=valid_img_paths,
        mask_paths=valid_msk_paths,
        image_size=size_tuple,
        pad_size=pad_size,
        data_type="valid",
    )

    train_loader = DataLoader(
        train_ds,
        batch_size=batch_size,
        num_workers=num_workers,
        shuffle=True,
        pin_memory=True,
        worker_init_fn=seed_worker,
    )
    # Validation must not shuffle: FieldIoU pairs each sample with its field by position.
    valid_loader = DataLoader(
        valid_ds,
        batch_size=batch_size,
        num_workers=num_workers,
        shuffle=False,
        pin_memory=True,
    )

    return train_loader, valid_loader


def intermediate_metric_calculation(
    predictions: torch.Tensor,
    targets: torch.Tensor,
    use_dice: bool = False,
    smooth: float = 1e-6,
    dims: tuple[int, int] = (2, 3),
) -> torch.Tensor:
    # dims corresponding to image height and width: [B, C, H, W].

    # Intersection: |G ∩ P|. Shape: (batch_size, num_classes)
    intersection = (predictions * targets).sum(dim=dims)

    # Summation: |G| + |P|. Shape: (batch_size, num_classes).
    summation = predictions.sum(dim=dims) + targets.sum(dim=dims)

    if use_dice:
        # Dice Shape: (batch_size, num_classes)
        metric = (2.0 * intersection + smooth) / (summation + smooth)
    else:
        # Union. Shape: (batch_size, num_classes)
        union = summation - intersection

        # IoU Shape: (batch_size, num_classes)
        metric = (intersection + smooth) / (union + smooth)

    # Compute the mean over the remaining axes (batch and classes).
    # Shape: Scalar
    total = metric.mean()

    return total


class Loss(nn.Module):
    def __init__(self, smooth: float = 1e-6, use_dice: bool = False) -> None:
        super().__init__()
        self.smooth = smooth
        self.use_dice = use_dice

    def forward(self, predictions: torch.Tensor, targets: torch.Tensor) -> Any:
        # predictions --> (B, #C, H, W) unnormalized (logits)
        # targets     --> (B, #C, H, W) one-hot encoded

        # Numerically stable BCE that fuses sigmoid + log internally
        pixel_loss = functional.binary_cross_entropy_with_logits(
            predictions, targets, reduction="mean"
        )

        probs = torch.sigmoid(predictions)
        mask_loss = 1 - intermediate_metric_calculation(
            probs, targets, use_dice=self.use_dice, smooth=self.smooth
        )
        total_loss = mask_loss + pixel_loss

        return total_loss


def convert_2_onehot(matrix: torch.Tensor, num_classes: int = 3) -> torch.Tensor:
    """
    Perform one-hot encoding across the channel dimension.
    """
    matrix = matrix.permute(0, 2, 3, 1)
    matrix = torch.argmax(matrix, dim=-1)
    matrix = functional.one_hot(matrix, num_classes=num_classes)
    matrix = matrix.permute(0, 3, 1, 2)

    return matrix


class Metric(nn.Module):
    def __init__(self, num_classes: int = 3, smooth: float = 1e-6, use_dice: bool = False):
        super().__init__()
        self.num_classes = num_classes
        self.smooth = smooth
        self.use_dice = use_dice

    def forward(self, predictions: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        # predictions  --> (B, #C, H, W) unnormalized
        # targets      --> (B, #C, H, W) one-hot encoded

        # Converting unnormalized predictions into one-hot encoded across channels.
        # Shape: (B, #C, H, W)
        predictions = convert_2_onehot(predictions, num_classes=self.num_classes)  # one hot encoded

        metric = intermediate_metric_calculation(
            predictions, targets, use_dice=self.use_dice, smooth=self.smooth
        )

        # Compute the mean over the remaining axes (batch and classes). Shape: Scalar
        return metric


class FieldIoU:
    """Per-field validation IoU, accumulated in dataset order.

    Cage-6 is a third of the corpus, so the global mean this script has always selected
    on is roughly a Cage-6 score -- the checkpoint that maximises it can be regressing
    Cage-3 and Cage-4 the whole way. Selecting on the macro-average across field types
    gives every arena one vote regardless of how much footage of it exists.

    The pad border is cropped before scoring so this matches score_masks.py, which grades
    the checkpoints afterwards; selecting on one definition and grading on another would
    make the Phase-A ladder disagree with its own stopping rule.
    """

    def __init__(self, fields: list[str], pad_size: int) -> None:
        self.fields = fields
        self.pad_size = pad_size
        self.intersection = np.zeros(len(fields))
        self.union = np.zeros(len(fields))
        self.position = 0

    def reset(self) -> None:
        self.intersection[:] = 0.0
        self.union[:] = 0.0
        self.position = 0

    def update(self, predictions: torch.Tensor, targets: torch.Tensor) -> None:
        pad = self.pad_size
        predicted = torch.argmax(predictions, dim=1) == FIELD_CLASS
        truth = targets[:, FIELD_CLASS] > 0.5
        if pad:
            predicted = predicted[:, pad:-pad, pad:-pad]
            truth = truth[:, pad:-pad, pad:-pad]
        intersection = (predicted & truth).sum(dim=(1, 2)).double().cpu().numpy()
        union = (predicted | truth).sum(dim=(1, 2)).double().cpu().numpy()

        count = len(intersection)
        window = slice(self.position, self.position + count)
        self.intersection[window] += intersection
        self.union[window] += union
        self.position += count

    def summary(self) -> dict[str, float]:
        """Global / macro / worst-field IoU, plus one entry per field."""
        by_field: dict[str, float] = {}
        for field in sorted(set(self.fields)):
            selected = np.asarray(self.fields) == field
            union = self.union[selected].sum()
            by_field[field] = float(self.intersection[selected].sum() / union) if union else 0.0

        total_union = self.union.sum()
        values = list(by_field.values())
        return {
            "global": float(self.intersection.sum() / total_union) if total_union else 0.0,
            "macro": float(np.mean(values)) if values else 0.0,
            "worst": float(np.min(values)) if values else 0.0,
            **{f"iou/{name}": value for name, value in by_field.items()},
        }


def to_device(data: Any, device: torch.device) -> Any:
    """Move tensor(s) to chosen device"""
    if isinstance(data, (list, tuple)):
        return [to_device(x, device) for x in data]
    return data.to(device, non_blocking=True)


class DeviceDataLoader:
    """Wrap a dataloader to move data to a device"""

    def __init__(self, dl: Any, device: torch.device) -> None:
        self.dl = dl
        self.device = device

    def __iter__(self) -> Generator[Any, None, None]:
        """Yield a batch of data after moving it to device"""
        for b in self.dl:
            yield to_device(b, self.device)

    def __len__(self) -> int:
        """Number of batches"""
        return len(self.dl)


class TrainingStepInterface:
    def __init__(self, model: nn.Module, amp_dtype: torch.dtype | None = None) -> None:
        self.model = model
        self.amp_dtype = amp_dtype

    def autocast(self) -> ContextManager[Any]:
        """bf16 autocast is ~1.67x on an A6000 at this resolution and needs no GradScaler.

        Verify it is loss-neutral before spending a full budget on it -- run the same
        seed both ways for 20 epochs and compare (Phase A's amp_check arms).
        """
        if self.amp_dtype is None:
            return nullcontext()
        return torch.autocast(device_type="cuda", dtype=self.amp_dtype)

    def step(self, data: Any) -> tuple[torch.Tensor, torch.Tensor]:
        raise NotImplementedError


class TrainStep(TrainingStepInterface):
    def __init__(
        self,
        model: nn.Module,
        optimizer_fn: torch.optim.Optimizer,
        loss_fn: nn.Module,
        amp_dtype: torch.dtype | None = None,
    ) -> None:
        super().__init__(model, amp_dtype)
        self.optimizer_fn = optimizer_fn
        self.loss_fn = loss_fn

    def step(self, data: Any) -> tuple[torch.Tensor, torch.Tensor]:
        with self.autocast():
            preds = self.model(data[0])
            loss = self.loss_fn(preds, data[1])
        self.optimizer_fn.zero_grad(set_to_none=True)
        loss.backward()
        self.optimizer_fn.step()
        return preds, loss


class ValidationStep(TrainingStepInterface):
    def __init__(
        self, model: nn.Module, loss_fn: nn.Module, amp_dtype: torch.dtype | None = None
    ) -> None:
        super().__init__(model, amp_dtype)
        self.loss_fn = loss_fn

    def step(self, data: Any) -> tuple[torch.Tensor, torch.Tensor]:
        with torch.no_grad(), self.autocast():
            preds = self.model(data[0]).detach()
            loss = self.loss_fn(preds, data[1])
        return preds.float(), loss.float()


def step(
    epoch_num: int,
    loader: DeviceDataLoader,
    step_interface: TrainingStepInterface,
    metric_fn: nn.Module,
    step_name: str,
    field_iou: FieldIoU | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    loss_record = MeanMetric()
    metric_record = MeanMetric()

    loader_len = len(loader)
    if field_iou is not None:
        field_iou.reset()

    for data in tqdm(
        iterable=loader,
        total=loader_len,
        dynamic_ncols=True,
        desc=f"{step_name} :: Epoch: {epoch_num}",
    ):
        preds, loss = step_interface.step(data)

        metric = metric_fn(preds.detach(), data[1])
        if field_iou is not None:
            field_iou.update(preds.detach(), data[1])

        loss_value = loss.detach().item()
        metric_value = metric.detach().item()

        loss_record.update(loss_value)
        metric_record.update(metric_value)

    current_loss = loss_record.compute()
    current_metric = metric_record.compute()

    return current_loss, current_metric


def log_epoch(
    metrics_file: Any,
    writer: "csv.DictWriter | None",
    epoch: int,
    losses: tuple[Any, Any, Any, Any],
    field_summary: dict[str, float],
) -> "csv.DictWriter":
    """Append one row to metrics.csv, writing the header on the first call.

    score_masks.py grades the checkpoint ladder properly afterwards; this row is what
    keeps an interrupted or ungraded run readable.
    """
    train_loss, train_metric, valid_loss, valid_metric = losses
    row = {
        "epoch": epoch,
        "train_loss": float(train_loss),
        "train_iou": float(train_metric),
        "val_loss": float(valid_loss),
        "val_iou_mean": float(valid_metric),
        **{f"val_{key}": value for key, value in field_summary.items()},
    }
    if writer is None:
        writer = csv.DictWriter(metrics_file, fieldnames=list(row))
        writer.writeheader()
    writer.writerow(row)
    metrics_file.flush()

    if field_summary:
        print(
            f"  val IoU  macro {field_summary['macro']:0.4f}  "
            f"worst {field_summary['worst']:0.4f}  global {field_summary['global']:0.4f}"
        )
    return writer


def resolve_selection(
    dataset_root: Path,
    field_index_override: Path | None,
    requested_metric: str,
    valid_dataset: SegDataset,
    pad_size: int,
) -> tuple[FieldIoU | None, str]:
    """Set up per-field validation IoU, or fall back to the global mean without it."""
    field_of = load_field_index(dataset_root, field_index_override)
    if not field_of:
        if requested_metric != "global":
            print(
                f"Warning: no {FIELD_INDEX_NAME}; falling back to --select-metric global. "
                "On this corpus the global mean is roughly a Cage-6 score."
            )
        return None, "global"

    valid_fields = [field_of.get(p.name, "unknown") for p in valid_dataset.img_paths]
    counts = {f: valid_fields.count(f) for f in sorted(set(valid_fields))}
    print(f"validation fields ({len(counts)}): {counts}")
    return FieldIoU(valid_fields, pad_size), requested_metric


def save_augmentation_samples(
    dataset: SegDataset,
    output_dir: Path,
    num_samples: int = 20,
) -> None:
    samples_dir = output_dir / "augmentation_samples"
    samples_dir.mkdir(parents=True, exist_ok=True)

    indices = pyrandom.sample(range(len(dataset)), min(num_samples, len(dataset)))

    for i, idx in enumerate(indices):
        image = dataset.read_file(dataset.img_paths[idx])
        gt_mask = dataset.read_file(dataset.mask_paths[idx]).astype(np.int32)

        image_pil = Image.fromarray(image)
        mask_pil = Image.fromarray(gt_mask[:, :, 0].astype(np.uint8))
        image_aug, mask_aug = SegDataset._augment(image_pil, mask_pil)

        mask_arr = np.array(mask_aug)
        mask_display = np.zeros((*mask_arr.shape, 3), dtype=np.uint8)
        mask_display[mask_arr > 0] = [255, 0, 0]

        fig, axes = plt.subplots(1, 3, figsize=(12, 4))
        axes[0].imshow(image_pil)
        axes[0].set_title("Original")
        axes[0].axis("off")
        axes[1].imshow(image_aug)
        axes[1].set_title("Augmented")
        axes[1].axis("off")
        axes[2].imshow(image_aug)
        axes[2].imshow(mask_display, alpha=0.45)
        axes[2].set_title("Mask Overlay")
        axes[2].axis("off")
        fig.suptitle(dataset.img_paths[idx].name, fontsize=8)
        fig.tight_layout()
        fig.savefig(samples_dir / f"sample_{i:02d}.png", dpi=100)
        plt.close(fig)

    print(f"Saved {len(indices)} augmentation samples to {samples_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a semantic segmentation model")
    parser.add_argument(
        "dataset_location",
        type=str,
        help="Path to the directory containing the images and annotations",
    )
    parser.add_argument(
        "-c",
        "--checkpoint",
        type=str,
        default=None,
        help="Path to the checkpoint file",
    )
    parser.add_argument(
        "-b",
        "--batch-size",
        type=int,
        default=8,
        help="Batch size",
    )
    parser.add_argument(
        "-nw",
        "--num-workers",
        type=int,
        default=4,
        help="Number of workers",
    )
    parser.add_argument(
        "-ne",
        "--num-epochs",
        type=int,
        default=500,
        help="Number of training epochs",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="",
        help="Output directory for the model and logs",
    )
    parser.add_argument(
        "--backbone",
        type=str,
        default="r50",
        choices=["mbv3", "r50", "r101"],
        help="Backbone architecture (default: r50)",
    )
    parser.add_argument(
        "--image-size",
        type=int,
        default=IMAGE_SIZE,
        help=f"Training image size before padding (default: {IMAGE_SIZE})",
    )
    parser.add_argument(
        "--pad-size",
        type=int,
        default=PAD_SIZE,
        help=f"Border padding size (default: {PAD_SIZE})",
    )
    parser.add_argument(
        "--decoder",
        type=str,
        default="v3",
        choices=list(VALID_DECODERS),
        help="Decoder architecture: v3 (torchvision) or v3plus (SMP) (default: v3)",
    )
    parser.add_argument(
        "--train-list",
        type=Path,
        default=None,
        help="image list to train on instead of <dataset>/train (an arm from make_field_splits.py)",
    )
    parser.add_argument(
        "--val-list",
        type=Path,
        default=None,
        help="image list to validate on instead of <dataset>/val",
    )
    parser.add_argument(
        "--save-period",
        type=int,
        default=0,
        help="also save ckpt_ep<N>.pth every N epochs (0 = best checkpoint only). "
        "A ladder is required to answer when to stop training",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=4176,
        help="RNG seed. Note that this does NOT make a run reproducible: seed_everything "
        "leaves cudnn.benchmark on, so kernel selection varies run to run and two runs at "
        "the same seed diverge. A seed pair therefore measures seed variation plus that "
        "nondeterminism, which together are the real run-to-run noise floor",
    )
    parser.add_argument(
        "--amp",
        type=str,
        default="off",
        choices=list(AMP_DTYPES),
        help="autocast precision (default: off). bf16 is ~1.67x on an A6000",
    )
    parser.add_argument(
        "--select-metric",
        type=str,
        default="macro",
        choices=("macro", "worst", "global"),
        help="which validation IoU picks the best checkpoint (default: macro-average "
        "across field types; global is ~a Cage-6 score on this corpus)",
    )
    parser.add_argument(
        "--field-index",
        type=Path,
        default=None,
        help=f"override the {FIELD_INDEX_NAME} used for per-field validation IoU",
    )
    assert torch.cuda.is_available(), "CUDA is not available"
    assert torch.cuda.device_count() > 0, "No CUDA devices available"

    args = parser.parse_args()
    seed = args.seed
    seed_everything(seed)
    pyrandom.seed(seed)

    dataset_location = Path(args.dataset_location)
    checkpoint_path = Path(args.checkpoint) if args.checkpoint else None
    if args.output:
        output = Path(args.output)
    else:
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output = dataset_location.parent / f"output_{date_str}"
    batch_size = args.batch_size
    num_workers = args.num_workers
    num_epochs = args.num_epochs
    num_classes = NUM_CLASSES
    backbone_model_name = args.backbone
    image_size = args.image_size
    pad_size = args.pad_size
    decoder = args.decoder

    output.mkdir(parents=True, exist_ok=True)

    output_model = output / f"model_{backbone_model_name}.pth"
    fig_path = output / "plot.png"

    device = get_default_device()

    model = build_model(
        backbone=backbone_model_name,
        num_classes=num_classes,
        device=device,
        decoder=decoder,
    )
    if checkpoint_path is not None:
        checkpoint_cfg = load_model_config(checkpoint_path)
        assert checkpoint_cfg.backbone == backbone_model_name, (
            f"Model backbone mismatch. {checkpoint_cfg.backbone} != {backbone_model_name}"
        )
        assert checkpoint_cfg.decoder == decoder, (
            f"Model decoder mismatch. {checkpoint_cfg.decoder} != {decoder}"
        )
        checkpoints = torch.load(checkpoint_path, map_location=device)
        model.model.load_state_dict(checkpoints, strict=False)
        model.eval()

    _ = model(
        torch.randn((2, 3, image_size + 2 * pad_size, image_size + 2 * pad_size), device=device)
    )

    train_loader, valid_loader = get_dataset(
        data_directory=dataset_location,
        batch_size=batch_size,
        num_workers=num_workers,
        image_size=image_size,
        pad_size=pad_size,
        train_list=args.train_list,
        val_list=args.val_list,
    )
    print(
        f"train: {len(train_loader.dataset):,} images"
        f"{f' from {args.train_list}' if args.train_list else ''}"
    )
    print(f"valid: {len(valid_loader.dataset):,} images")

    field_iou, select_metric = resolve_selection(
        dataset_location, args.field_index, args.select_metric, valid_loader.dataset, pad_size
    )

    for i, j in valid_loader:
        print(
            f"Image shape: {i.shape}, Image type: {i.dtype},"
            f" Mask shape: {j.shape}, Mask type: {j.dtype}"
        )
        break

    save_augmentation_samples(train_loader.dataset, output)

    train_device_loader = DeviceDataLoader(train_loader, device)
    valid_device_loader = DeviceDataLoader(valid_loader, device)

    metric_name = "iou"
    use_dice = True if metric_name == "dice" else False

    metric_fn = Metric(num_classes=num_classes, use_dice=use_dice).to(device)
    loss_fn = Loss(use_dice=use_dice).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)

    liveloss = PlotLosses(
        outputs=[MatplotlibPlot(figpath=str(fig_path)), "ExtremaPrinter"],
        mode="script",
    )

    model_config = ModelConfig(
        backbone=backbone_model_name,
        image_size=image_size,
        pad_size=pad_size,
        num_classes=num_classes,
        decoder=decoder,
    )

    def save_checkpoint(path: Path) -> None:
        torch.save(model.model.state_dict(), path)
        save_model_config(path, model_config)

    amp_dtype = AMP_DTYPES[args.amp]
    best_metric = 0.0
    train_step = TrainStep(model, optimizer, loss_fn, amp_dtype)
    valid_step = ValidationStep(model, loss_fn, amp_dtype)

    metrics_path = output / "metrics.csv"
    metrics_file = metrics_path.open("w", newline="")
    metrics_writer: csv.DictWriter | None = None

    print(
        f"seed {seed} | amp {args.amp} | selecting on val {select_metric} IoU"
        f"{f' | ladder every {args.save_period} epochs' if args.save_period else ''}"
    )

    for epoch in range(1, num_epochs + 1):
        model.train()
        train_loss, train_metric = step(
            epoch_num=epoch,
            loader=train_device_loader,
            step_interface=train_step,
            metric_fn=metric_fn,
            step_name="train",
        )

        model.eval()
        valid_loss, valid_metric = step(
            epoch_num=epoch,
            loader=valid_device_loader,
            step_interface=valid_step,
            metric_fn=metric_fn,
            step_name="valid",
            field_iou=field_iou,
        )

        liveloss.update(
            {
                "loss": train_loss,
                metric_name: train_metric,
                "val_loss": valid_loss,
                f"val_{metric_name}": valid_metric,
            }
        )
        liveloss.send()
        plt.close("all")

        field_summary = field_iou.summary() if field_iou is not None else {}
        metrics_writer = log_epoch(
            metrics_file,
            metrics_writer,
            epoch,
            (train_loss, train_metric, valid_loss, valid_metric),
            field_summary,
        )

        if args.save_period and epoch % args.save_period == 0:
            save_checkpoint(output / f"ckpt_ep{epoch:03d}.pth")

        # Without a field index there is no per-field summary, so this falls back to the
        # legacy class-mean IoU rather than silently selecting on nothing.
        selected = field_summary.get(select_metric, float(valid_metric))
        if np.isfinite(selected) and selected >= best_metric:
            print(f"Saving model. {select_metric} {selected:0.4f} >= {best_metric:0.4f}")
            save_checkpoint(output_model)
            best_metric = selected

    metrics_file.close()
    print(f"Best val {select_metric} IoU: {best_metric:0.4f}")
    print(f"Wrote {output_model} and {metrics_path}")


if __name__ == "__main__":
    main()
