# YOLO training data tools

Design facts for the dataset and cache tools under `training/yolo/`.

## Disk image caches

Training with `cache="disk"` writes one `.npy` per image next to it. Ultralytics regenerates them and never prunes them, but they dwarf the images. On megamind, 297 GB of `.npy` sat next to 24 GB of unique data and filled the NVMe to 97 %. One corpus grew to 196 GB of cache against 12 GB of images.

Deleting a cache is safe. The only cost of a wrong delete is a few minutes of IO to re-cache on the next run. So both tools skip caches that were used recently.

### `clear_image_cache.py`

- Age is measured from access time by default, not modification time. A cache is written once and read by every later run, so mtime says when the dataset was first trained on. A cache generated months ago and used daily looks ancient by mtime. atime says when it was last read.
- The root filesystem is mounted `relatime`, which updates atime on read once a day, so day-granularity ages are accurate. On a `noatime` mount atime is frozen, and the tool says so instead of deleting a hot cache.
- Datasets referenced by a running process are never touched, whatever their age.

```bash
python training/yolo/clear_image_cache.py --dry-run          # default 7-day floor
python training/yolo/clear_image_cache.py --older-than 14    # delete caches unused 14+ days
python training/yolo/clear_image_cache.py --older-than 0     # everything not in use
```

### `reclaim_stale_caches()` in `training/yolo/train.py`

Runs before training starts and deletes caches unused for `max_age_days`.

- The dataset about to be trained on is protected explicitly. `busy_datasets` skips this process and its ancestors, so a shell that merely mentions a path does not pin it. That also means it cannot see that this process is about to use the data.
- It is never fatal. A failure here must not take down a multi-hour training run.

## Merging scenes by class name

`training/yolo/remap_labels_by_name.py` merges per-scene YOLO datasets into one flat dataset, remapping classes by name.

`remap_labels.py` applies one source-index to target-index map. That fails when scenes do not share a vocabulary, as in the archived `nhrl_seg_indiv_labeled`: 50 scenes declare 105 classes and 6 declare 111, with shifted indices (`Floor` 36 vs 39, `House Bot` 41 vs 44, `Mrs Buff MK2` 67 vs 70). One index map there produces a silently mislabelled corpus, the failure that left `Mrs Buff MK2` labelled as our own robot for months.

- It reads each scene's own `data.yml`, maps source class names to the target vocabulary, drops the names told to drop, and writes one flat dataset.
- Images are copied, not linked, so the output survives the source being archived or moved to another filesystem.
- A name with no explicit rule falls through to the config's `default` target and is printed, so a new non-robot class upstream is visible.
- `--to-bbox` converts polygons to axis-aligned boxes in the same pass with `seg_to_bbox.polygon_to_bbox`, which avoids a full intermediate copy of the corpus.

```bash
python training/yolo/remap_labels_by_name.py \
    --src /media/storage/auto-battlebots-archive/nhrl_seg_indiv_labeled \
    --config training/yolo/remap_config_3class.toml \
    --out training/data/nhrl_robots_bbox_3class --to-bbox
```

### Review verdicts

`passing_frames()` returns the image stems marked `pass` in a scene's `validation_state.json`, or `None` for an untracked scene. These datasets carry a per-frame human verdict from `validate_yolo_dataset.py`. Ignoring it pulls in frames a person already rejected: 1,680 of them across the 56 archived scenes, 4.7 % of the corpus.

## Segmentation to bbox conversion

`training/yolo/seg_to_bbox.py` converts a YOLO segmentation dataset into a detection dataset.

- Class ids and the train/val/test split structure are preserved exactly.
- A polygon row `class x1 y1 x2 y2 ...` becomes a box row `class cx cy w h` from the polygon's axis-aligned bounds. Rows already in box form (4 coordinates) pass through.
- Images are hardlinked when the output is on the same filesystem, otherwise copied.
- It was built for the seg-vs-bbox comparison: identical images, classes, and splits, box labels only, so any box-AP difference comes from the head and nothing else.

```bash
python training/yolo/seg_to_bbox.py training/data/nhrl_seg/nhrl_robots -o training/data/nhrl_robots_bbox
```

### Multi-contour rows

`split_subcontours()` splits one label row's point list where it jumps to a disjoint contour. A single YOLO-seg row can hold several contours concatenated end to end. In this corpus 23.6 % of sampled polygons contain a step larger than 3 % of the frame, against a median step of 0.21 % and a p99 of 1.8 %. Handing the whole path to `fillPoly` fills the bridge between blobs, which turned one Bee-Roll row into a box covering 26 % of the frame.

- `SUBCONTOUR_JUMP = 0.03` is about the p99.7 of consecutive-point steps.
- Over-splitting is safe. The pieces rasterise into the same mask, so parts of a contiguous shape touch and merge back into one contour. Only truly disjoint blobs stay separate.
- `TOP_CONTOURS = 3` in `top_contours_bbox()` keeps the largest parts, because a robot's mask commonly splits into a few real parts.
- One box per label encloses its `top_n` largest contours. Min/max over every point is wrong for two reasons: a bridged row gives a box far larger than the robot, and a shattered mask emits one row per fragment, which turned one robot into about 270 separate targets scattered across the crowd while the robot itself sat unboxed. Rasterising the label into one mask and keeping only the biggest contour clips the other parts out of the box. Enclosing the three largest recovers the parts and still ignores the long tail of speckle.
- The trade-off is real. When a label's second contour is genuinely distant (debris across the arena, a detached blade), enclosing it re-inflates the box. `top_n` is a parameter so the balance can be retuned without touching callers.

## Scaling split construction

`training/yolo/make_scaling_splits.py` builds the data-scaling arms.

- `choose_val_scenes()` picks `count` whole scenes whose frame share is near `frac` and whose house_bot share matches the target. It searches random whole-scene combinations instead of taking scenes in order. Scene sizes are so uneven that any fixed rule lands far off the target, and the random search gets within a fraction of a percent on both axes.
- `summarize()` reports frames, scenes and the scene list. Class counts are per scene, so they are exact only for whole-scene arms. Frame counts are reported either way.

## Pooling real and synthetic data

`training/yolo/pool_datasets.py` pools a full real dataset with a fraction of a synthetic dataset into one flat YOLO tree. It was written for the synthetic scene-variation probe of the data_epoch_min experiment (Recipe C in `docs/experiments/perception/experiment_runbook.md`).

- Every real frame is included. Synthetic frames are subsampled by `--synth-fraction`, taken either in filename order or at random (`--synth-order sequential` or `random`).
- Synthetic filenames carry a timestamp and frame index (`...T17-01-18-000050...`), so filename order is render-sequence order. `sequential` is scene-correlated and `random` spreads scene coverage. The gap between the two accuracy-vs-fraction curves at a fixed real count says how much synthetic scene variation the model needs.
- Only the image and its stem-matched `.txt` label are linked. `.npy` and `labels.cache` files next to the images are ultralytics `cache="disk"` artifacts and are skipped. Training regenerates them.
- The output is a flat `images/` and `labels/` tree for `split_yolo_dataset.py`, then `validate_yolo_integrity.py --strict`.
- Pool into the train split only. Build a real-only val split separately and grade on the external eval, never on same-corpus val.

```bash
python pool_datasets.py --real ../data/real_our_robots --synth ../data/all_robot_keypoints/train \
  --out ../data/phaseB_r100_s25_seq --synth-fraction 0.25 --synth-order sequential
```

## Detect-head training defaults

The `yolo26n` entry in `training/yolo/train.py` sets `imgsz` 640, the same as `yolo26n-seg`, so a seg-vs-bbox box-quality comparison varies the head type and not the input size. `batch` differs (128 vs 32), so override it back for a strict comparison.

- Epochs are 100, not 500. Every arm in `data_scaling_2026-07-27.md` peaked on val between epoch 63 and 90 on `nhrl_robots_bbox_2class`, and none was still rising at 100. `category_addition_2026-07-25.md` found fully annealed 150-epoch endpoints failed the eval parity gate. A 500-epoch schedule anneals far past the point where this data generalizes.
- Cache is `ram`, not `disk`. The disk cache writes one full-resolution `.npy` per image, about 200 GB against 12 GB of JPEGs on this corpus (the blowup described under `reclaim_stale_caches()`). The resized RAM cache is about 17 GB per DDP process.
- Batch defaults to 64, safe for a single-GPU run. The size sweep (`model_size_2026-09-04.md`) overrode it to 96 with `--batch`, because 96 splits evenly over 3 GPUs (32 per GPU, measured at 32.4 GB for `yolo26x`) and 64 and 128 do not. On one GPU, `yolo26x` at batch 96 needs about 93 GB, and `yolo26m` and `yolo26l` do not fit either.

## `training/yolo/train.py` configuration

`train.py` sets `MPLBACKEND=Agg` before ultralytics imports matplotlib. Training is headless, and an inherited DISPLAY (for example an SSH-forwarded one) otherwise makes matplotlib pick TkAgg, which ultralytics fails to restore after plotting metrics, and the run crashes.

`resolve_dataset()` accepts a dataset directory and returns the yaml inside it. Ultralytics wants the yaml file: handed a directory it falls through to its archive check and dies with `IsADirectoryError`, several seconds into a job that has already claimed all three GPUs. CLAUDE.md and the experiment plans document the directory form, so `train.py` resolves it instead of every command line changing.

### Per-model entries

| Entry | Meaning |
| --- | --- |
| `yolo26s-pose` | Pose size-sweep arm (`pose_model_size`), between `yolo26n-pose` and `yolo26x-pose`. The sweep passes `--batch 96 --epochs 200` itself, so 64 and 500 are only the single-GPU defaults. Batch 64 fits in one A6000's memory. |
| `yolo26m-pose` | Middle of the pose size ladder (`pose_model_size_ladder_plan_2026-09-24.md`). Copied from `yolo26s-pose`. `run_domain_mix_arm.sh` passes batch, epochs, size and cache itself. |

### Command line flags

| Flag | Meaning |
| --- | --- |
| `--batch` | Total batch across GPUs, as Ultralytics defines it. Lets a model-size sweep hold batch constant without editing the per-model recipes. |
| `--imgsz` | A single int. Ultralytics `check_imgsz` rejects an `[h, w]` pair for training. With `--rect` the long side becomes `IMGSZ` and the short side rounds to stride 32. |
| `--rect` | Sizes each batch to the images' own aspect ratio. On this uniformly 16:9 corpus `--imgsz 640` gives 384x640 and `--imgsz 1024` gives 576x1024, which cuts the 43.8% of a square tensor that is grey padding. |
| `--cache` | `ram` avoids per-epoch disk IO if the resized cache fits in RAM. `disk` reads the full-resolution `.npy` each epoch and starves the GPUs if it exceeds RAM. Beats the model's own setting. |
| `--save-period` | Writes `weights/epoch{0,N,2N,...}.pt` besides `last.pt` and `best.pt`. Turns one long run into the epoch-vs-metric ladder for the `data_epoch_min` experiment. |
| `--fraction` | Subsamples one dataset. It cannot set a real:synthetic ratio, use `pool_datasets.py` for that. |
| `--seed` | Vary it to measure run-to-run variance. `data_epoch_min` Exp 1 found about 0.05 recall, which can exceed the parity margin. |
| `--lrf` | Default 0.1, raised from the Ultralytics 0.01. `category_addition_2026-07-25` found every fully annealed 150-epoch endpoint failed the parity gate while epochs 75 to 100 cleared it, so a less annealed endpoint generalizes better on the robot-camera eval. |
| `--cos-lr` | Untested on this data. Use it to A/B the schedule shape against linear decay. |
| `--close-mosaic` | The Ultralytics default of 10 overlaps the steepest part of the late external-eval decline. Pass 0 to test whether it contributes. |
| `--patience` | Only meaningful once val represents the deployment camera. Training data is NHRL overhead cage footage and the eval is the robot's own ZED, so val fitness keeps improving while eval recall collapses and this never fires (`category_addition_2026-07-25`). |
| `--flipud` | Default 0.0, lowered from 0.5. The deployment ZED has a fixed up-vector and never sees the arena inverted, so flipped batches train on an orientation that cannot occur at inference. |

### Hyperparameters

`hyper_params` mirrors the Ultralytics cfg keys. `lr0`, `lrf`, `cos_lr`, `close_mosaic`, `patience`, `degrees` and `flipud` come from the flags above. The rest are fixed.

| Key | Meaning |
| --- | --- |
| `lr0` | Initial learning rate. Ultralytics suggests 1e-2 for SGD and 1e-3 for Adam. |
| `momentum` | SGD momentum, or Adam beta1. |
| `weight_decay` | Optimizer weight decay, 5e-4. |
| `warmup_epochs` | Fractions allowed. |
| `warmup_momentum` | Momentum at the start of warmup. |
| `warmup_bias_lr` | Bias learning rate at the start of warmup. |
| `box`, `cls`, `dfl`, `pose`, `kobj` | Loss gains for box, class, distribution focal loss, pose and keypoint objectness. `cls` scales with pixels. |
| `label_smoothing` | Fraction. |
| `nbs` | Nominal batch size, the batch the loss is normalized to. |
| `hsv_h`, `hsv_s`, `hsv_v` | HSV hue, saturation and value augmentation, as fractions. |
| `degrees` | Rotation, plus or minus degrees. |
| `translate` | Translation, plus or minus fraction of the image. |
| `scale` | Scale, plus or minus gain. |
| `shear` | Shear, plus or minus degrees. |
| `perspective` | Perspective, plus or minus fraction. Valid range 0 to 0.001. |
| `flipud`, `fliplr` | Vertical and horizontal flip probability. |
| `bgr` | Probability of swapping the colour channels to BGR. |
| `mosaic`, `mixup`, `copy_paste` | Probability of each augmentation. `copy_paste` applies to segmentation labels. |
| `copy_paste_mode` | `flip` or `mixup`. |

`auto_augment`, `erasing` and `crop_fraction` are left out on purpose. Ultralytics reads them only in `ClassificationDataset` and `classify_transforms` (`data/dataset.py:753`), so they do nothing for detect and pose and only made the config look more tuned than it was. `crop_fraction` is also deprecated upstream.

## `training/yolo/fine_tune_train.py` configuration

`fine_tune_train.py` warm-starts from `--checkpoint` at a reduced learning rate. `--lr0` defaults to 0.001, 10x lower than from scratch.

| Entry or flag | Meaning |
| --- | --- |
| `yolo26n` config | Detect head. Its batch (128) matches `train.py`'s `yolo26n`, so a warm-start arm is comparable to the cold-start runs it is graded against (`data_epoch_min` Exp 2). |
| `--save-period` | Writes `weights/epoch{0,N,2N,...}.pt` besides `last.pt` and `best.pt`. Produces the warm-start epochs-to-parity ladder for the `data_epoch_min` experiment. 0 disables. |
| `--fraction` | Fine-tunes on the first FRACTION of the shuffled train split, val untouched. The single-source real-data lever for the warm-regime data-floor sweep (Exp 3, Phase A). |

Its `hyper_params` use the `train.py` keys and meanings above with these differences:

| Key | Value here | Meaning |
| --- | --- | --- |
| `lr0` | `--lr0` | Initial learning rate. |
| `lrf` | 0.01 | Final rate is `lr0 * lrf`. |
| `warmup_bias_lr` | 0.01 | Lowered from 0.1 because this is a fine-tune. |
| `degrees` | 180.0 | Rotation, plus or minus degrees. |
| `flipud` | 0.5 | Vertical flip probability. |
| `auto_augment`, `erasing`, `crop_fraction` | `randaugment`, 0.4, 0.2 | Passed through. They are classification-only (see above) and do nothing for pose. |

## `training/yolo/remap_config_2class.toml`

Read by `remap_labels_by_name.py` to map the archived `nhrl_seg_indiv_labeled` scenes into the 2-class Option B vocabulary. It does not work with `remap_labels.py` (see "Merging scenes by class name").

| Key | Meaning |
| --- | --- |
| `target_names` | Output vocabulary, `robot` then `house_bot`. |
| `default` | `robot`. Every competitor robot name falls through to it. |
| `[map]` | Per-name rules. A value of `__drop__` removes the name. |

- `robot` means anything in the field that is not the house bot: every competitor, our own machines and the two Flight Controller objects. Instance identity is the keypoint model's job.
- `Floor` and `Floor letter word` are dropped. They are field segmentation (88,823 polygons) and belong to the deeplab task.
- `House Bot` and its seasonal liveries map to `house_bot`. It is neutral, never a target, and the only class kept separate.
- The five exhibition machines (`Beater Bot`, `Horizontal Spinner`, `Noob Toob`) NHRL runs against entrants are House-operated and play the same role, so they map to `house_bot`. Newer SegmentFlow exports declare them but no instances are labelled yet, so those rules are for the corpus that is coming, not for anything on disk.
- `Flight Controller 1` and `2` were the `object` class and now fold into `robot`.
- `object` is deliberately not a class. It held 246 boxes from a single 2024 mini-bot scene, far too few to learn, and the distinction it encoded is not one the detector is asked to make.

## `training/yolo/remap_config_seg.toml`

Index map for `remap_labels.py`, source class id to target class id. It is index-stale and unused: `docs/experiments/perception/category_addition_2026-07-25.md` and `data_epoch_min_plan.md` record that it carries 111 entries for an 84-class source and was regenerated by name. Targets: 1 robot, 2 house bot, 3 mr stabs mk2, 4 mrs buff mk3, 5 floor.

| Source ids | Target |
| --- | --- |
| 39 | 5 (floor) |
| 44 to 47 | 2 (house bot) |
| 69 | 3 (mr stabs mk2) |
| 70 | 4 (mrs buff mk3) |
| every other id from 0 to 110 | 1 (robot) |

One index map cannot serve scenes whose indices are shifted, which is why `remap_config_2class.toml` maps by name.

## `training/yolo/remap_config_synthetic.toml`

Index map over the synthetic renders (checked by `training/synthetic/tests/test_annotations.py`), source class id to target class id.

| Source id | Source class | Target id |
| --- | --- | --- |
| 0 | background | 5 |
| 1 | floor | 6 |
| 2 | object | 0 |
| 3 | robot | 1 |
| 4 | mr stabs mk2 | 3 |
| 5 | mrs buff mk3 | 4 |

No source id maps to target 2 (house bot), so the synthetic renders carry no house bot class.

## `training/yolo/configs/seg_merged_robot_house_floor.yaml`

Read by `make_seg_dataset`. In the `merged_classes` mode, `remaining_to_group` is the catch-all: any class name not listed under `groups` maps to it (`combat_robot` here).

## Synthetic arm runs

`training/yolo/run_synthetic_arms.sh` trains the three synthetic-vs-real arms of `docs/experiments/perception/synthetic_arms_plan.md` one after another, not in parallel. Each arm gets the same 3-GPU DDP recipe, so arm-to-arm differences come from the training set and nothing else. It renames each run directory per arm afterwards, because `train.py` names every run by date and model key alone.

## Input geometry dataset builders

Two scripts build the training corpora for the input-resolution arms (`docs/experiments/perception/input_geometry_2026-09-05.md`), plus a gate that decides whether the crop arm is worth building.

### Stretched copy (`training/yolo/make_stretched_dataset.py`)

Builds an anisotropically resized copy of a YOLO dataset, for the stretch arm (arm D), which fills the input tensor by ignoring aspect ratio instead of padding.

- Squeezing 16:9 into a square costs 0.5x horizontally and 0.89x vertically. A robot lands at sqrt(0.5 * 0.89) = 0.67 of its source size against 0.50 for a letterbox, which is 1.33x more robot at the same 409,600 tensor pixels and no padding.
- Normalized YOLO labels need no change, because `cx cy w h` are fractions of width and height. The label files are symlinks. Only images are rewritten.
- Train at `imgsz=640` with `rect=False`. The ultralytics letterbox is a no-op on already square images.

```bash
python training/yolo/make_stretched_dataset.py \
  training/data/nhrl_robots_bbox_2class training/data/nhrl_robots_bbox_2class_stretch
```

### Field-cropped copy (`training/deeplab/build_field_crop_dataset.py`)

Builds the field-cropped corpus for the crop arm (arm E), which asks whether spending the input tensor on the field alone beats spending it on the whole frame. At deployment the field is already known (`field_filter.track_field` runs before perception and is computed once at init), so cropping to it costs arithmetic. The training corpus has no depth, so the field comes from the DeepLab RGB model.

- The `masks` stage runs DeepLab over every image and records the field bounding box in normalized frame coordinates, cached to JSON. It is GPU-bound, a few minutes over 32k images.
- The `crop` stage reads that JSON, crops each image to the box plus a margin, and rewrites the labels. It is CPU-bound, so the margin can be re-tuned without the GPU.
- A crop moves normalized YOLO coordinates, unlike the stretch arm. Every box is re-expressed against the cropped frame and clipped to it.
- `crop_one()` returns labels in, labels out, and frames dropped. With `drop_clipped` set, a frame whose crop discarded a label is skipped entirely. Keeping it would leave a robot visible with no box on it, which trains a false negative and is the one case where an unlabelled image is not a useful background negative. It costs 0.3% of frames at margin 0.20.

```bash
python training/deeplab/build_field_crop_dataset.py masks \
  --model ../../data/models/field_deeplabv3p_r50_2026-07-29.pth \
  --src ../data/nhrl_robots_bbox_2class --out field_boxes.json
python training/deeplab/build_field_crop_dataset.py crop \
  --src ../data/nhrl_robots_bbox_2class --dst ../data/nhrl_robots_bbox_2class_fieldcrop \
  --boxes field_boxes.json --margin 0.10
```

### Field fraction gate (`training/deeplab/field_fraction_gate.py`)

A go/no-go gate for the crop arm. Cropping closes the train/deploy domain gap only if the field fills a similar fraction of the frame in both corpora. NHRL overhead cage cameras are framed on the cage, so the field may already fill the frame there, while the ZED shoots from inside the cage and sees much less. If the fractions differ sharply, cropping teaches a scale the deployment camera never delivers.

- It runs the DeepLab field model over a sample of both corpora and reports, per corpus, the fraction of the frame covered by the field mask and by its bounding box (the box is what a crop uses).
- `--labels` reports how much margin the crop needs before it stops clipping annotated robots, and how much zoom survives that margin. A crop tight enough to zoom cuts robots in half. A crop loose enough to keep them covers the frame.
- `--tensors` reports what the crop is worth once letterboxed into a real engine input, against not cropping. `tensor_tradeoff()` computes zoom (the letterbox scale relative to arm A's 640x640) and padding per engine shape, cropped and uncropped. Source resolution cancels out of the zoom ratio for a 16:9 frame, so 1280x720 stands in for both corpora.
- A letterbox scale is set by whichever axis binds first, and the field box spans nearly the full frame width on both corpora. Cropping removes rows the width-bound scale already applied, so it buys no zoom at any tensor shape 16:9 or squarer. It only spends the freed rows on padding.
- Eval recordings have different framing, so the gate reports each recording separately. One outlier recording cannot decide it.

```bash
python training/deeplab/field_fraction_gate.py --model data/models/field_deeplabv3p_r50_2026-07-29.pth \
  --train-images training/data/nhrl_robots_bbox_2class/train/images \
  --eval-root training/data/nhrl_keypoints_eval_test --sample 300 --device cpu
```

## Dataset builders

### `make_uniform_aspect_subset.py`

Builds a uniform-aspect-ratio view of a YOLO dataset for rectangular training.

- `rect=True` sorts images by aspect ratio and gives each batch its own tensor shape. Ultralytics then refuses to shuffle when the shapes are not all equal (`ultralytics/models/yolo/detect/train.py:95`), and the silent `shuffle=False` feeds the optimizer aspect-sorted (in practice recording-ordered) batches for the whole run.
- `nhrl_robots_bbox_2class` is 99.85% 1920x1080, but 39 train frames from one pre-letterboxed YouTube segment are 1920x886. Measured on 2026-09-05, those 39 frames gave one batch of 810 a different shape and would have killed shuffle for the whole run. Dropping them makes every batch 384x640 at `imgsz=640` and 576x1024 at `imgsz=1024`.
- Only the train split is filtered by default. Validation runs with `rect=True` and `shuffle=False` regardless, so odd-aspect val frames are harmless, and leaving val byte-identical keeps val metrics comparable with a square-trained baseline.
- Images and labels are symlinked, so the view costs no disk and cannot drift from the source.

```bash
python training/yolo/make_uniform_aspect_subset.py \
    training/data/nhrl_robots_bbox_2class training/data/nhrl_robots_bbox_2class_16x9
```

### `make_all_keypoints_arm.py`

Builds the all-keypoints arm: a domain-mix arm list plus both hand-reviewed eval sets. It trains a deployment model on every reviewed keypoint frame, so neither eval set can score that model afterwards.

- Each eval set contributes only the frames its `validation_state.json` marks `pass`.
- Two things in the eval labels would cost frames. Both are fixed in a rewritten copy under `<out>/eval_labels/` beside symlinked images, because ultralytics finds a label by swapping `images` for `labels` in the image path.
- Rows in a class the model does not have (`object`, index 4) are dropped.
- Keypoints outside [0, 1] become `0 0 0`, unlabelled. They mark a robot partly out of frame (up to 1.17 on the ZED set), and ultralytics drops the whole frame for one of them. Clamping to the edge would move the point. The box stays as labelled.

```bash
venv/bin/python training/yolo/make_all_keypoints_arm.py \
    --arm training/data/domain_mix_arms_2026-09-19/d50000.txt \
    --val training/data/domain_mix_arms_2026-09-19/val.txt \
    --eval training/data/cage_high_x50_conf044 training/data/nhrl_keypoints_eval_test \
    --out training/data/all_keypoints_2026-09-25 --name all_keypoints
```

### `seg_to_boxonly_pose.py`

Builds an opponent-only YOLO pose dataset from a segmentation-polygon dataset (for example `training/data/nhrl_seg/nhrl_robots`). Used for Exp 2: mix real opponent boxes (single class, box-only) into a keypoint dataset.

- A frame containing any `--exclude-frame-classes` label (for example our own robots) is skipped whole, so those robots never appear unlabelled in the opponent data. Our robots come only from the keypoint dataset, with real keypoints.
- Each `--keep-class` polygon becomes its bounding box, written as a pose row `<out-class> cx cy w h 0 0 0 ...` with `--num-keypoints` visibility-0 (masked) keypoints. Ultralytics ignores vis-0 keypoints in the location loss, so these rows are box-only supervision and stay schema-compatible with real keypoint rows.
- Frames with no kept opponent boxes are skipped. Paired images are copied (hardlinked when possible).

```bash
python training/yolo/seg_to_boxonly_pose.py training/data/nhrl_seg/nhrl_robots \
    -o training/data/opponent_pose \
    --keep-class 1 --exclude-frame-classes 3 4 --out-class 2 --num-keypoints 2
```

## Workarounds

Facts that used to live in comments beside the code.

### Export and engine build

- The calibrator holds its CUDA buffer on the instance. TensorRT receives a raw device address and does not keep the allocation alive. torch already owns the CUDA context, so pycuda would be a second dependency for the same pointer.
