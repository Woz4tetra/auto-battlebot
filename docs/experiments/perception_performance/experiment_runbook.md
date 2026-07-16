# Perception model experiment runbook

How to take a perception experiment from raw data to a scored model. Covers the dataset recipes used across the experiments run so far, training on megamind, exporting the engine, and scoring against the eval set.

Pipeline: **collect dataset -> prepare megamind -> train -> export engine -> score**.

All Python runs in the project venv (`venv/bin/python`). All paths are relative to the repo root unless noted.

---

## 1. Build the dataset

Three recipes have been used. All converge on the same split/validate step.

### Recipe A - Synthetic generation (BlenderProc)

Used for `all_robot_keypoints`, `meshy_grade`. Config lives in `training/synthetic/config_*.toml`; each target robot is a `[[robots]]` block with `class_id`, `scale`, `color_mapping`, and `[robots.keypoints]`.

```bash
# tmux, multi-hour to multi-day. Renders into config's [output] dir (../data/<name>).
tmux new-session -d -s render -c /home/ben/auto-battlebot \
  'bash training/synthetic/docker/run_synthetic.sh --require-gpu auto-battlebot-synthetic \
     blenderproc run render_scenes.py -- config_meshy_grade.toml --num-images 35000'
```

- Inspect a small debug batch (`--num-images 40`) and per-class counts before committing to a full run.
- **Scale gotcha:** `[[robots]]` `scale` multiplies the mesh's native size with no auto-normalization (distractors auto-normalize, robots do not). Meshy GLBs are ~1.9 m native and must be scaled to ~0.25-0.29 m to match our robots.
- **Meshy models:** `color_mapping = []` (renders the baked texture), `flat_ground = true` (Z-up GLB sits flat instead of the CAD ±90° pitch), keypoints copied from the model's sidecar JSON.
- Output is flat `images/` + `labels/` (YOLO pose rows: `cls cx cy w h  kx ky kv  kx ky kv`).
- The render writes a stub `data.yml` for its own self-check (train/val both point at the flat dir). Replace it with a real split config below.

### Recipe B - Real seg -> box-only pose (`seg_to_boxonly_pose.py`)

Used for Exp 2. Turns a segmentation-polygon dataset into opponent-only, box-only pose labels: skips any frame with our robots, converts kept polygons to bboxes, appends vis-0 (masked) keypoints so it stays schema-compatible with real keypoint rows.

```bash
venv/bin/python training/yolo/seg_to_boxonly_pose.py training/data/nhrl_seg/nhrl_robots \
  -o training/data/opponent_pose \
  --keep-class 1 --exclude-frame-classes 3 4 --out-class 2 --num-keypoints 2
```

### Recipe C - Pool multiple sources

Used for Exp 2 (our-robot keypoints + opponent boxes into one dataset). Hardlink (`os.link`) the pairs from each source into one flat `images/`+`labels/` with source-prefixed filenames. Hardlinks make this instant; a forking `cp`/`ln` loop is pathologically slow at scale.

Do **not** use `merge_yolo_datasets.py` for this: it needs a `validation_state.json` per input, only ingests flat datasets, and drops `flip_idx`.

### Split + data.yml + validate (all recipes)

```bash
# Splits flat images/labels into train/val/test. Globs *.jpg, copies files. Remainder after
# train+val goes to test; grading uses the external eval set, so test can be ~0.
venv/bin/python training/yolo/split_yolo_dataset.py \
  training/data/<name>/images training/data/<name>/labels training/data/<name> -t 0.9 -v 0.1
```

Author `training/data/<name>/data.yml` (pose):

```yaml
path: /home/ben/auto-battlebot/training/data/<name>   # location ON megamind
train: train/images
val: val/images
test: test/images
nc: <N>
names: [ ... ]              # class order defines the --labels mapping at score time
kpt_shape: [2, 3]
flip_idx: [0, 1]
colors: [ "#...", ... ]     # one per class
```

```bash
venv/bin/python training/yolo/validate_yolo_integrity.py training/data/<name> --strict
```

Expect: correct class count, our-robot rows with vis 1/2 keypoints, opponent rows with vis-0, no malformed rows.

---

## 2. Prepare megamind

megamind = the training box (3x RTX A6000, sm86). Its repo is mounted locally at
`/run/user/1000/gvfs/sftp:host=megamind,user=ben/home/ben/auto-battlebot`, and `ssh megamind` works for commands.

- **Check the archive first.** Big datasets already live in `/media/storage/auto-battlebots-archive/` (`nhrl_robots`, `our_robot_keypoints`, `nhrl_seg`, ...). Symlink/copy on megamind instead of re-uploading over the ~5 MB/s link.
- **Upload a new dataset** (only the split, not the flat source):

```bash
rsync -a --info=progress2 \
  training/data/<name>/train training/data/<name>/val training/data/<name>/data.yml \
  megamind:/home/ben/auto-battlebot/training/data/<name>/
```

- **Do not run heavy IO / benchmarks / cache builds on megamind during a training run** - it evicts the training's page cache and spikes epoch time ~30x until it recovers.

---

## 3. Run training

Run inside a megamind tmux session (survives disconnect), working dir `/home/ben/auto-battlebot`:

```bash
ssh megamind tmux new-session -d -s train -c /home/ben/auto-battlebot \
  'venv/bin/python training/yolo/train.py training/data/<name>/data.yml yolo26n-pose 2>&1 | tee /tmp/train.log'
```

`train.py <data.yml> <model>...` options:
- `-d 0 1 2` GPUs (default all three), `-e <epochs>` override, `-c <ckpt>` resume (into a **new** run dir), `-w` workers.
- `--cache disk` (default) reads full-res `.npy` each epoch; `ram` avoids per-epoch IO if the resized cache fits in RAM. Epoch speed collapses if the `.npy` cache exceeds RAM page cache - downscale source images above ~720p first.

Output lands in `runs/projects/auto_battlebots_<ts>_<model>/weights/best.pt` (not under `training/`).

Monitor `runs/projects/<run>/results.csv`: `metrics/mAP50-95(B)` (box), `metrics/mAP50-95(P)` (pose). Cancel when the metric you care about plateaus - box and pose plateau at different epochs.

---

## 4. Download + format the model

```bash
GV="/run/user/1000/gvfs/sftp:host=megamind,user=ben/home/ben/auto-battlebot"
cp "$GV/runs/projects/<run>/weights/best.pt" data/models/yolo26n-pose_<name>_<date>.pt

# ONNX: disables the end2end NMS head -> raw pre-NMS output [1, 4+nc+3*nk, 8400] that
# the C++ model and score.py expect.
venv/bin/python training/yolo/convert_to_onnx.py data/models/yolo26n-pose_<name>_<date>.pt

# TensorRT: platform+GPU tag auto-appended. Cap --workspace to fit free VRAM (a render may be
# sharing the GPU). yolo26n needs ~1 GiB.
venv/bin/python training/yolo/convert_to_tensorrt.py \
  data/models/yolo26n-pose_<name>_<date>.onnx --workspace 1
# -> data/models/yolo26n-pose_<name>_<date>_x86_64_sm89.engine
```

**Engines are GPU-arch-specific.** Build the `x86_64_sm89` engine on the dev box for local scoring. Build the `aarch64_sm87` engine **on the Jetson** for deployment. Do not build the scoring engine on megamind (sm86, incompatible with the dev GPU).

---

## 5. Score the model

```bash
venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate <name>=data/models/yolo26n-pose_<name>_<date>_x86_64_sm89.engine \
  --labels "<GT label per engine class index, in class order>" \
  --taxonomy training/model_eval/taxonomy.yaml --conf 0.5 \
  --output training/data/nhrl_keypoints_eval_test/scores_<name>
```

- **`--labels` length MUST equal the engine's class count.** `score.py` infers `num_classes` from the label count and derives `num_keypoints` from the output width. A wrong count misparses the tensor to `num_keypoints=0` and returns ~0 recall (looks like a broken engine). Check the printed `output [...] num_keypoints=N num_classes=M` line.
- `--labels` is one shared mapping for all candidates, so engines with **different class counts cannot share a run**. Score them separately.
- `taxonomy.yaml` = general box metrics (all classes). `taxonomy_keypoint.yaml` = excludes opponents, so keypoint metrics reflect our robots only.
- Multiple candidates with the **same class order** get paired-bootstrap significance (`--baseline NAME`).
- Levels: `agnostic` (all robots -> one blob), `archetype`/`instance` (per-class AP). Read `ap50_95/<class>` per class.

### Reference points (measured)

- Opponent box AP50-95: **0.084** (generic CAD synthetic, floor) / **~0.21** (real-trained, ceiling).
- Agnostic recall: **0.675** (real seg blob, best pure detector).
- Keypoint heading error on our robot: **9.0 deg** (dedicated our-robot model, good) / **38.5 deg** (opponent-diluted combined model, unusable for aim assist).

### Meshy fidelity grade (Exp 1) example

6 classes `[mr_stabs_mk2, mrs_buff_mk3, clyde, sphinx, wreckcreation, ironwarrior]`; the four named opponents map to `opponent`:

```bash
--labels "mr_stabs_mk2,mrs_buff_mk3,opponent,opponent,opponent,opponent"
```

If opponent recall/AP clears the 0.084 floor toward the 0.21 ceiling, the Meshy reproductions are faithful.

---

## Locations

- Datasets: `training/data/<name>/` (local) + megamind mirror + `/media/storage/auto-battlebots-archive/`
- Models: `data/models/<name>.{pt,onnx,engine}`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_<name>/{summary.csv, headline.png, confusion_*.png, significance.csv}`
- Writeups: `docs/experiments/perception_performance/<name>_<date>.md`
