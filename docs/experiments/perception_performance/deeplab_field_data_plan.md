# How much field data does DeepLab need, and when do I stop training it?

Status: **not started** (2026-07-27). Corpus surveyed, throughput measured, blocking defects found.
Phase 0 must land before any training run is meaningful.

## Questions

1. **How many field images do I need for each field type?** — you arrive at an event with a cage the
   model has never seen. How many labelled frames of *that cage* buy usable floor segmentation?
2. **When do I stop training DeepLab?** — the current recipe defaults to 500 epochs and keeps
   whichever checkpoint scored best on val. Neither number is grounded.

These mirror `data_scaling_2026-07-27` (frames vs scenes for the detector), but the axis is
different. For the detector the unit of diversity was the *fight*; for a floor-segmentation model the
unit is the *field* — cage geometry, floor colour, wall paint, lighting rig, camera viewpoint. A new
opponent is a new object in a known scene; a new cage is a whole new scene.

## What the corpus actually is

`training/data/floor_mask_dataset`, 19 GB. The file count badly overstates it:

| | |
|---|---|
| `.jpg` files on disk | 54,883 |
| minus offline `_augment-N` copies | −8,444 |
| minus `floor_dataset__`-prefixed duplicates | −11,689 |
| **unique labelled images** | **34,750** |
| splits as shipped | train 49,394 / val 5,488 / test **1** |

Three defects, all of which have to be fixed before a number from this corpus means anything.

### 1. 11,689 frames are stored twice, byte-for-byte

Every scene under a `floor_dataset__` prefix has an identically-named, identically-sized twin without
it. Sampled 400 name-pairs and MD5'd both sides: **400/400 identical**. This is a merge artifact —
`merge_segmask_datasets.py` was run over a tree that already contained the images.

### 2. Train and val leak into each other

The split was drawn per-file, not per-source-frame. **1,114 source frames have `_augment-N` siblings
on both sides of the train/val line**, and the duplicate pairs above put the same pixels in train
under one name and val under another. Val IoU is therefore measuring partial memorisation, which is
exactly the signal Q2 needs it not to measure.

### 3. The offline augment copies are redundant

`SegDataset._augment` already applies brightness/contrast/saturation/hue/blur/flips online at
`semantic_train.py:104`. The baked `_augment-000/001` files add a second, frozen layer of the same
thing — they inflate epoch time by ~25 %, break the split, and buy nothing.

### Field-type breakdown (deduplicated, 34,750 frames)

| field | frames | scenes | share |
|---|---|---|---|
| Cage-6 | 11,217 | 13 | 32.3 % |
| Cage-2 | 5,565 | 6 | 16.0 % |
| own recent recordings (2025-11 … 2026-03) | 4,491 | 10 | 12.9 % |
| mini-bot rig | 2,960 | 15 | 8.5 % |
| MobileCam (1/3/4) | 2,604 | 6 | 7.5 % |
| Cage-5 | 2,174 | 7 | 6.3 % |
| own ZED, 2023 | 2,100 | 3 | 6.0 % |
| SkyCam (A/B) | 1,976 | 4 | 5.7 % |
| unclassified | 599 | 7 | 1.7 % |
| Cage-7 | 509 | 5 | 1.5 % |
| Cage-1 | 319 | 1 | 0.9 % |
| Cage-4 | 152 | 2 | 0.4 % |
| Cage-3 | 84 | 1 | 0.2 % |

**Cage-6 alone is a third of the corpus.** Global mean IoU is therefore ~⅓ a Cage-6 score, which
makes it the wrong model-selection metric — the checkpoint that maximises it can be regressing
Cage-3 and Cage-4 the whole way. Everything below scores **macro-average across field types** and
**worst field**, not the global mean.

The long tail is already the answer to a weaker version of Q1: Cage-3 has 84 frames and Cage-4 has
152. If those score acceptably, the per-field floor is *below* 84 and the experiment is mostly
confirming a bound you already have.

## The missing piece: there is no segmentation eval harness

`training/model_eval/score.py` grades detectors and keypoints only. Nothing in the repo computes
mask IoU against held-out ground truth, so today the only stopping signal is the leaky val IoU
printed by `livelossplot`. **`score_masks.py` is the load-bearing deliverable of Phase 0** — without
it neither question has a measurable answer.

## Throughput — measured, not estimated

Benchmarked on one A6000 (r50 / v3plus, 344 + 2×20 = 384 px):

| config | img/s | epoch @ 29.5 k train |
|---|---|---|
| fp32, batch 8 (current default) | 101 | 4.9 min |
| fp32, batch 16 | 104 | 4.7 min |
| **bf16 autocast, batch 32** | **174** | **2.8 min** |
| dataloader alone, 32 workers | 623 | — |

Two things follow. **The GPU is the bottleneck, not the input pipeline** — the loader can feed 6×
what the model consumes, so batch size and worker count are not where the time goes. And **bf16
autocast is a free 1.67×**. Adding it is the single highest-leverage change to the training script
and it is a prerequisite for the run budget below.

There are 3× A6000, so three arms run concurrently. `semantic_train.py` is single-GPU today; no DDP
is needed, just one process pinned per GPU.

---

## Phase 0 — fix the corpus, build the harness (no training)

Nothing downstream is interpretable until this lands.

1. **Inventory and label every frame with its field type**, parsed from the filename
   (`Cage-N-<view>`, `MobileCam-N`, `SkyCam-X`, `zed_*`, `mini_bot_*`, own-recording timestamps).
   Write a manifest so every later arm is reconstructible.
2. **Verify the duplicate claim at full scale** before deleting anything — MD5 all 11,689 pairs, not
   the 400 already sampled. Any pair that differs gets kept and flagged.
3. **Drop the offline `_augment-N` copies.** Online augmentation already covers them.
4. **Rebuild the splits scene-disjoint and field-stratified.** Every field type appears in both train
   and eval, but no *scene* crosses the line. Hold ~15 % out.
5. **Write `score_masks.py`** — the harness.
6. **Re-measure the deployed model** (`field_deeplabv3p_r50_2026-04-29.pth`) on the new eval set.
   That is the baseline anchor everything else is compared against.

A sanity check worth doing before trusting any of it: `view_mask.py` on ~30 random frames per field,
because a systematically wrong mask in a rare field would masquerade as "this field needs more data".

### What `score_masks.py` reports

Per field type, and macro-averaged across fields:

- **field-class IoU** — the primary number.
- **boundary F1 at 2 / 5 / 10 px** — the mask feeds `PointCloudFieldFilter`, which fits the field
  plane and hands wall bounds to navigation. A mask that is 2 % wrong in the interior costs nothing;
  one that is 2 % wrong at the wall line moves the boundary. IoU alone will not see the difference.
- **worst-field IoU** — the metric that actually gates deployment.
- **paired bootstrap, 1000×**, same statistical treatment as `model_eval/score.py`, so results are
  comparable in kind to the detector experiments.

Grade the `.pth` checkpoints directly rather than converting each arm to TensorRT. Engine conversion
is only needed for the final deployment candidate; a per-arm conversion would add ~20 runs of
overhead to measure a quantisation effect this experiment is not asking about.

---

## Phase A — when do I stop training? (Q2)

**2 runs on the full cleaned corpus, cold start, 150 epochs, checkpoint every 10.**

Two runs, not one, and they differ **only by seed**. This is the same discipline that made
`data_scaling` readable: that series found a 0.048 recall spread between two nominally identical
detector runs, which is larger than most of the effects people wanted to claim. **The seed pair
measures the noise floor δ, and δ is what makes every gate in Phase B meaningful.** Skipping it makes
Phase B's ladder uninterpretable.

Answer: the earliest checkpoint whose **macro-average IoU** is within δ of that arm's best, holding
worst-field IoU within δ too. Reported as an epoch count, with the caveat that it is a count for
*this* corpus size.

Two changes to `semantic_train.py` are prerequisites, not nice-to-haves:

- **`--save-period`** — it currently saves only the best-so-far checkpoint, so there is no ladder to
  grade. Q2 cannot be answered without one.
- **bf16 autocast** — the 1.67× above. Verify it does not move final IoU by running the seed-A arm
  both ways for the first 20 epochs before committing the budget.

Worth fixing at the same time, since it is the same edit: the best-checkpoint rule selects on global
val IoU, which as established is ⅓ Cage-6. Select on macro-average instead.

Cost: 150 epochs × 2.8 min ≈ **7 h per run**, 2 runs concurrent on 2 GPUs ≈ **7 h wall**.

---

## Phase B — how many images per field type? (Q1)

**Leave-one-field-out with an add-back ladder.** Hold a probe field entirely out of training, then
add back N labelled frames of it and watch the IoU on that field's held-out scenes.

### Probe fields

| probe | frames | scenes | why |
|---|---|---|---|
| **Cage-2** | 5,565 | 6 | a genuinely different NHRL cage, enough data to build the whole ladder |
| **Cage-5** | 2,174 | 7 | second NHRL cage — replication, so the answer is not one draw |
| **own ZED 2023** | 2,100 | 3 | the deployment camera. Most decision-relevant, weakest holdout |

Cage-6 is unusable as a probe: removing it removes a third of the corpus, so the base arm would
differ from every other arm in size as well as field coverage — the exact confound that made the
earlier data-floor result unusable.

The ZED probe is a spot check at 3 rungs, not a full ladder, and its result is indicative only —
3 scenes cannot give a scene-disjoint holdout with any power. It is included because it is the only
probe that answers the question *for the camera the robot actually uses*, and a weak reading there
beats a strong reading on footage the robot will never see.

### Ladder

N ∈ **{0, 50, 150, 400, 1000, all}** frames of the probe field, drawn scene-spread (sampling across
all of that field's scenes rather than taking whole scenes). `data_scaling` found scene diversity beat
frame count for the detector and the gap widened as data shrank; for a fixed field the analogous
choice is to spread the budget across that field's recordings.

**Rungs are nested** (`50 ⊂ 150 ⊂ 400 ⊂ 1000 ⊂ all`) so a drop between rungs cannot be blamed on
which frames were drawn.

Everything that is *not* the probe field is held fixed across all rungs, so the only variable is how
much probe-field data the model saw.

### Fine-tune, with a cold-start control

Each rung **fine-tunes from its own N=0 LOFO base checkpoint** for 30 epochs, rather than cold-starting.

Two reasons. It is the operationally real question — you land at an event, label some frames of the
new cage, and retrain overnight; nobody cold-starts. And it costs ~2.5× less, which is what makes
replication across two probe fields affordable at all.

The risk is that the fine-tune curve is an artifact of where the base checkpoint sits rather than a
statement about data volume. **Control: cold-start Cage-2 at N=150 and N=all.** If those two land
within δ of their fine-tuned counterparts, the ladder reads as a data-volume result. If they do not,
the ladder is reported as a fine-tuning result only — still useful, narrower claim.

### Arms and cost

| | runs | epochs | cost |
|---|---|---|---|
| LOFO bases (Cage-2, Cage-5, ZED) | 3 | 150 | 3 × 7 h |
| Cage-2 ladder (5 rungs above N=0) | 5 | 30 | 5 × 1.4 h |
| Cage-5 ladder (5 rungs) | 5 | 30 | 5 × 1.4 h |
| ZED spot check (N ∈ 150, 400, all) | 3 | 30 | 3 × 1.4 h |
| cold-start controls | 2 | 150 | 2 × 7 h |
| **total** | **18** | | **~53 GPU-h ≈ 18 h wall on 3 GPUs** |

Phase A + Phase B ≈ **25 h wall**. A weekend, unattended.

**If that is too much**, drop the ZED spot check and the Cage-5 ladder (−9 runs, ~10 h). The cost is
replication: the answer then rests on one probe field, and `data_scaling`'s scene-arm caveat applies —
a single draw is indicative, not settled.

### Gate

For each rung, the smallest N whose probe-field IoU is within δ of the N=all ceiling, with boundary
F1 not significantly worse. Reported as *"N frames of a new field reaches within δ of saturation;
below that, X."*

The N=0 rung is the interesting one in its own right: it measures **how well the model generalises to
a completely unseen field**. If N=0 already clears the gate, the answer to Q1 is "zero — the model
transfers" and the rest of the ladder is confirmation.

---

## Phase C — downstream confirmation

Mask IoU is a proxy. The mask's real job is to feed `PointCloudFieldFilter` a plane fit and give
navigation its wall bounds, so the winning checkpoint gets one replay before anyone believes it:
convert to TensorRT, replay an SVO, and compare field-fit stability and wall bounds against the
deployed model. A checkpoint that wins on IoU but destabilises the plane fit has not won.

---

## Deliverable

1. **A per-field saturation curve** — IoU vs N labelled frames of that field, one line per probe
   field, against a horizontal "field held out entirely" line and a "field fully represented" ceiling.
2. **An epoch-vs-macro-IoU curve** with the noise band from the seed pair drawn on it, and the epoch
   where it flattens.
3. **Two numbers** for `my_takeaways.md`, in the same one-line form as the existing entries.
4. **A clean corpus** — `deeplab_field_2026-07-28/`, deduplicated, field-labelled, scene-disjoint
   splits, in the same role `nhrl_robots_bbox_2class` now plays for the detector. This outlives the
   experiment and is arguably worth more than either answer.

## Risks / caveats

- **Field type is inferred from filenames, not verified.** `Cage-6-Red` and `Cage-6-Red-High` are the
  same cage from different mounts; `Cage-6-Overhead-High` may or may not be the same physical floor
  across a two-year span of tournaments (`jun24` … `feb26`). If NHRL repainted or re-floored a cage,
  one label spans two visually distinct fields. Spot-check one frame per scene before trusting the
  grouping.
- **The domain gap is the elephant, same as in `data_scaling`.** 79 % of this corpus is NHRL
  broadcast footage — overhead, SkyCam, MobileCam. The deployed model runs on the robot's own ZED at
  floor level. A saturation curve measured on broadcast fields may not transfer to the deployment
  viewpoint at all. This is why the ZED probe is in Phase B despite its weak holdout, and it is the
  strongest argument for spending the *next* labelling budget on ZED footage rather than on more
  cages.
- **"New field" here is a held-out field from the same corpus**, sharing capture pipeline, resolution
  and era with the training fields. A genuinely new cage at a future event will be harder than any
  probe measured here, so treat the resulting N as a **lower bound**.
- **One seed per Phase-B arm.** δ comes from the Phase A pair and is assumed to hold at the smaller
  arm sizes, which is an assumption, not a measurement — noise usually grows as data shrinks. Rungs
  differing by less than δ are not separable.
- **Fine-tuning confounds Phase B** unless the cold-start controls agree. Stated above; repeating
  because it determines how the headline may be worded.
- **bf16 is assumed loss-neutral.** Checked over 20 epochs, not 150. If the seed-A fp32/bf16 pair
  diverges late, the budget has to be rebuilt at fp32 (~1.67× everything).
- **Deleting 11,689 duplicates and 8,444 augment copies is destructive.** Archive the current
  `floor_mask_dataset` to `/media/storage/auto-battlebots-archive/` *before* Phase 0 step 3, not
  after. 19 GB.
- **Boundary F1 has no established threshold** in this project the way mAP 0.5 does for the detector.
  The first run of it produces a number nobody can yet calibrate; the deployed-model anchor is what
  gives it meaning, which is why Phase 0 step 6 is not optional.
- **Cage-3 (84 frames) and Cage-4 (152) are too small to hold scenes out of.** They stay in training
  and are reported as-is; they cannot be probes.

## Artifacts / locations

- Corpus (current, to be archived): `training/data/floor_mask_dataset/`
- Corpus (clean): `training/data/deeplab_field_2026-07-28/` + `manifest.json`
- Arms: `training/data/deeplab_field_2026-07-28/arms/*.txt` (image lists, not copies)
- Runs: `training/data/deeplab_field_2026-07-28/runs/<arm>/`
- Scores: `training/data/deeplab_field_2026-07-28/scores/`
- Baseline anchor: `data/models/field_deeplabv3p_r50_2026-04-29.pth` (r50 / v3plus / 344 / pad 20)
- Writeup: `docs/experiments/perception_performance/deeplab_field_data_<date>.md`

New scripts (Phase 0):

- `training/deeplab/build_field_manifest.py`
- `training/deeplab/make_field_splits.py`
- `training/deeplab/score_masks.py`

Edits to `training/deeplab/semantic_train.py`: `--save-period`, `--train-list` / `--val-list`,
bf16 autocast, macro-average model selection.

`--train-list` matters more than it looks. Without it every arm needs a physical copy of ~30 k
images; 18 arms would be ~350 GB against 282 GB free. Ultralytics already accepts image lists and
`data_scaling` used them for exactly this reason — the five detector arms cost kilobytes. Same trick
here.

---

## Commands

```bash
source scripts/activate_python.sh
```

### Phase 0 — corpus and harness

```bash
# Archive before anything destructive. 19 GB.
rsync -a --info=progress2 training/data/floor_mask_dataset/ \
    /media/storage/auto-battlebots-archive/floor_mask_dataset_2026-07-27/

# 1. Field labels + duplicate/augment inventory. Read-only, writes only the manifest.
python3 training/deeplab/build_field_manifest.py training/data/floor_mask_dataset \
    --out training/data/deeplab_field_2026-07-28/manifest.json

# 2. Verify the duplicate claim at full scale (all 11,689 pairs, not a sample).
python3 training/deeplab/build_field_manifest.py training/data/floor_mask_dataset \
    --manifest training/data/deeplab_field_2026-07-28/manifest.json \
    --verify-duplicates --hash md5

# 3. Clean corpus + field-stratified, scene-disjoint splits. Nested arms asserted here.
python3 training/deeplab/make_field_splits.py \
    --manifest training/data/deeplab_field_2026-07-28/manifest.json \
    --out training/data/deeplab_field_2026-07-28 \
    --val-frac 0.15 --drop-augment-copies --drop-duplicates --seed 4176

# 4. Eyeball masks per field before trusting any score.
python3 training/deeplab/view_mask.py <image> <mask>   # ~30 random frames per field

# 5. Baseline anchor: deployed model on the new eval set.
python3 training/deeplab/score_masks.py \
    training/data/deeplab_field_2026-07-28/eval \
    --candidate deployed=data/models/field_deeplabv3p_r50_2026-04-29.pth \
    --by-field --boundary-tol 2,5,10 --bootstrap 1000 \
    --output training/data/deeplab_field_2026-07-28/scores/anchor
```

### Phase A — epoch ladder

```bash
cd training/deeplab

# bf16 sanity check before spending the budget: same seed, 20 epochs, both precisions.
python3 semantic_train.py ../data/deeplab_field_2026-07-28 \
    --train-list ../data/deeplab_field_2026-07-28/arms/full.txt \
    -b 32 -nw 16 -ne 20 --backbone r50 --decoder v3plus --amp bf16 \
    -o ../data/deeplab_field_2026-07-28/runs/amp_check_bf16
python3 semantic_train.py ../data/deeplab_field_2026-07-28 \
    --train-list ../data/deeplab_field_2026-07-28/arms/full.txt \
    -b 16 -nw 16 -ne 20 --backbone r50 --decoder v3plus \
    -o ../data/deeplab_field_2026-07-28/runs/amp_check_fp32

# Seed pair, full corpus, 150 epochs, ladder every 10. One per GPU, concurrent.
CUDA_VISIBLE_DEVICES=0 python3 semantic_train.py ../data/deeplab_field_2026-07-28 \
    --train-list ../data/deeplab_field_2026-07-28/arms/full.txt \
    -b 32 -nw 16 -ne 150 --save-period 10 --seed 4176 \
    --backbone r50 --decoder v3plus --amp bf16 \
    -o ../data/deeplab_field_2026-07-28/runs/phaseA_seedA

CUDA_VISIBLE_DEVICES=1 python3 semantic_train.py ../data/deeplab_field_2026-07-28 \
    --train-list ../data/deeplab_field_2026-07-28/arms/full.txt \
    -b 32 -nw 16 -ne 150 --save-period 10 --seed 9001 \
    --backbone r50 --decoder v3plus --amp bf16 \
    -o ../data/deeplab_field_2026-07-28/runs/phaseA_seedB

# Grade the whole ladder in one paired invocation -> gives delta directly.
cd /home/ben/auto-battlebot
python3 training/deeplab/score_masks.py \
    training/data/deeplab_field_2026-07-28/eval \
    --candidate deployed=data/models/field_deeplabv3p_r50_2026-04-29.pth \
    --candidate-glob 'training/data/deeplab_field_2026-07-28/runs/phaseA_seed*/ckpt_ep*.pth' \
    --baseline deployed --by-field --boundary-tol 2,5,10 --bootstrap 1000 \
    --output training/data/deeplab_field_2026-07-28/scores/phaseA
```

### Phase B — LOFO add-back ladder

```bash
cd training/deeplab

# LOFO bases: probe field removed entirely. 3 runs, one per GPU.
for i in 0 1 2; do
  probe=$(echo "cage2 cage5 zed2023" | cut -d' ' -f$((i+1)))
  CUDA_VISIBLE_DEVICES=$i python3 semantic_train.py ../data/deeplab_field_2026-07-28 \
      --train-list ../data/deeplab_field_2026-07-28/arms/lofo_${probe}_n0.txt \
      -b 32 -nw 16 -ne 150 --save-period 10 --seed 4176 \
      --backbone r50 --decoder v3plus --amp bf16 \
      -o ../data/deeplab_field_2026-07-28/runs/lofo_${probe}_n0 &
done; wait

# Add-back rungs: fine-tune from that probe's own N=0 base, 30 epochs each.
# 13 runs total (5 + 5 + 3); 3 at a time.
for probe in cage2 cage5; do
  for n in 50 150 400 1000 all; do
    python3 semantic_train.py ../data/deeplab_field_2026-07-28 \
        --train-list ../data/deeplab_field_2026-07-28/arms/lofo_${probe}_n${n}.txt \
        -c ../data/deeplab_field_2026-07-28/runs/lofo_${probe}_n0/model_r50.pth \
        -b 32 -nw 16 -ne 30 --save-period 10 --seed 4176 \
        --backbone r50 --decoder v3plus --amp bf16 \
        -o ../data/deeplab_field_2026-07-28/runs/lofo_${probe}_n${n}
  done
done

for n in 150 400 all; do
  python3 semantic_train.py ../data/deeplab_field_2026-07-28 \
      --train-list ../data/deeplab_field_2026-07-28/arms/lofo_zed2023_n${n}.txt \
      -c ../data/deeplab_field_2026-07-28/runs/lofo_zed2023_n0/model_r50.pth \
      -b 32 -nw 16 -ne 30 --save-period 10 --seed 4176 \
      --backbone r50 --decoder v3plus --amp bf16 \
      -o ../data/deeplab_field_2026-07-28/runs/lofo_zed2023_n${n}
done

# Cold-start controls: does the fine-tune ladder read as a data-volume result?
for n in 150 all; do
  python3 semantic_train.py ../data/deeplab_field_2026-07-28 \
      --train-list ../data/deeplab_field_2026-07-28/arms/lofo_cage2_n${n}.txt \
      -b 32 -nw 16 -ne 150 --save-period 10 --seed 4176 \
      --backbone r50 --decoder v3plus --amp bf16 \
      -o ../data/deeplab_field_2026-07-28/runs/cold_cage2_n${n}
done

# Score each probe against its own held-out scenes.
cd /home/ben/auto-battlebot
for probe in cage2 cage5 zed2023; do
  python3 training/deeplab/score_masks.py \
      training/data/deeplab_field_2026-07-28/eval \
      --field-filter ${probe} \
      --candidate-glob "training/data/deeplab_field_2026-07-28/runs/*_${probe}_n*/ckpt_ep*.pth" \
      --baseline lofo_${probe}_n0 --by-field --boundary-tol 2,5,10 --bootstrap 1000 \
      --output training/data/deeplab_field_2026-07-28/scores/lofo_${probe}
done
```

### Phase C — downstream check on the winner

```bash
python3 training/deeplab/convert_to_tensorrt.py \
    training/data/deeplab_field_2026-07-28/runs/<winner>/model_r50.pth \
    -o data/models/field_deeplabv3p_r50_<date>_x86_64_sm86.engine --keep-onnx

# Replay against the deployed model; compare field-fit stability and wall bounds.
./scripts/build_and_run.sh -c config/playback.toml
```

### Validation before committing anything

```bash
./scripts/lint
```
