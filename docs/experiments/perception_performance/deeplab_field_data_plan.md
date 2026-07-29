# How much field data does DeepLab need, and when do I stop training it?

Status: **Phase 0 landed, corpus rebuilt, blocked on four reconciliation items** (2026-07-29).
The harness and the training-script edits are in. The corpus was rebuilt on top of a convex hull fix
and folded in new material, so the arms, the field index and the mask provenance all need to be
brought back in sync before Phase A means anything.

## Questions

1. **How many field images do I need for each field type?** You arrive at an event with a cage the
   model has never seen. How many labelled frames of *that cage* buy usable floor segmentation?
2. **When do I stop training DeepLab?** The current recipe defaults to 500 epochs and keeps whichever
   checkpoint scored best on val. Neither number is grounded.

These mirror `data_scaling_2026-07-27` (frames vs scenes for the detector), but the axis is
different. For the detector the unit of diversity was the *fight*; for a floor-segmentation model the
unit is the *field*: cage geometry, floor colour, wall paint, lighting rig, camera viewpoint. A new
opponent is a new object in a known scene. A new cage is a whole new scene.

## What the corpus is now

`/home/ben/Desktop/deeplab_field_2026-07-29`, 2 classes (`background`, `floor`).

| | |
|---|---|
| train | 29,049 |
| val | 6,001 |
| **total** | **35,050** |
| val share | 17.2 % |
| scenes covered by `field_index.json` | 80 |

It was assembled by `build_field_dataset.py` from two pieces:

- **Held corpus `deeplab_field_2026-07-28_hull_old`**, 32,563 frames (train 26,963 / val 5,600). This
  is the `make_field_splits.py` output from Phase 0, and its train/val assignment was **preserved
  verbatim**, so anchor scores computed against it stay comparable.
- **2,487 new frames** across 16 source videos, assigned at video level with seed 0. Three are
  SegmentFlow exports (`LdkIH0p8NEM` 778, `jgMOA903mqU` 224, `qkKV55hWmiQ` 78) and thirteen are local
  `segmask_floor/*` clips.

The three defects the original plan was written to fix are gone. No `_augment-N` copies survive. The
`floor_dataset__` duplicate twins are collapsed to one side (74 names carry the prefix, none of them
paired). The split is scene-disjoint by construction: the held half was field-stratified and
scene-disjoint already, the new half was assigned whole-video.

### Field breakdown as actually split

| field | train | train scenes | val | val scenes |
|---|---|---|---|---|
| cage6 | 9,209 | 11 | 1,684 | 2 |
| cage2 | 4,483 | 5 | 987 | 1 |
| mini_bot | 2,507 | 11 | 444 | 4 |
| mobilecam | 2,097 | 4 | 337 | 2 |
| **not in index** | **2,086** | — | **401** | — |
| cage5 | 1,945 | 3 | 223 | 4 |
| zed2023 | 1,452 | 2 | 645 | 1 |
| own_arena | 1,137 | 2 | 370 | 1 |
| own_room_floor | 1,083 | 4 | 199 | 2 |
| own_practice_table | 1,069 | 1 | 260 | 3 |
| skycam | 1,067 | 3 | 281 | 1 |
| cage7 | 443 | 4 | 66 | 1 |
| unclassified | 310 | 3 | 28 | 1 |
| cage3 | 84 | 1 | **0** | 0 |
| cage4 | 76 | 1 | 76 | 1 |
| cage1 | **1** | 1 | **0** | 0 |

**Cage-6 is now 31 % of the corpus rather than 32 %**, so the original conclusion stands unchanged:
global mean IoU is roughly a Cage-6 score, and it is the wrong model-selection metric. Everything
below scores **macro-average across field types** and **worst field**. `semantic_train.py
--select-metric macro` is the default and already does this.

### What the hull fix and validation removed

2,187 frames that are in `field_index.json` are no longer in the dataset. The losses are not evenly
spread:

| field | in index | kept | dropped |
|---|---|---|---|
| skycam | 1,976 | 1,348 | **628** |
| own_arena | 1,839 | 1,507 | 332 |
| cage6 | 11,217 | 10,893 | 324 |
| cage1 | 319 | **1** | **318** |
| own_practice_table | 1,628 | 1,329 | 299 |
| mobilecam | 2,604 | 2,434 | 170 |
| cage2 | 5,565 | 5,470 | 95 |
| others | | | 21 |

Two consequences worth stating plainly:

- **Cage-1 is dead as a field.** One frame survived. It cannot be scored and it should not appear in
  any per-field table.
- **Cage-3 has no eval representation.** 84 train frames, one scene, nothing in val. It stays in
  training and drops out of macro-average and worst-field, same as the original plan said, but now
  because it has no eval side rather than because it is small.

**Cage-4 is the free data point.** 76 train frames over one scene, and 76 eval frames over its own
scene. That is a Q1 answer sitting in the corpus before any ladder runs: if the model reaches
acceptable IoU on cage4 with 76 labelled frames of it, the per-field floor is already below 76 and
the Phase B ladder is mostly confirming a bound. Score it first. It costs nothing.

## Blockers before Phase A

These four are the whole reason this is not "ready to run".

### 1. Mask provenance is unverified, and it gates everything

The held corpus is literally named `deeplab_field_2026-07-28_hull_old`, and the convex hull fix
landed the next day in `a170f38` ("more efficient dataset validation. fix convex hull mask"). Mask
mtimes split 26,968 at 2026-07-28 and 8,082 at 2026-07-29.

- **Risk:** if the 26,968 older masks predate the hull fix, then 77 % of the corpus is annotated with
  the bug, and every IoU number in Phase A and Phase B is measuring the bug rather than the model.
- **Check:** re-run the hull generation on a sample of the 2026-07-28 masks and diff against what is
  on disk. If they differ, regenerate the whole corpus before anything else. The source tree
  `deeplab_field_2026-07-28_hull_old` is no longer on disk, so this may mean rebuilding from the
  original segmask exports.

Nothing below is worth running until this is settled.

### 2. `field_index.json` is stale in both directions

It carries 34,750 entries. 2,187 of those frames are gone from the dataset, and the 2,487 new frames
are absent from it.

Both `semantic_train.py` (per-field validation IoU, macro-average checkpoint selection) and
`score_masks.py` (`--by-field`) read this file and bucket anything missing as field `unknown`.
`score_masks.py --exclude-field` defaults to `unclassified` only, so **`unknown` gets a vote in the
macro-average**. Right now that vote is 401 val frames drawn from 16 unrelated videos, treated as if
it were one arena. It also means Phase A would select checkpoints partly on that bucket.

Fix: regenerate the index over the new dataset so the new material carries real field and scene
labels. `build_field_manifest.py` infers field from the filename, and the new material uses naming
schemes (`segmask_floor/*`, bare YouTube ids) that it does not yet parse. Expect to extend its
patterns or hand-label the 16 videos in `field_overrides.toml`.

### 3. There are no arms

Phase A needs `arms/full.txt`. Phase B needs `arms/lofo_<probe>_n<N>.txt`.

`make_field_splits.py` writes these, but it consumes a `manifest.json` and materializes the corpus
from scratch, including the split. Running it now would discard the preserved train/val assignment
that is the whole point of `build_field_dataset.py`, and the anchor scores would stop being
comparable.

Fix: split the arm emission out of `make_field_splits.py` so it can run against an already-split
corpus, reading `field_index.json` and the existing `train/` directory. The nesting assertion
(`50 ⊂ 150 ⊂ 400 ⊂ 1000 ⊂ all`) and the scene-spread draw order move with it. `PROBE_FIELDS` and
`LADDER` are already the right constants.

### 4. The corpus lives on the Desktop and is the only copy

The tree it was built from is gone. Move it under `training/data/` where every other dataset lives,
and archive it to `/media/storage/auto-battlebots-archive/` before touching it.

## Harness and training script: done

Both shipped in `666e669`. No further edits are prerequisites.

`score_masks.py` reports, per field type and macro-averaged:

- **field-class IoU**, the primary number.
- **boundary F1 at 2 / 5 / 10 px**. The mask feeds `PointCloudFieldFilter`, which fits the field plane
  and hands wall bounds to navigation. A mask that is 2 % wrong in the interior costs nothing; one
  that is 2 % wrong at the wall line moves the boundary. IoU alone will not see the difference.
- **worst-field IoU**, the metric that actually gates deployment.
- **paired bootstrap, 1000×**, the same statistical treatment as `model_eval/score.py`, so results are
  comparable in kind to the detector experiments.

It grades `.pth` checkpoints directly. Engine conversion is only needed for the final deployment
candidate; per-arm conversion would add ~20 runs of overhead to measure a quantisation effect this
experiment is not asking about.

`semantic_train.py` now has `--save-period`, `--train-list` / `--val-list`, `--amp bf16`,
`--select-metric {macro,worst,global}` defaulting to macro, `--field-index` and `--seed`.

One caveat carried in its own `--seed` help text: `seed_everything` leaves `cudnn.benchmark` on, so
two runs at the same seed can still diverge. The seed pair in Phase A therefore measures seed
variation *plus* kernel nondeterminism, which together is the run-to-run noise floor. That is the
right quantity for the gates below, but it is not a reproducibility guarantee.

`--train-list` matters more than it looks. Without it every arm needs a physical copy of ~29 k
images; 18 arms would be ~350 GB. Arms stay plain image lists and cost kilobytes.

## Throughput

Benchmarked on one A6000 (r50 / v3plus, 344 + 2×20 = 384 px), on a 29.5 k train set. The new train
split is 29,049, so these carry over directly.

| config | img/s | epoch @ 29 k train |
|---|---|---|
| fp32, batch 8 (old default) | 101 | 4.9 min |
| fp32, batch 16 | 104 | 4.7 min |
| **bf16 autocast, batch 32** | **174** | **2.8 min** |
| dataloader alone, 32 workers | 623 | — |

Two things follow. **The GPU is the bottleneck, not the input pipeline**: the loader can feed 6× what
the model consumes, so batch size and worker count are not where the time goes. And **bf16 autocast
is a free 1.67×**.

There are 3× A6000, so three arms run concurrently. `semantic_train.py` is single-GPU; no DDP is
needed, just one process pinned per GPU.

---

## Phase 0b: reconcile the corpus (no training)

1. **Archive first.** `rsync` the Desktop tree to `/media/storage/auto-battlebots-archive/`, then move
   it to `training/data/deeplab_field_2026-07-29/`.
2. **Settle the hull question** (blocker 1). Regenerate if the older masks are pre-fix.
3. **Regenerate `field_index.json`** over the full 35,050 frames, including scene and field for the 16
   new videos (blocker 2).
4. **Emit arms** from the existing split (blocker 3). Assert nesting.
5. **Eyeball masks per field.** `view_mask.py` on ~30 random frames per field, and all of the new
   material. A systematically wrong mask in a rare field masquerades as "this field needs more data".
6. **Score the deployed model** (`field_deeplabv3p_r50_2026-04-29.pth`) on the new val set, with and
   without the new material, `--by-field`. This is the anchor everything else is compared against, and
   the with/without pair says whether the 2,487 new frames changed the eval set's difficulty.
7. **Score cage4 specifically.** 76 train frames, its own eval scene. Read it as a preliminary Q1
   answer before spending the Phase B budget.

---

## Phase A: when do I stop training? (Q2)

**2 runs on the full corpus, cold start, 150 epochs, checkpoint every 10.**

Two runs, not one, differing **only by seed**. This is the same discipline that made `data_scaling`
readable: that series found a 0.048 recall spread between two nominally identical detector runs,
larger than most of the effects people wanted to claim. **The seed pair measures the noise floor δ,
and δ is what makes every gate in Phase B meaningful.** Skipping it makes Phase B's ladder
uninterpretable.

Answer: the earliest checkpoint whose **macro-average IoU** is within δ of that arm's best, holding
worst-field IoU within δ too. Reported as an epoch count, with the caveat that it is a count for
*this* corpus size.

Before committing the budget, run the **bf16 sanity check**: seed-A arm at both precisions for 20
epochs. If final IoU moves, the whole budget rebuilds at fp32 (~1.67× everything).

Cost: 150 epochs × 2.8 min ≈ **7 h per run**, 2 runs concurrent on 2 GPUs ≈ **7 h wall**.

---

## Phase B: how many images per field type? (Q1)

**Leave-one-field-out with an add-back ladder.** Hold a probe field entirely out of training, then add
back N labelled frames of it and watch IoU on that field's held-out scenes.

### Probe fields, with the split as it now stands

| probe | train frames | train scenes | eval frames | eval scenes | note |
|---|---|---|---|---|---|
| **cage2** | 4,483 | 5 | 987 | **1** | primary probe, but graded on one recording |
| **cage5** | 1,945 | **3** | 223 | 4 | replication; thin draw pool, better eval spread |
| **zed2023** | 1,452 | **2** | 645 | **1** | deployment camera. Most decision-relevant, weakest holdout |

Cage-6 is unusable as a probe: removing it removes a third of the corpus, so the base arm would differ
from every other arm in size as well as field coverage, the exact confound that made the earlier
data-floor result unusable.

Two of these got weaker than the original plan assumed, because the field-stratified split put scenes
where it put them:

- **cage2 has a single eval scene.** The headline ladder is graded on one recording, so a quirk of
  that recording moves the whole curve. Consider re-splitting cage2 to put two scenes in eval before
  running, accepting that it breaks anchor comparability for that field only.
- **cage5 has three train scenes**, so "scene-spread" sampling has almost nothing to spread over. At
  N=50 that is ~17 frames per scene, which is a reasonable spread; at N=1000 the rungs are mostly
  adding near-neighbours from the same three recordings. Expect the cage5 curve to flatten early for
  reasons that are about scene count, not frame count.
- **zed2023 has two train scenes and one eval scene.** Report it as indicative only. It is included
  because it is the only probe that answers the question *for the camera the robot actually uses*, and
  a weak reading there beats a strong reading on footage the robot will never see.

### Ladder

N ∈ **{0, 50, 150, 400, 1000, all}** frames of the probe field, drawn scene-spread. `data_scaling`
found scene diversity beat frame count for the detector and the gap widened as data shrank; for a
fixed field the analogous choice is to spread the budget across that field's recordings.

**Rungs are nested** (`50 ⊂ 150 ⊂ 400 ⊂ 1000 ⊂ all`) so a drop between rungs cannot be blamed on which
frames were drawn. Everything that is *not* the probe field is held fixed across all rungs.

For cage5 the top two rungs are 1,000 and 1,945, a factor of 1.9. For zed2023, 1,000 and 1,452. Both
are tighter than cage2's 1,000 to 4,483, so read the top of those two ladders as one point, not two.

### Fine-tune, with a cold-start control

Each rung **fine-tunes from its own N=0 LOFO base checkpoint** for 30 epochs, rather than cold-starting.

Two reasons. It is the operationally real question: you land at an event, label some frames of the new
cage, and retrain overnight; nobody cold-starts. And it costs ~2.5× less, which is what makes
replication across two probe fields affordable at all.

The risk is that the fine-tune curve is an artifact of where the base checkpoint sits rather than a
statement about data volume. **Control: cold-start cage2 at N=150 and N=all.** If those land within δ
of their fine-tuned counterparts, the ladder reads as a data-volume result. If not, the ladder is
reported as a fine-tuning result only, which is still useful but a narrower claim.

### Arms and cost

| | runs | epochs | cost |
|---|---|---|---|
| LOFO bases (cage2, cage5, zed2023) | 3 | 150 | 3 × 7 h |
| cage2 ladder (5 rungs above N=0) | 5 | 30 | 5 × 1.4 h |
| cage5 ladder (5 rungs) | 5 | 30 | 5 × 1.4 h |
| zed2023 spot check (N ∈ 150, 400, all) | 3 | 30 | 3 × 1.4 h |
| cold-start controls | 2 | 150 | 2 × 7 h |
| **total** | **18** | | **~53 GPU-h ≈ 18 h wall on 3 GPUs** |

Phase A + Phase B ≈ **25 h wall**. A weekend, unattended.

**If that is too much**, drop the zed2023 spot check and the cage5 ladder (−9 runs, ~10 h). The cost is
replication: the answer then rests on one probe field graded on one eval scene, which given cage2's
single-scene eval is a real weakness, not a formality.

### Gate

For each rung, the smallest N whose probe-field IoU is within δ of the N=all ceiling, with boundary F1
not significantly worse. Reported as *"N frames of a new field reaches within δ of saturation; below
that, X."*

The N=0 rung is interesting in its own right: it measures **how well the model generalises to a
completely unseen field**. If N=0 already clears the gate, the answer to Q1 is "zero, the model
transfers" and the rest of the ladder is confirmation. Cage-4's 76-frame result from Phase 0b step 7
is the cheap preview of this.

---

## Phase C: downstream confirmation

Mask IoU is a proxy. The mask's real job is to feed `PointCloudFieldFilter` a plane fit and give
navigation its wall bounds, so the winning checkpoint gets one replay before anyone believes it:
convert to TensorRT, replay an SVO, and compare field-fit stability and wall bounds against the
deployed model. A checkpoint that wins on IoU but destabilises the plane fit has not won.

---

## Deliverable

1. **A per-field saturation curve**: IoU vs N labelled frames of that field, one line per probe field,
   against a horizontal "field held out entirely" line and a "field fully represented" ceiling.
2. **An epoch-vs-macro-IoU curve** with the noise band from the seed pair drawn on it, and the epoch
   where it flattens.
3. **Two numbers** for `my_takeaways.md`, in the same one-line form as the existing entries.
4. **A corpus in the tree**: `training/data/deeplab_field_2026-07-29/` with a regenerated index and
   arms, in the same role `nhrl_robots_bbox_2class` now plays for the detector. This outlives the
   experiment and is arguably worth more than either answer.

## Risks / caveats

- **Mask provenance is the top risk.** See blocker 1. 77 % of the corpus carries a pre-fix mtime and
  the source tree is gone. If the hull bug is baked in, every number here measures it.
- **2,487 frames are unindexed and currently vote as a field called `unknown`.** See blocker 2. This
  affects Phase A checkpoint selection, not just reporting.
- **The new material is unaudited.** The 16 new videos went through `build_field_dataset.py`'s label
  space check (sampled masks only, 40 frames) and nothing else. They have not been through
  `validate_segmask_dataset.py` and no one has eyeballed them. `data_scaling`'s headline finding was
  that data hygiene mattered more than data volume; that applies here.
- **cage2, the primary probe, has one eval scene.** Stated above. It caps how strongly the headline
  can be worded.
- **Field type is inferred from filenames, not verified.** `Cage-6-Red` and `Cage-6-Red-High` are the
  same cage from different mounts; `Cage-6-Overhead-High` may or may not be the same physical floor
  across a two-year span of tournaments (`jun24` to `feb26`). If NHRL repainted or re-floored a cage,
  one label spans two visually distinct fields. Spot-check one frame per scene before trusting the
  grouping.
- **The domain gap is the elephant, same as in `data_scaling`.** ~66 % of this corpus is NHRL broadcast
  footage (cages, SkyCam, MobileCam). The deployed model runs on the robot's own ZED at floor level. A
  saturation curve measured on broadcast fields may not transfer to the deployment viewpoint at all.
  This is why the zed2023 probe is in Phase B despite its weak holdout, and it is the strongest
  argument for spending the *next* labelling budget on ZED footage rather than on more cages. That 66 %
  figure will move once the 2,487 unindexed frames are labelled.
- **"New field" here is a held-out field from the same corpus**, sharing capture pipeline, resolution
  and era with the training fields. A genuinely new cage at a future event will be harder than any
  probe measured here, so treat the resulting N as a **lower bound**.
- **One seed per Phase-B arm.** δ comes from the Phase A pair and is assumed to hold at the smaller arm
  sizes, which is an assumption, not a measurement. Noise usually grows as data shrinks. Rungs
  differing by less than δ are not separable.
- **Fine-tuning confounds Phase B** unless the cold-start controls agree. Stated above; repeating
  because it determines how the headline may be worded.
- **bf16 is assumed loss-neutral.** To be checked over 20 epochs, not 150. If the seed-A fp32/bf16 pair
  diverges late, the budget rebuilds at fp32.
- **Boundary F1 has no established threshold** in this project the way mAP 0.5 does for the detector.
  The first run produces a number nobody can yet calibrate; the deployed-model anchor is what gives it
  meaning, which is why Phase 0b step 6 is not optional.
- **cage1 (1 frame) and cage3 (84 train, 0 eval) cannot be scored.** They stay in training and drop out
  of macro-average and worst-field. cage4 (76/76) can be scored and should be.

## Artifacts / locations

- Corpus (current, Desktop, only copy): `/home/ben/Desktop/deeplab_field_2026-07-29/`
- Corpus (target): `training/data/deeplab_field_2026-07-29/`
- Archive: `/media/storage/auto-battlebots-archive/deeplab_field_2026-07-29/`
- Arms: `training/data/deeplab_field_2026-07-29/arms/*.txt` (image lists, not copies)
- Runs: `training/data/deeplab_field_2026-07-29/runs/<arm>/`
- Scores: `training/data/deeplab_field_2026-07-29/scores/`
- Baseline anchor: `data/models/field_deeplabv3p_r50_2026-04-29.pth` (r50 / v3plus / 344 / pad 20)
- Writeup: `docs/experiments/perception_performance/deeplab_field_data_<date>.md`

Scripts, all present:

- `training/deeplab/build_field_manifest.py` (needs new naming patterns, blocker 2)
- `training/deeplab/make_field_splits.py` (needs a split-preserving arm mode, blocker 3)
- `training/deeplab/build_field_dataset.py` (built the current corpus)
- `training/deeplab/split_by_scene.py` (not used on this corpus; the split came from the held half)
- `training/deeplab/score_masks.py`
- `training/deeplab/validate_segmask_dataset.py`
- `training/deeplab/convex_hull_masks.py` (blocker 1)

---

## Commands

```bash
source scripts/activate_python.sh
DS=training/data/deeplab_field_2026-07-29
```

### Phase 0b: reconcile the corpus

```bash
# 1. Archive, then move into the tree. This is currently the only copy.
rsync -a --info=progress2 /home/ben/Desktop/deeplab_field_2026-07-29/ \
    /media/storage/auto-battlebots-archive/deeplab_field_2026-07-29/
mv /home/ben/Desktop/deeplab_field_2026-07-29 "$DS"

# 2. Hull provenance. Regenerate a sample and diff against what is on disk.
#    If they differ, the corpus is annotated with the pre-fix hull and must be rebuilt.
python3 training/deeplab/convex_hull_masks.py --help   # confirm the sample/diff entry point

# 3. Regenerate the field index over all 35,050 frames.
python3 training/deeplab/build_field_manifest.py "$DS" --out "$DS/manifest.json"
#    Expect the 16 new videos to land as 'unclassified'. Add their patterns to
#    field_labels.py or pin them in field_overrides.toml, then re-run.

# 4. Emit arms from the existing split (needs the split-preserving mode).
python3 training/deeplab/make_field_splits.py \
    --manifest "$DS/manifest.json" --out "$DS" --probes cage2,cage5,zed2023

# 5. Eyeball masks per field, and all of the new material.
python3 training/deeplab/view_mask.py <image> <mask>

# 6. Baseline anchor: deployed model on the new val set.
python3 training/deeplab/score_masks.py "$DS/val" \
    --candidate deployed=data/models/field_deeplabv3p_r50_2026-04-29.pth \
    --by-field --boundary-tol 2,5,10 --bootstrap 1000 \
    --output "$DS/scores/anchor"

# 7. Cage-4 preview: 76 train frames, its own eval scene. Preliminary Q1 answer.
python3 training/deeplab/score_masks.py "$DS/val" \
    --candidate deployed=data/models/field_deeplabv3p_r50_2026-04-29.pth \
    --field-filter cage4 --boundary-tol 2,5,10 --bootstrap 1000 \
    --output "$DS/scores/cage4_preview"
```

### Phase A: epoch ladder

```bash
cd training/deeplab
DS=../data/deeplab_field_2026-07-29

# bf16 sanity check before spending the budget: same seed, 20 epochs, both precisions.
python3 semantic_train.py "$DS" --train-list "$DS/arms/full.txt" \
    -b 32 -nw 16 -ne 20 --backbone r50 --decoder v3plus --amp bf16 \
    -o "$DS/runs/amp_check_bf16"
python3 semantic_train.py "$DS" --train-list "$DS/arms/full.txt" \
    -b 16 -nw 16 -ne 20 --backbone r50 --decoder v3plus \
    -o "$DS/runs/amp_check_fp32"

# Seed pair, full corpus, 150 epochs, ladder every 10. One per GPU, concurrent.
CUDA_VISIBLE_DEVICES=0 python3 semantic_train.py "$DS" --train-list "$DS/arms/full.txt" \
    -b 32 -nw 16 -ne 150 --save-period 10 --seed 4176 \
    --backbone r50 --decoder v3plus --amp bf16 --select-metric macro \
    -o "$DS/runs/phaseA_seedA" &
CUDA_VISIBLE_DEVICES=1 python3 semantic_train.py "$DS" --train-list "$DS/arms/full.txt" \
    -b 32 -nw 16 -ne 150 --save-period 10 --seed 9001 \
    --backbone r50 --decoder v3plus --amp bf16 --select-metric macro \
    -o "$DS/runs/phaseA_seedB" &
wait

# Grade the whole ladder in one paired invocation, which gives delta directly.
cd /home/ben/auto-battlebot
python3 training/deeplab/score_masks.py training/data/deeplab_field_2026-07-29/val \
    --candidate deployed=data/models/field_deeplabv3p_r50_2026-04-29.pth \
    --candidate-glob 'training/data/deeplab_field_2026-07-29/runs/phaseA_seed*/ckpt_ep*.pth' \
    --baseline deployed --by-field --boundary-tol 2,5,10 --bootstrap 1000 \
    --output training/data/deeplab_field_2026-07-29/scores/phaseA
```

### Phase B: LOFO add-back ladder

```bash
cd training/deeplab
DS=../data/deeplab_field_2026-07-29

# LOFO bases: probe field removed entirely. 3 runs, one per GPU.
for i in 0 1 2; do
  probe=$(echo "cage2 cage5 zed2023" | cut -d' ' -f$((i+1)))
  CUDA_VISIBLE_DEVICES=$i python3 semantic_train.py "$DS" \
      --train-list "$DS/arms/lofo_${probe}_n0.txt" \
      -b 32 -nw 16 -ne 150 --save-period 10 --seed 4176 \
      --backbone r50 --decoder v3plus --amp bf16 --select-metric macro \
      -o "$DS/runs/lofo_${probe}_n0" &
done; wait

# Add-back rungs: fine-tune from that probe's own N=0 base, 30 epochs each.
# 13 runs total (5 + 5 + 3); 3 at a time.
for probe in cage2 cage5; do
  for n in 50 150 400 1000 all; do
    python3 semantic_train.py "$DS" \
        --train-list "$DS/arms/lofo_${probe}_n${n}.txt" \
        -c "$DS/runs/lofo_${probe}_n0/model_r50.pth" \
        -b 32 -nw 16 -ne 30 --save-period 10 --seed 4176 \
        --backbone r50 --decoder v3plus --amp bf16 --select-metric macro \
        -o "$DS/runs/lofo_${probe}_n${n}"
  done
done

for n in 150 400 all; do
  python3 semantic_train.py "$DS" \
      --train-list "$DS/arms/lofo_zed2023_n${n}.txt" \
      -c "$DS/runs/lofo_zed2023_n0/model_r50.pth" \
      -b 32 -nw 16 -ne 30 --save-period 10 --seed 4176 \
      --backbone r50 --decoder v3plus --amp bf16 --select-metric macro \
      -o "$DS/runs/lofo_zed2023_n${n}"
done

# Cold-start controls: does the fine-tune ladder read as a data-volume result?
for n in 150 all; do
  python3 semantic_train.py "$DS" \
      --train-list "$DS/arms/lofo_cage2_n${n}.txt" \
      -b 32 -nw 16 -ne 150 --save-period 10 --seed 4176 \
      --backbone r50 --decoder v3plus --amp bf16 --select-metric macro \
      -o "$DS/runs/cold_cage2_n${n}"
done

# Score each probe against its own held-out scenes.
cd /home/ben/auto-battlebot
for probe in cage2 cage5 zed2023; do
  python3 training/deeplab/score_masks.py training/data/deeplab_field_2026-07-29/val \
      --field-filter ${probe} \
      --candidate-glob "training/data/deeplab_field_2026-07-29/runs/*_${probe}_n*/ckpt_ep*.pth" \
      --baseline lofo_${probe}_n0 --by-field --boundary-tol 2,5,10 --bootstrap 1000 \
      --output training/data/deeplab_field_2026-07-29/scores/lofo_${probe}
done
```

### Phase C: downstream check on the winner

```bash
python3 training/deeplab/convert_to_tensorrt.py \
    training/data/deeplab_field_2026-07-29/runs/<winner>/model_r50.pth \
    -o data/models/field_deeplabv3p_r50_<date>_x86_64_sm86.engine --keep-onnx

# Replay against the deployed model; compare field-fit stability and wall bounds.
./scripts/build_and_run.sh -c config/playback/<profile>.toml
```

### Validation before committing anything

```bash
./scripts/lint
```
