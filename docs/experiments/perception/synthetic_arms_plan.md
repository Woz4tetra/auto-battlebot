# Synthetic vs real opponent detector — three clean arms (plan)

Replaces the retracted `synthetic_plus_bbox_2026-07-22.md`. That experiment compared a "real" baseline
that was **34.6 % BlenderProc renders** against a mix that was **51.8 % renders**, on a val split that
was **35 % renders** and frame-level random. Every number in it about synthetic-vs-real is unusable.

This plan re-asks the same question with datasets whose provenance is verified *before* training, and
keeps training short (100 epochs) so the answer arrives in days, not weeks.

## Question

Does synthetic opponent data help, hurt, or do nothing to a real-data opponent detector — measured
against a baseline that contains **zero** synthetic frames?

Three arms, one variable (training-set provenance). Everything else — architecture, epochs, batch,
imgsz, seed, val split, grading — is held identical.

| arm | train set | expected train frames |
|---|---|---|
| `real_only` | audited real corpus, unchanged | 25,914 |
| `synth_only` | renders only, no real frames | 17,995 |
| `mixed` | real + all renders | 43,909 |

## Vocabulary and why it is 2-class

All three use the canonical `nhrl_robots_bbox_2class` vocabulary: `["robot", "house_bot"]`, where
`robot` means anything in the field that is not the house bot. This is the corpus the project has
standardized on, it is documented as **zero synthetic**, and its val split is **scene-disjoint** —
the two properties the retracted experiment lacked.

**Consequence to design around:** the synthetic source contains no house bot. `synth_only` therefore
cannot predict `house_bot` at all, and its `house_bot` metrics will be zero by construction, not by
failure. House-bot regression is only comparable between `real_only` and `mixed`. The decision rules
below score on `robot`/opponent metrics for exactly this reason.

## Datasets to build

### A. `real_only` — the audited real corpus

`training/data/nhrl_robots_bbox_2class` is already this dataset: 32,487 frames, 71 scenes, zero
synthetic, scene-disjoint val (train 25,914 / val 6,573; `robot` 56,293 / 19,541,
`house_bot` 19,541 / 4,806). Provenance: 56 per-scene real segmentation datasets from
`/media/storage/auto-battlebots-archive/nhrl_seg_indiv_labeled` → `remap_labels_by_name.py` with
`remap_config_2class.toml` → manual validation → scene-stratified 80/20 carve.

**No rebuild. Use it as-is**, and run the audit in the gate below to confirm it is still clean.

### B. `synth_only` — renders with the real frames removed

Source: `training/data/all_robot_keypoints` (BlenderProc `synthgen`, 3 classes
`[mr_stabs_mk2, mrs_buff_mk3, nhrl_robot]`). It is **not** purely synthetic — it carries 497 real
frames of our own robot (452 train / 45 val, filenames `mrs-buff-mk3-keypoints*`). Those must be
dropped, or this arm reproduces the exact defect being corrected.

```bash
# 1. keypoints -> boxes, all three synthetic classes folded into `robot` (class 0)
python training/yolo/pose_to_bbox.py training/data/all_robot_keypoints \
    -o training/data/synth_only_2class \
    --class-map 0:0,1:0,2:0 \
    --names-from training/data/nhrl_robots_bbox_2class/data.yml

# 2. drop the real frames and their labels — this arm must be 100% rendered
find training/data/synth_only_2class -name 'mrs-buff-mk3-keypoints*' -delete

# 3. expect 17,995 train / 2,004 val / 1 test, and zero non-`synthetic__` filenames
```

### C. `mixed` — real train + all renders, real val

```bash
python training/yolo/pool_datasets.py \
    --real training/data/nhrl_robots_bbox_2class/train \
    --synth training/data/synth_only_2class/train \
    --synth-fraction 1.0 --synth-order random \
    -o training/data/mixed_2class/train
```

`pool_datasets.py` prefixes every filename with its source, which is what makes an arm auditable after
the fact — the retracted experiment was diagnosable only because its pooled copy survived.

**Val is the real scene-disjoint val, identical across all three arms.** No synthetic in any val. All
three train against the same 6,573 real frames so the loss curves are comparable, and `synth_only`'s
val doubles as a live read on sim-to-real transfer.

## Provenance gate — run before any training

The experiment does not start until this table is reproduced and pasted into the report. This is the
step whose absence invalidated the previous run.

| check | `real_only` | `synth_only` | `mixed` |
|---|---|---|---|
| train frames | 25,914 | 17,995 | 43,909 |
| filenames matching `synthetic` | **0** | 17,995 (all) | 17,995 |
| real-footage frames | 25,914 | **0** | 25,914 |
| val frames (real, scene-disjoint) | 6,573 | 6,573 | 6,573 |
| synthetic frames in val | **0** | **0** | **0** |

```bash
for d in nhrl_robots_bbox_2class synth_only_2class mixed_2class; do
  echo "== $d"
  for s in train val; do
    n=$(ls training/data/$d/$s/images 2>/dev/null | grep -c '\.jpg$')
    syn=$(ls training/data/$d/$s/images 2>/dev/null | grep -c 'synthetic')
    echo "  $s: $n frames, $syn synthetic"
  done
done
python training/yolo/validate_yolo_integrity.py training/data/<each> --strict
```

Also confirm no eval leakage: no `nhrl_keypoints_eval_test` recording may appear as a training scene.

## Training

Identical for all three, per the 2class README's standard recipe:

```bash
python3 train.py ../data/<dataset>/data.yml yolo26n -e 100 --save-period 25 --seed 0
```

100 epochs, not 500: `data_scaling_2026-07-27` found the arms converged well before 100, and three
arms at 500 epochs would cost ~60 h for a question this plan can answer in a fraction of that.

**Known confound to state in the report, not to hide:** at fixed epochs, `mixed` sees 1.7× the frames
per epoch, so it gets 1.7× the gradient steps. If `mixed` wins by a small margin, that margin is not
cleanly attributable to the synthetic data. Two ways to settle it, in order of cost: report
`--save-period 25` checkpoints so the arms can be compared at matched *steps* rather than matched
epochs; or add a fourth arm, `real_only` at 170 epochs. Decide before running, not after seeing the
result.

## Grading

**Primary — the independent eval.** `nhrl_keypoints_eval_test` (372 hand-reviewed frames of unseen
real fights), scored by `score.py` with `taxonomy_merged.yaml` (the 2-class scoring view: GT
`mr_stabs_mk2`/`mrs_buff_mk3` → `opponent`, `object` excluded), conf 0.5, paired bootstrap 1000×.
This is real, unseen, and hand-corrected — the only number that decides anything.

**Secondary — the scene-disjoint real val.** `model.val()` on `nhrl_robots_bbox_2class/val`. Reported
for training health only. The retracted experiment's central lesson is that a same-corpus val split
told the opposite story from the independent eval; this one is at least scene-disjoint, but it still
does not decide.

Report per arm: agnostic recall / precision / F1 (bootstrapped, with CIs), `robot` AP50-95,
`house_bot` AP50-95 (`real_only` vs `mixed` only), and wrong-class rate.

## Decision rules — fixed in advance

- **Adopt synthetic** if `mixed` beats `real_only` on agnostic recall with a 95 % CI excluding 0 on the
  independent eval, and `house_bot` AP does not drop more than 0.02.
- **Reject** if `mixed` is worse with a CI excluding 0.
- **Neutral / keep real-only** if the CI includes 0 — and say so plainly. This is the likeliest outcome
  and it is a real result, not a failed experiment.
- `synth_only` is the calibration arm, not a candidate. Its job is to measure how far generic renders
  get on their own; a near-zero score there bounds how much the `mixed` arm could possibly have gained.

## What to expect, and why the arms are still worth running

The corrected cut-paste probe (see the retracted report's 2026-07-31 note) found the detector needs a
robot embedded in a scene *coherent with it*: pasting a real opponent onto a different real arena
frame recovers only 18 % of what stripping context away removes. The synthetic renders place robots in
HDRI living rooms and on plain carpet — no cage, no floor logo, no overhead framing. So the prior is
that `synth_only` scores poorly and `mixed` lands neutral.

The value here is a **trustworthy** measurement of that, on arms that are what their names say. If the
result is neutral again, it will be the first time that conclusion is actually supported.

## Deliverables

1. `synth_only_2class` and `mixed_2class` datasets, each with a README carrying the gate table (the
   `nhrl_robots_bbox_2class` README is the template).
2. Three 100-epoch `yolo26n` runs, checkpoints every 25 epochs.
3. `score.py` output for all three on the independent eval, plus the paired-bootstrap table.
4. A report, `synthetic_arms_<date>.md`, opening with the reproduced provenance gate.
