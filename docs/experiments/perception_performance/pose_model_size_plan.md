# Does model size matter for the keypoint model - or does the corpus?

Supersedes the version of this plan executed as `pose_model_size_2026-09-05.md`. That sweep
trained `yolo26{n,s,x}-pose` on `all_robot_keypoints` and scored the deployed keypoint model
beside them as a reference. It found `x` better than `n` on every keypoint metric and the
first all-robots pose model to beat the deployed one, and it rejected `x` on latency.

The reference is the problem. `yolo26n-pose_our_robots_2026-05-01` was not trained on
`all_robot_keypoints`. Its checkpoint records `data: training/data/our_robot_keypoints/data.yml`,
a 2-class corpus that was not in the repo when that sweep ran. So the one comparison the
report leaned on for "which pose model should ship" varied model size, class vocabulary,
corpus size, epoch count and fine-tune lineage at once. This revision holds size and
schedule fixed and adds the corpus as a factor.

## What changed and why

`x` is out on latency: `+3.29 ms` of dev-box GPU time against roughly 1 ms of Jetson tick
headroom, and the keypoint model is already the slower of the two parallel branches. That
verdict does not depend on the corpus. So the deployable question is no longer "how big",
it is **which corpus produces the best `n`-sized pose model**, and the size sweep becomes
the secondary axis rather than the primary one.

## Questions

1. **Primary.** At `yolo26n-pose`, the only size that fits the tick budget, does training on
   `our_robot_keypoints` give lower `kp_heading_err_deg` than training on
   `all_robot_keypoints`? This is arm D against arm A, matched on everything but corpus.
2. **Secondary.** Does the size effect the previous sweep measured on `all_robot_keypoints`
   (nothing at `s`, a large gain at `x`) reproduce on `our_robot_keypoints`, or is it a
   property of that corpus? This is the D/E/F row against the A/B/C row.
3. **Control.** How much of the deployed model's standing is its corpus, and how much is its
   500-epoch schedule and its fine-tune lineage? Arm D is the matched-schedule counterpart
   the previous report lacked.

## Datasets

### `our_robot_keypoints` - what the deployed model was trained on

| | |
|---|---|
| Path | `training/data/our_robot_keypoints`, 6.9 GB - restored from `/media/storage/auto-battlebots-archive` |
| Size | 31,912 train / 3,530 val / 1 test frame; 46,878 boxes |
| Classes | `mr_stabs_mk2` (16,474 boxes), `mrs_buff_mk3` (30,404) - `nc: 2` |
| Keypoints | `kpt_shape: [2, 3]`, `flip_idx: [0, 1]` - front and back, same as the other corpus |
| Composition | 98.7% synthetic by box count; 497 real frames / 605 real boxes |
| Visibility flags | 61.1% flag `2`, 34.9% flag `1`, 4.0% flag `0` |

Class balance is 1.85:1 toward `mrs_buff_mk3`, against 3.3:1 toward `nhrl_robot` in
`all_robot_keypoints`, so the minority class carries 16,474 boxes here against 12,011 there.
Per-class `mr_stabs_mk2` metrics are less noisy on this corpus, not more.

### The two corpora share every real frame

The same 497 real frames appear in both, and they are the only real frames in either. The
2026-05-01 recording is named `2026-05-01T14-17-26-*` here and
`mrs-buff-mk3-keypoints-part-2__2026-05-01T14-17-26-*` in `all_robot_keypoints`, which is
why a plain filename join finds only 362 of them.

That pins down what the corpus contrast actually varies:

| | `our_robot_keypoints` | `all_robot_keypoints` |
|---|---:|---:|
| classes | 2 | 3 |
| synthetic frames | 34,945 | 19,999 |
| real frames | 497 | 497 |
| train frames | 31,912 | 18,447 |

Two things move together, class vocabulary and synthetic volume, and 1.73x more training
frames could account for a win on its own. Arm G below separates them.

### Both val splits are random frame-level splits

Checked for this plan. The `our_robot_keypoints` val synthetic indices run 34 to 34,943,
interleaved throughout the train range 0 to 34,945, with no index reused. That is the same
carve `all_robot_keypoints` has, and the previous report showed it ranks `s` second where
the eval set ranks it last.

**Do not rank arms on val, and do not use `best.pt`,** which is selected by val fitness.
Grade on `nhrl_keypoints_eval_test` with `score.py`, as before. Write a `README.md` for
`our_robot_keypoints` recording the carve, the way `all_robot_keypoints` now has one.

## The corpus is restored

`training/data/our_robot_keypoints` now holds the copy from
`/media/storage/auto-battlebots-archive`: 6.9 GB, 31,912 train / 3,530 val / 1 test frame,
images and labels paired in every split. `check_det_dataset` loads it from the new location
and reports `nc: 2`, `names {0: mr_stabs_mk2, 1: mrs_buff_mk3}`, `kpt_shape [2, 3]`,
`flip_idx [0, 1]`. Nothing further is needed before queueing arm D.

Three things about the restored tree worth knowing:

- **The `data.yml` looks broken and is not.** It has no `path:` key and uses the Roboflow
  `train: ../train/images` form, unlike `all_robot_keypoints`, which carries an absolute
  `path:`. Ultralytics strips the leading `../` and resolves against the yaml's own
  directory. Leave it alone.
- **Keep the copied `labels.cache` files.** They were written in May 2026 when the corpus
  last lived at this path, so their stored `im_file` entries already point at
  `/home/ben/auto-battlebot/training/data/our_robot_keypoints/...`, their format is the
  current `DATASET_CACHE_VERSION` 1.0.3, and the hash check passes for both splits. Verified
  with `get_hash(img2label_paths(im) + im)` against each cache. Ultralytics will reuse them
  and skip a 35,442-file label scan. An earlier draft of this plan said to delete them as
  stale; that was wrong.
- **`mrs-buff-mk3-keypoints-part-2/` and its 86 MB zip came along**, 171 MB in total. That is
  the Roboflow staging export for the 135 real frames already merged into `train/` and
  `val/`, and no training or scoring path reads it. It carries its own nested `data.yaml`,
  so do not point `train.py` or `score.py` at that subdirectory by mistake;
  `resolve_dataset` only looks for `data.yml`/`data.yaml` at the top level, so the directory
  form in the commands below is safe.

**Disk is the remaining constraint.** `train.py` defaults to `cache="disk"`, which writes one
2,764,928-byte `.npy` per frame. 35,442 frames is **98 GB of cache**, against 172 GB free on
`/` after the copy. `all_robot_keypoints` is holding 48 GB of `.npy` next to its images and
its arms are finished, so reclaim that before queueing:

```bash
venv/bin/python training/yolo/clear_image_cache.py --older-than 0 --dry-run   # then drop --dry-run
```

`train.py` sweeps caches unused for 7 days automatically and spares the dataset it is about
to train on, but the 2026-09-05 caches are not yet old enough for that to fire.

## Arms - a 2x3 grid, half of it already run

| arm | model | corpus | status |
|---|---|---|---|
| A | `yolo26n-pose` | `all_robot_keypoints` | done, `2026-09-05` |
| B | `yolo26s-pose` | `all_robot_keypoints` | done, `2026-09-05` |
| C | `yolo26x-pose` | `all_robot_keypoints` | done, `2026-09-06` |
| **D** | `yolo26n-pose` | `our_robot_keypoints` | **new, run first** |
| **E** | `yolo26s-pose` | `our_robot_keypoints` | **new** |
| **F** | `yolo26x-pose` | `our_robot_keypoints` | **new, conditional** |
| G | `yolo26n-pose` | `our_robot_keypoints`, `--fraction 0.578` | optional, see below |

A, B and C are reused as trained. Their engines are
`data/models/yolo26{n,s,x}-pose_all_robot_keypoints_2026-09-05_last_x86_64_sm86.engine` and
their scores are under `training/data/nhrl_keypoints_eval_test/scores_pose_size_abc/`. Do
not retrain them; the schedule below is theirs.

**F was conditional and is no longer.** `x` cannot deploy at the current tick budget whatever
it is trained on, so at 19.3 h its only value is telling us whether the `n` -> `x` jump is a
corpus property or a size property. The plan gated it on D or E showing a corpus effect.

Ungated 2026-09-07, before any of D, E or F had a result, on the grounds that `x` has twice
now broken the size pattern the smaller arms establish: `model_size_2026-09-04.md` found `s`
through `l` tied and only `x` moved, and `pose_model_size_2026-09-05.md` found `s` no better
than `n` and `x` better than both on every keypoint metric. A gate that reads D and E as
evidence about F assumes the very monotonicity those two reports found absent. F is queued
with D and E.

**G separates vocabulary from volume.** `--fraction 0.578` subsamples `our_robot_keypoints`
to 18,447 train frames, matching A. If D beats A and G does not, the win is training volume
and the fix is more synthetic data on either corpus rather than a class-vocabulary change.
Note `--fraction` cuts real and synthetic frames alike, so G keeps about 287 of the 497 real
frames; it is a volume control, not a real:synthetic control.

`train.py` already carries all three pose sizes. `yolo26s-pose` was added for the previous
sweep and needs nothing further.

## Design

| | |
|---|---|
| Epochs | 200, `--save-period 50`, matching A/B/C exactly |
| Batch | 96 (`-b 96`, 32/GPU across 3 GPUs) |
| imgsz | 640 |
| Devices | `-d 0 1 2`, submitted through `training/gpu_queue.py` |
| Seed | 0, single seed |
| Endpoint | epoch 200 (`last.pt`) for every arm |

**Do not shorten the schedule for the new arms.** The previous report found keypoint
placement plateaus by epoch 100 and that a 100-epoch run would have reached the same PCK,
which is a fair argument for 100 epochs in a fresh experiment. It is not available here: A,
B and C ran to 200, and a corpus contrast between a 100-epoch arm and a 200-epoch arm would
reintroduce exactly the confound this revision exists to remove. Score D's ep100 checkpoint
as well, at a low confidence floor and at the operating point, to re-test the plateau claim
on the new corpus for free.

Effective weight decay is 0.00075, since `trainer.py` scales the declared 0.0005 by
`batch/nbs`. Same as A/B/C.

## Required tooling change - per-candidate `--labels`

`score.py` takes one global `--labels` list and passes `num_classes=len(class_labels)` to
`TrtYoloModel` (`score.py:797`), which is what splits the raw tensor into class scores and
keypoint values. A 2-class engine scored with three labels misparses to `num_keypoints=0`
and returns ~0 recall while looking like a broken engine. This bit `deploy_keypoints_2026-07-16.md`
once already.

D, E and F are 2-class. A, B and C are 3-class. So **the grid cannot be scored in one
invocation as `score.py` stands**, and scoring the two rows separately gives no paired
bootstrap across corpora, which is the CI the decision rule below asks for.

Add a repeatable per-candidate override, mirroring the existing `--stretch` flag
(`score.py:858`), which already exists so that "a mixed run scores each arm the way it was
trained":

```
--candidate-labels D=mr_stabs_mk2,mrs_buff_mk3
```

Candidates without an override keep the global `--labels`. Build the detector for each
candidate with its own list rather than the shared one. This is a small change in
`build_detector` and it removes a footgun that has now cost two experiments.

Sanity check after the change: every engine must print its own
`num_keypoints=2 num_classes=N` line, `N` of 2 for D/E/F and 3 for A/B/C.

## Running the arms

```bash
Q="venv/bin/python training/gpu_queue.py"
D="training/data/our_robot_keypoints"      # train.py resolves the directory to data.yml

$Q submit --name D_n_our --by <agent> -- \
  venv/bin/python training/yolo/train.py $D yolo26n-pose -d 0 1 2 -b 96 -e 200 --save-period 50
$Q submit --name E_s_our --by <agent> -- \
  venv/bin/python training/yolo/train.py $D yolo26s-pose -d 0 1 2 -b 96 -e 200 --save-period 50

$Q status
$Q logs <id> --tail 40
```

Queue D alone and score it before submitting anything else. It answers question 1 by itself,
it is the cheapest arm at ~6 h, and it is where a restore problem or a disk-space problem
will show up. E follows, F only if the corpus factor survives D and E.

Several agents share the three GPUs and each arm takes all of them, so check `$Q status`
before queueing: it prints the run order and an estimated finish for each job.

## Decision rule - register before looking

The previous report recorded that criterion (a) was under-specified: it named a metric and a
CI but not the confidence, and the verdict for `x` changed with the choice. Fixing that here.

**Primary, question 1.** Train the deployed pose model on `our_robot_keypoints` instead of
`all_robot_keypoints` only if, comparing D against A:

- (a) `kp_heading_err_deg` **at conf 0.5**, the deployed operating point, improves with a
  paired-bootstrap 95% CI excluding 0; **and**
- (b) `kp_pck@0.1` at conf 0.5 does not get worse by a CI excluding 0; **and**
- (c) the matched-box count at conf 0.5 does not fall. A heading gain bought by discarding
  hard detections is the artifact the previous report caught in the epoch ladder, where
  ep100 -> ep200 halved apparent heading error at conf 0.5 and changed nothing at conf 0.05.

Report conf 0.05 / 0.3 / 0.5 / 0.6 for every arm regardless, and say plainly when the ranking
flips between them.

**Secondary, question 2.** The size effect reproduces on `our_robot_keypoints` if E - D has
the same sign and rough magnitude as B - A, and F - D as C - A. State this as a comparison of
deltas, not of absolute numbers; the two corpora need not produce comparable absolute scores
for the interaction to be readable.

**Latency does not re-open.** Criterion (b) of the previous plan stands as measured: the
keypoint branch runs at 7.33 ms against the blob model's 7.05 in a 12.86 ms parallel batch
inside a 33.17 ms tick, leaving about 1 ms. `s` costs 1.29x and `x` 2.52x `n` on the dev box.
No corpus changes an engine's inference time, so **D is the only new arm that could deploy**,
and E and F are measurements rather than candidates. Benchmark the new engines anyway, as a
check that a 2-class head does not change the cost.

## Scoring

One invocation over the whole grid, once `--candidate-labels` exists:

```bash
venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate A=data/models/yolo26n-pose_all_robot_keypoints_2026-09-05_last_x86_64_sm86.engine \
  --candidate B=data/models/yolo26s-pose_all_robot_keypoints_2026-09-05_last_x86_64_sm86.engine \
  --candidate C=data/models/yolo26x-pose_all_robot_keypoints_2026-09-05_last_x86_64_sm86.engine \
  --candidate D=data/models/yolo26n-pose_our_robot_keypoints_<date>_last_x86_64_sm86.engine \
  --candidate E=data/models/yolo26s-pose_our_robot_keypoints_<date>_last_x86_64_sm86.engine \
  --candidate deployed=data/models/yolo26n-pose_our_robots_2026-05-01_x86_64_sm86.engine \
  --labels "mr_stabs_mk2,mrs_buff_mk3,opponent" \
  --candidate-labels D=mr_stabs_mk2,mrs_buff_mk3 \
  --candidate-labels E=mr_stabs_mk2,mrs_buff_mk3 \
  --candidate-labels deployed=mr_stabs_mk2,mrs_buff_mk3 \
  --taxonomy training/model_eval/taxonomy_keypoint.yaml \
  --conf 0.5 --baseline A --bootstrap 1000 \
  --output training/data/nhrl_keypoints_eval_test/scores_pose_corpus/conf0.5
```

- The global `--labels` is the 3-class list for A/B/C. Class 2 `nhrl_robot` maps to the eval
  GT's `opponent`; the eval vocabulary has no `nhrl_robot`, so the literal training name
  would score every third-class detection as a false positive. Keypoint matching in
  `score.py` is class-blind, so this affects box metrics only.
- `--baseline A` makes every delta read "against the corpus the previous sweep used". Re-run
  with `--baseline D` for the size comparisons within the new corpus.
- `taxonomy_keypoint.yaml` excludes `house_bot` and `object`, so metrics cover our robots only.
- Repeat at `--conf 0.05 / 0.3 / 0.5 / 0.6`.
- Score D's ep100 checkpoint in a separate run against D's ep200, at conf 0.05 and 0.5.

## Latency

```bash
venv/bin/python training/model_eval/benchmark_engines.py \
  --candidate A=... --candidate D=... --candidate E=... --candidate deployed=... \
  --frame <an eval frame> --iterations 300
```

Run it on an idle box. The previous sweep discarded a latency table taken while another
agent's job held the GPUs, which reported `n` slower than `s`. Dev-box numbers give ordering
only; `yolo26n` runs ~1.3 ms here and ~9.5-11 ms inside the Jetson pipeline.

Only if D wins: build `aarch64_sm87` engines on the Orin, `sudo jetson_clocks`, `trtexec`
plus `benchmark_engines.py`, swap into `config/_jetson.toml` `[keypoint_model.engine]
candidates` and read `mcap_latency_report.py`, which trims to after field init by default.

## Cost

Scaling the previous sweep's measured times by the corpus ratio, 31,912 / 18,447 = 1.73:

| arm | est. time |
|---|---:|
| D | ~6.0 h |
| E | ~7.3 h |
| F | ~19.3 h, conditional |
| G | ~3.5 h, optional |

D and E together are ~13 h. The full grid with F is ~33 h; all three are queued (see the arms
table for why F is no longer gated). Engine builds add ~10 min per arm. Reusing A, B and C
saves the 18.9 h they cost.

## Risks / caveats

- **The corpus contrast varies two things**, class vocabulary and 1.73x the training frames.
  Arm G is the control that separates them and is worth running before writing a conclusion
  that names the vocabulary as the cause.
- **Single seed.** `data_epoch_min` measured ~0.048 run-to-run recall spread; the equivalent
  for heading is still unmeasured. A D-against-A heading gap under a couple of degrees is
  `ns` in practice whatever the CI says on one seed each.
- **Neither val split is scene-disjoint.** Both are random frame-level carves of
  synthetic-dominated corpora, now checked and recorded for both. Every val number in this
  experiment measures fit to a renderer.
- **The eval set holds few of our robots.** With `house_bot` and `object` excluded, the
  keypoint metrics rest on the `mr_stabs_mk2` and `mrs_buff_mk3` boxes only, a few hundred to
  about a thousand depending on threshold. Report the matched-box count beside every metric.
- **The deployed model still is not a controlled comparison,** even with arm D. It ran 500
  epochs and fine-tuned from `yolo26n-pose_our_robots_2026-04-24.pt`, which is on neither the
  repo nor the archive drive, so its full lineage cannot be reproduced. Arm D is the control
  for its corpus, not for its schedule or its parent. Say so rather than reading D - deployed
  as a schedule effect.
- **Its training run directory is also gone.** `runs/projects/` only goes back to July 2026,
  so there are no curves or `results.csv` for the deployed model. Everything known about how
  it was trained comes from `train_args` inside the `.pt`.
- **98 GB of disk cache** for the new corpus, against 172 GB free. Reclaim the finished
  arms' caches first and check `df` before queueing E.
- **A null result is a real result.** If D lands on top of A, the deployed model's edge in
  the previous report was its schedule or its lineage rather than its corpus, and the answer
  is that the two corpora are interchangeable at `n`. That would make real cage footage the
  only remaining lever, which is already the standing recommendation.

## Deliverable

`docs/experiments/perception_performance/pose_model_size_corpus_<date>.md`, structured like
`model_size_2026-09-04.md`, reporting the 2x3 grid with A/B/C carried over from
`pose_model_size_2026-09-05.md`. Add `README.md` to `training/data/our_robot_keypoints`
recording its composition and val carve, and update the answer in `my_takeaways.md` that
`pose_model_size_2026-09-05.md` wrote.
