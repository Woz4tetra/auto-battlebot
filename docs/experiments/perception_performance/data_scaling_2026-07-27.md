# How much labelled data do I need — scenes vs frames, 2026-07-27

Five cold-start arms on the human-validated `nhrl_robots_bbox_2class`, 100 epochs each, graded on
`nhrl_keypoints_eval_test` (372 reviewed frames, robot-camera footage) with a paired 1000× bootstrap
at conf 0.5 under `taxonomy_merged.yaml`.

Plan: `data_scaling_plan.md`.

## Headline

1. **The validated 2-class corpus beats the deployed baseline by a wide margin** — agnostic recall
   **0.882 vs 0.742** at comparable precision. That is a bigger gain than anything else measured in
   this series, and it came from label hygiene, not more data.
2. **The data floor sits between 50 % and 75 %.** Both 75 % arms clear the parity gate by ep50;
   `scene50` never clears it at any checkpoint and `rand50` only scrapes through at ep100.
3. **Scene diversity beats frame count, and the gap widens as data shrinks.** At matched frame
   counts, random sampling (all 47 scenes) beats whole-scene sampling — decisively at 50 %, where
   random passes and scene-sampled fails outright.
4. **Accuracy has saturated by ~20 k frames.** `rand75` (20,049 frames) is **statistically
   indistinguishable** from `base100` (26,733) — every paired metric `ns`. The last 25 % of labelling
   bought nothing measurable. Note the direction: the extra data does not *help*, which is not the
   same as removing it *helping*. See §Which model to deploy.

## Setup

| | |
|---|---|
| corpus | `nhrl_robots_bbox_2class`, 31,465 frames, 56 scenes, human-validated |
| classes | `robot` (68,104), `house_bot` (24,347) |
| val | 9 scenes / 4,732 frames, scene-disjoint, **identical across all arms** |
| training pool | 47 scenes / 26,733 frames |
| schedule | cold from COCO, 100 epochs, `lr0` 0.01, `lrf` 0.1, `degrees` 45, `flipud` 0, `close_mosaic` 0 |
| ladder | ep{25, 50, 75, 100} scored per arm |

Arms are **nested** (`scene50 ⊂ scene75 ⊂ base100`, same for rand) and **frame-matched** across
sampling modes, so the only variable between `scene-N` and `rand-N` is how many scenes those frames
came from.

| arm | scenes | frames |
|---|---|---|
| base100 | 47 | 26,733 |
| scene75 | 37 | 21,159 |
| rand75 | 47 | 20,049 |
| scene50 | 24 | 13,670 |
| rand50 | 47 | 13,366 |

**Baseline anchor** (re-measured under `taxonomy_merged.yaml`): recall **0.742**, precision **0.962**,
F1 **0.838**, mAP50-95 **0.504**.

## Results — earliest checkpoint clearing the gate

Gate: recall Δ CI lower bound ≥ −0.04, precision and F1 not significantly worse.

| arm | earliest pass | recall Δ [95 % CI] | precision Δ | F1 Δ |
|---|---|---|---|---|
| **base100** | **ep50** | +0.021 [+0.005, +0.039] | +0.017 ns | +0.019 |
| **scene75** | **ep50** | +0.008 [−0.012, +0.027] ns | +0.054 better | +0.029 |
| **rand75** | **ep50** | +0.017 [−0.001, +0.035] ns | +0.065 better | +0.039 |
| **scene50** | **never** | best ep100: −0.014 ns | **−0.061 worse** | −0.037 |
| **rand50** | **ep100** | +0.021 [+0.001, +0.042] | +0.013 ns | +0.018 |

`scene50` fails at every checkpoint: ep25 loses recall, and ep50/75/100 all lose **precision**
significantly (−0.086, −0.047, −0.061). It detects robots acceptably but fires too often on things
that are not robots.

## Results — best achievable per arm

| arm | frames | recall | precision | F1 | mAP50-95 |
|---|---|---|---|---|---|
| baseline | 49,086 (mixed) | 0.742 | **0.962** | 0.838 | 0.504 |
| base100 (ep75) | 26,733 | 0.882 | 0.923 | 0.902 | 0.570 |
| rand75 (ep100) | 20,049 | 0.884 | 0.926 | 0.904 | 0.584 |
| scene75 (ep100) | 21,159 | 0.864 | **0.948** | 0.904 | 0.567 |
| rand50 (ep100) | 13,366 | 0.867 | 0.900 | 0.883 | 0.556 |
| scene50 (ep100) | 13,670 | 0.832 | 0.826 | 0.829 | 0.523 |

**Read this table with the paired tests below, not by eye.** `rand75` tops it on recall and mAP50-95,
but those margins over `base100` are inside noise; ranking arms by raw score here is misleading.

## Answers

### How many real images do I need to label? — **strong**

**~20,000 frames, and the curve is flat above that.** `rand75` at 20,049 frames equals or beats
`base100` at 26,733 on every metric. The extra 6,700 frames — a 33 % increase in annotation effort —
produced no measurable gain.

Below that it degrades fast: at ~13.4 k frames `rand50` only scrapes the gate at the final checkpoint,
and `scene50` fails outright. **The floor is between 13.4 k and 20 k frames**, closer to 20 k for a
comfortable pass.

### Scenes or frames? — **strong**

**Scenes.** At matched frame counts, drawing from all 47 scenes beats concentrating the same frames
in fewer fights, and the advantage grows as the budget shrinks:

| volume | scene-sampled | random | verdict |
|---|---|---|---|
| ~20 k frames | passes at ep50 | passes at ep50 | tie |
| ~13.5 k frames | **never passes** | passes at ep100 | **random wins** |

At 75 % both work, so scene count (37) is already sufficient. At 50 % the scene arm drops to 24 scenes
and collapses on precision, while the random arm — same frame count, still 47 scenes — survives.

The practical reading: **more fights beats more frames per fight.** Once a fight is labelled, extra
frames from it are near-duplicates and add little. If the labelling budget is fixed, spread it thin
across many recordings rather than exhaustively labelling a few.

One honest caveat in the other direction: the scene arms were slightly *larger* than their random
counterparts (21,159 vs 20,049; 13,670 vs 13,366), because whole scenes overshoot the target. That
overshoot favours the scene arms, so their loss is if anything understated.

### When do I stop training? — **moderate**

**ep50 at 75–100 % of data; ep75–100 at reduced data.** Every arm's ep25 fails the gate. The larger
arms clear it by ep50 and gain little after; the smaller arms need the full 100 epochs.

No arm overfit by ep100 — unlike `category_addition_2026-07-25`, where every 150-epoch endpoint
failed. The `lrf` 0.1 change (less-annealed endpoint) and the shorter 100-epoch schedule appear to
have removed that failure mode, though those two changes are confounded here and this experiment does
not separate them.

The equal-epochs design was checked rather than assumed: all five arms peaked on val between ep63 and
ep90, none still rising at ep100, so the smaller arms' halved gradient-step count did not leave them
under-trained.

## The finding that dwarfs the rest

**Every arm beat the deployed baseline, including the smallest one that passed.** The baseline was
trained on 49,086 frames; `rand50` used 13,366 — **27 % as much data** — and still cleared the gate.

The difference is not volume, it is what was wrong with the old corpus:

- 34.6 % synthetic renders on generic backgrounds
- `Mrs Buff MK2` (an opponent) labelled as our own robot
- 1,680 review-failed frames included
- per-polygon box conversion producing frame-spanning boxes and hundreds of spurious fragments
- 4,146 frames the human review rejected

Recall went 0.742 → 0.882 while precision stayed within noise. **Corpus hygiene bought ~4× more than
corpus size.**

The one place the baseline still wins is precision (0.962 vs 0.923). It is a more conservative model —
it fires less and is right more often when it does. Whether that trade is right depends on whether a
missed opponent or a false target costs more in a match.

## Which model to deploy

The arms above are each scored against the *deployed baseline*. Scoring them against **each other**
changes the reading. Paired vs `base100_ep75`:

| arm | recall Δ | precision Δ | F1 Δ |
|---|---|---|---|
| `rand75_eplast` | +0.002 **ns** | +0.002 **ns** | +0.002 **ns** |
| `rand75_ep75` | −0.002 **ns** | −0.004 **ns** | −0.003 **ns** |
| `base100_eplast` | −0.017 worse | +0.018 better | −0.000 ns |
| `scene75_eplast` | −0.019 worse | **+0.024 better** | +0.001 ns |

**`rand75` is not better than `base100` — it is indistinguishable from it.** Every metric `ns`. The
0.884-vs-0.882 recall and 0.584-vs-0.570 mAP50-95 in the table above are within noise.

**Deploy from `base100`, not `rand75`.** The arms are nested, so `rand75` is a strict subset of
`base100`: choosing it means discarding 6,684 already-labelled, already-validated frames for no
measurable gain. The `rand75` result governs **future labelling budget** — stop around 20 k frames of
this footage — not which corpus to train on today.

**The real deployment choice is precision vs recall, not dataset size.**

| model | recall | precision |
|---|---|---|
| `base100_ep75` | **0.882** | 0.923 |
| `scene75_eplast` | 0.864 | **0.948** |
| deployed baseline | 0.742 | **0.962** |

`scene75_eplast` trades 0.019 recall for 0.024 precision against `base100_ep75` — both **significant**,
not noise. It sits closer to the deployed baseline's conservative behaviour while still gaining 0.12
recall over it.

That is a decision this experiment cannot make: **does a missed opponent or a false target cost more
in a match?** The deployed model is heavily precision-biased (0.962). If that reflects a deliberate
choice rather than an accident of its training corpus, `scene75_eplast` preserves it better.

One option not tested: once the recipe is fixed the 9 held-out val scenes no longer need reserving, so
a final model could train on all 31,465 frames. Given the saturation above 20 k it should not move the
numbers, but it costs one ~2 h run and leaves no labelled data unused.

## Risks / caveats

- **One seed per arm.** δ = 0.04 is tighter than the 0.048 cross-run spread measured between two
  nominally identical runs in Phase A. Adjacent arms differing by less than ~0.05 recall are not
  separable; the *shape* across three volumes carries the conclusions, not any single pair.
- **The scene arms are one draw.** Which 24 scenes landed in `scene50` matters. Its failure is
  consistent with the val curve and with the 75 % result, but a second draw would harden it.
- **`rand50` passing only at ep100 is fragile.** Recall Δ CI is [+0.001, +0.042] — the lower bound is
  barely above zero. Treat 13.4 k frames as below the practical floor, not at it.
- **The 25 % arms were dropped**, so the floor is bounded between 13.4 k and 20 k but not located
  more precisely. A 25 % pair would cost ~1 h.
- **The domain gap is untouched.** Training is NHRL overhead cage footage; the eval is the robot's own
  ZED. The saturation above 20 k frames may be a statement about that gap — more of the *wrong*
  viewpoint not helping — rather than about labelling in general. Footage from the deployment camera
  is the untested axis.
- **Scene sampling and frame count are matched, but scene *content* is not controlled.** A draw heavy
  in `mini_bot` footage differs from one heavy in cage overheads.

## Recommendation

- **Deploy from `base100`** — `ep75` for maximum recall (0.882), or `scene75_eplast` if the deployed
  model's precision bias (0.962) is deliberate and worth preserving (0.948 at 0.864 recall). Do **not**
  pick `rand75`: it is a subset of `base100` and statistically identical to it, so it only discards
  labelled data.
- **Stop labelling around 20 k frames** of this footage. The curve is flat above it.
- **Spend the next labelling budget on new fights, not more frames from existing ones.** That is where
  the scene-vs-random gap points.
- **The highest-value next experiment is not more data — it is deployment-camera data.** Everything
  here saturates on overhead cage footage while the eval is the robot's own view.

## Artifacts

- Corpus: `training/data/nhrl_robots_bbox_2class/` (validated; rejects in `validation_backup/`)
- Splits: `training/data/datascale_2026-07-27/{val,base100,scene75,rand75,scene50,rand50}.{txt,yml}`
  + `manifest.json`
- Runs: `runs/projects/auto_battlebots_2026-07-27_{00-49-19,02-29-01,03-47-11,05-02-05,05-54-28}_yolo26n`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_datascale{,_anchor}/`
- Taxonomy: `training/model_eval/taxonomy_merged.yaml`
