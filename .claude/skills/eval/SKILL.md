---
name: eval
description: Score model or control performance against a baseline. Two modes - detector/keypoint accuracy (model_eval) and navigation/control (sim sweep). Use when asked to "evaluate", "score", "benchmark a model", "compare against baseline", "run the sweep", or measure detector/nav performance.
---

# eval

Two independent evaluation flows. Pick by what the user is measuring. If unclear, ask:
"detector accuracy or nav/control?"

---

## Mode A: detector / keypoint accuracy (`training/model_eval/`)

`score.py` runs candidate TensorRT engines directly on ground-truth images (same
preprocessing and NMS as the C++ pipeline). No playback run, no hardware, seconds per
model. Full docs: `training/model_eval/README.md`.

Activate the project venv first: `source scripts/activate_python.sh`.

```bash
# blob model (YOLO-seg); --labels is the C++ label_indices map, lowercased
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
    --candidate deployed=data/models/yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm89.engine \
    --labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3 \
    --conf 0.6 --taxonomy training/model_eval/taxonomy.yaml

# keypoint model (YOLO-pose): our-robot GT only, keypoint metrics added automatically
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
    --candidate deployed=data/models/yolo26n-pose_our_robots_2026-05-01_x86_64_sm89.engine \
    --labels mr_stabs_mk2,mrs_buff_mk3 \
    --conf 0.5 --taxonomy training/model_eval/taxonomy_keypoint.yaml
```

- GT arg is a dataset dir or a root of subdatasets. If `.edit_state.json` exists there,
  only reviewed frames are scored (this handles the in-progress test dataset correctly).
- Repeat `--candidate name=engine` to compare models; two or more triggers the paired
  bootstrap significance table against the baseline (first candidate, or `--baseline`).
- Match the C++ config when grading deployed models: blob `--conf 0.6`, keypoint
  `--conf 0.5`, `--nms-iou 0.45` (default).
- Outputs `summary.csv`, `headline.png`, `confusion_*.png`, `significance.csv` to
  `<gt>/scores` (override with `--output`). Levels: `agnostic` (localize), `archetype`
  (via taxonomy), `instance` (labels as-is).

**Test dataset** (`training/data/nhrl_keypoints_eval_test/`): built with
`make_eval_dataset.py` from the May fight recordings, labeled from scratch in
`edit_labels.py` (one subdataset per recording). To build GT for new recordings, see the
README workflow (make_eval_dataset.py or export_labels.py pre-labels + edit_labels.py).

Baseline numbers: `docs/experiments/perception_performance/baseline_2026-07-07.md`.
Playback-recorded MCAP scoring was removed 2026-07-07; direct inference scores slightly
higher than the old playback numbers (the C++ stack context is not measured).

Related: perception reliability over a fight uses
`training/model_eval/perception_reliability.py`.

---

## Mode B: navigation / control (sim sweep)

Batch-drives the headless kinematic sim: builds a per-run config overlay, launches sim +
`build/auto_battlebot`, scores the produced MCAP, tabulates `results.csv`.

```bash
source scripts/activate_python.sh
python playground/control_stage0/sim_sweep.py                        # built-in latency sweep
python playground/control_stage0/sim_sweep.py --sweep my_sweep.toml --out sweep_out
```

- Output: `<out>/results.csv` (default `playground/control_stage0/sweep_out/`).
- Metrics are sim-time (tick index x `[sim].dt`), not wall-clock, since runs are accelerated.

**Gotcha:** only nav implementations in `diag_io.NAV_HW_IDS` (currently PursuitNavigation
and MotionProfileNavigation) have their diagnostics scored. A brand-new nav impl scores
**empty** in `results.csv` until its hardware id is added to `NAV_HW_IDS` in
`playground/control_stage0/diag_io.py`. If a new nav's rows are blank, check that first.

---

## Notes

- Do not modify anything under `data/` (recordings, SVO, engines).
- The branch this work happens on is typically `benw/performance-evaluation`.
