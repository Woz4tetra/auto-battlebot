---
name: eval
description: Score model or control performance against a baseline. Two modes - detector/keypoint accuracy (model_eval) and navigation/control (sim sweep). Use when asked to "evaluate", "score", "benchmark a model", "compare against baseline", "run the sweep", or measure detector/nav performance.
---

# eval

Two independent evaluation flows. Pick by what the user is measuring. If unclear, ask:
"detector accuracy or nav/control?"

---

## Mode A: detector / keypoint accuracy (`training/model_eval/`)

Scores a candidate YOLO-seg (blob) or YOLO-pose (keypoint) model against corrected
ground-truth labels built once from an SVO. Full docs: `training/model_eval/README.md`.

Activate the project venv first: `source scripts/activate_python.sh`.

1. **Record the labeling run** (keeps `/camera/image`, `/blob_detections`,
   `/keypoint_detections`). Set the SVO in the config, then:
   ```bash
   ./scripts/build_and_run.sh -c config/experiments/label_playback.toml
   ```

2. **Export YOLO pre-labels** (one dir per recording, shared images + a subdataset per model):
   ```bash
   python training/model_eval/export_labels.py data/recordings/*.mcap
   # restrict with --topics blob   or   --topics keypoint
   ```

3. **Correct the pre-labels** in the in-repo editor (point at the `blob` or `keypoint`
   subdataset):
   ```bash
   python training/model_eval/edit_labels.py training/data/model_eval/<recording>/keypoint
   ```

4. **Record each candidate** on the same SVO (no images, small files). Set
   `[robot_mask_model] model_path` + `label_indices` in the config, then:
   ```bash
   ./scripts/build_and_run.sh -c config/experiments/eval_candidate.toml
   ```

5. **Score** (GT dir is the subdataset; `--topic` must match it):
   ```bash
   python training/model_eval/score.py training/data/model_eval/<recording>/blob \
       --candidate generic=data/recordings/<run_a>.mcap \
       --candidate per_robot=data/recordings/<run_b>.mcap \
       --taxonomy training/model_eval/taxonomy.yaml
   ```
   Outputs `summary.csv`, `headline.png`, `confusion_*.png`. Levels: `agnostic` (localize),
   `archetype` (via taxonomy), `instance` (labels as-is).

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
