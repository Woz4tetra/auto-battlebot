# 2026-11-15 model eval notes

Notes moved out of code comments during the comment purge. Each entry names the file and symbol it came from.

## Notes moved from code comments (2026-11-15)

- `training/model_eval/conf_sweep.py`:

  ```text
  Find the confidence threshold whose detection count matches what a dataset should hold.

  A pre-label run at a fixed confidence over-detects on purpose, so the box count says as much about
  the threshold as about the model. This runs each candidate once at a low floor, keeps every
  detection's confidence, and then sweeps thresholds offline: one inference pass answers "what
  threshold would have produced the expected count", for every threshold at once.

  The expectation is a per-frame box rate, measured from a labelled set rather than assumed. For NHRL
  cage footage `nhrl_keypoints_eval_test` gives 2.81 boxes per frame over its 590 NHRL frames
  (mrs_buff_mk3 0.98, opponent 1.14, house_bot 0.68), which is below the nominal two robots plus a
  house bot because the house bot is only in frame 68 percent of the time.

  Matching the count is necessary, not sufficient: an arm can hit the right number of boxes with them
  in the wrong places, and a class the footage never contains (Mr Stabs, in Mrs Buff's matches) is a
  false positive at every threshold. The per-class table is printed for that reason.

  Detections are cached per candidate under --cache, so re-sweeping against a different expectation
  costs nothing.

  Usage:
      venv/bin/python training/model_eval/conf_sweep.py training/data/nhrl_cage_high_eval \
          --candidate base=data/models/yolo26s-pose_base_2026-09-13_last.pt \
          --map nhrl_robot=opponent --expected-per-frame 2.81 --device 0
  ```

- `training/model_eval/export_camera_transforms.py`, `export_sub_dataset` (`above size_index = bisect_right(field_size_stamps, stamp) - 1`):
  Extents the RANSAC plane fit measured at this field init, centred on the field origin. Needed to know where the arena edges are: the transform alone does not say. Estimates vary per init and some are plainly bad fits, so the value is reported as measured rather than snapped to a nominal arena size.
- `training/model_eval/interpret_context_vs_appearance.py`:

  ```text
  Context-vs-appearance interpretability for the synthetic+bbox opponent detectors.

  Tests the report's central claim: the opponent detector keys on *context* (arena / "not us,
  not house bot, on the floor") rather than the opponent's own *appearance*. Runs four probes on
  two models (real_bbox baseline, mix_all) over the independent eval, and prints metrics + saves
  figures:

    1. RISE occlusion-saliency  - where does the evidence for an opponent box live? Inside the box
       (appearance) or in the surrounding context? Metric: saliency concentration inside vs a
       context ring vs background, normalized by area.
    2. Cut-paste context swap   - opponent crop on a neutral gray canvas (appearance only) vs on a
       different frame's real arena (appearance + swapped context) vs original. If the score
       collapses without arena context, the model needs context, not the robot's pixels.
    3. Feature-space check      - are real-opponent and synthetic-opponent crops separable in
       backbone-embedding space (domain gap)? Linear-probe accuracy + centroid cosine distance,
       per model. Also whether mix_all pulls the two closer.
    4. Grad-CAM                 - gradient saliency at the P3 neck layer for the opponent score at
       each GT box; corroborates RISE with the inside-box mass fraction.

  All probes define the target consistently as the max opponent class score (channels object+robot)
  among head anchors whose center falls inside the target box.

  Usage:
    PYTHONPATH=. venv/bin/python training/model_eval/interpret_context_vs_appearance.py       --eval training/data/nhrl_keypoints_eval_test       --real data/models/yolo26n_nhrl_robots_bbox_2026-07-16.pt       --mix  data/models/yolo26n_mix_all_2026-07-22.pt       --synth training/data/synth_bbox_from_keypoints       --out training/data/nhrl_keypoints_eval_test/scores_synth_plus_bbox/interpretability
  ```

- `training/model_eval/make_field_projection_check.py`:

  ```text
  Draw the arena floor onto eval frames so the exported camera transforms can be eyeballed.

  Everything downstream of `export_camera_transforms.py` (floor distances, range bands) is only
  as good as `tf_field_from_camera`. This projects things whose correct appearance is known and
  lets you check them by eye:

  - The nominal 8 ft arena square and a 0.5 m grid on the floor plane. The square should sit on
    the cage walls. `field_size_m` from the RANSAC fit is deliberately not used, since it reads
    undersized and sometimes fails outright; the nominal is the honest thing to draw.
  - Range rings at 1, 2 and 3 m from the camera's nadir, which should look like concentric
    ellipses hugging the floor.
  - Each labeled robot's keypoint midpoint, round-tripped pixel -> floor -> pixel. The
    round-trip marker must land back on the original, and the printed residual says by how much.
    This catches a broken transform even when the grid looks plausible.

  Usage:
      python training/model_eval/make_field_projection_check.py         training/data/nhrl_keypoints_eval_test         -o docs/experiments/perception/assets/2026-08-03_mask_centroid/field_check.png
  ```

- `training/model_eval/make_robot_capture_mosaic.py`:

  ```text
  Mosaic of the sharpest real captures of each robot in an eval dataset.

  Supporting figure for the Meshy fidelity grades (`meshy_grade_*.md`). Those experiments
  ask whether a Meshy mesh built from a stale thumbnail still looks like the robot the
  camera sees at fight time, so the reader needs the clearest real views of each robot to
  compare against. One row per robot, the N best crops from its ground-truth boxes.

  Our robots are read from their own GT label. The eval GT labels every opponent as a
  generic `opponent`, so each opponent is named by the recording it fought in
  (`--opponent name=recording_prefix`), the same map the grade uses.

  "Best" is the variance of the Laplacian over the crop after it is resized to a fixed
  edge, which prefers sharp, large boxes and penalizes motion blur and small robots alike
  (a small crop is upsampled and reads as soft). Boxes touching the frame edge are skipped
  since they are partly out of view. Picks from one recording are kept at least
  `--min-gap` seconds apart, otherwise a robot that sits still for ten seconds fills its
  whole row with the same view.

  Usage:
      python training/model_eval/make_robot_capture_mosaic.py         training/data/nhrl_keypoints_eval_test         -o docs/experiments/perception/assets/<date>/robot_captures_mosaic.png
  ```

- `training/model_eval/mask_centroid_vs_box.py`, `floor_errors`:
  The same errors again, but measured on the arena floor in millimetres.
  Each estimate's pixel is projected onto the plane the robot sits on and the distance is taken between floor points, not scaled from a pixel distance. Perspective means a pixel is worth far more at the far wall than at the camera, so a single mm-per-pixel factor would misstate both ends.
- `training/model_eval/mask_centroid_vs_box.py`:

  ```text
  Measure how far a YOLO-seg mask centroid sits from its own bounding-box center.

  The segmentation head emits a mask and a box for the same detection, but the pipeline
  only consumes a position. This script asks whether the mask centroid carries position
  information the box center does not:

  1. Per detection, the offset between the mask centroid and that detection's box center,
     in pixels and as a fraction of the box's longer side (the same normalization
     score.py uses for keypoint PCK).
  2. Over IoU-matched detections, how far each position estimate lands from the ground-truth
     box center. A detect-only model can be passed as a third estimator so the mask
     centroid, the seg box center, and a bbox-only model are graded on one scale.

  Inference runs through ultralytics on the .pt weights, not the TensorRT engine, because
  TrtYoloModel discards the mask coefficients. Detection metrics from this path differ
  slightly from score.py's engine numbers; the offsets measured here are self-consistent
  because centroid and box come from the same forward pass.

  Example:

      python training/model_eval/mask_centroid_vs_box.py training/data/nhrl_keypoints_eval_test         --seg-weights data/models_v2/yolo26n-seg_nhrl_robots_2026-04-27.pt         --seg-labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3         --bbox-weights data/models/yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31.pt         --bbox-labels opponent,house_bot         --taxonomy training/model_eval/taxonomy_merged.yaml --conf 0.5
  ```

