# 2026-11-15 deeplab field corpus

Notes moved out of code comments during the comment purge. Each entry names the file and symbol it came from.

## Notes moved from code comments (2026-11-15)

- `training/deeplab/convex_hull_masks.py`:

  ```text
  Regenerate a segmask dataset with every mask blob replaced by its convex hull.

  Each `*_mask.png` is read as class IDs (0 = background). For every non-background
  class, each connected blob is replaced by the convex hull of that blob, so a
  mask with three separate blobs of one class comes out as three separate hulls.

  Speck blobs are dropped before hulling: any blob whose pixel area is at or below
  `--min-blob-fraction` of the largest blob of its class in that image is deleted
  outright rather than hulled. Otherwise a stray handful of pixels would survive as
  its own hull.

  That relative test alone deletes real regions when an occluder splits the field
  into uneven pieces: a robot across the middle of the frame can leave one side at
  9% of the other, which is a genuine half of the field, not a speck. So a blob also
  survives if it covers at least `--min-blob-image-fraction` of the whole image
  regardless of how it compares to the largest blob. Measured on the field corpus,
  runner-up blobs that survive the relative test start around 0.36% of the image,
  while the median dropped one is 0.08%, so the absolute floor separates the two
  cleanly.

  Hulls are painted largest-area first, so a small hull that lands inside a big one
  stays visible. Pixels from the surviving original blobs are stamped back on top
  afterwards, so hull expansion can only ever claim background pixels, never
  overwrite an existing annotation.

  Everything else in the dataset is reproduced as-is: source images are hardlinked
  (copied if the output lands on another filesystem), metadata files are copied,
  symlinks are recreated as symlinks.

  Usage:
      python convex_hull_masks.py /path/to/dataset -o /path/to/output
      python convex_hull_masks.py /path/to/dataset -o /path/to/output --samples 500

  Samples are before/after comparison renders written outside the output dataset
  (default `<output>_samples`) so they do not pollute the dataset tree.
  ```

- `training/deeplab/make_field_splits.py` (`above PROBE_FIELDS = ("cage2", "cage5", "zed2023")`):
  Phase B probes. Cage-6 is unusable: removing a third of the corpus would make the base arm differ in size as well as field coverage.
- `training/deeplab/score_masks.py`:

  ```text
  Score DeepLab field-mask checkpoints against held-out ground truth, per field type.

  The global mean IoU this project has been reading is roughly a Cage-6 score, one field
  is a third of the corpus, so a checkpoint that maximises it can be regressing the small
  cages the whole way. Everything here is therefore reported **macro-averaged across field
  types** and as **worst field**, with the global mean kept only for continuity.

  Metrics, per field and macro-averaged:

      field IoU       the primary number.
      boundary F1     at each --boundary-tol, in pixels at the model's native resolution.
                      The mask feeds PointCloudFieldFilter, which fits the field plane and
                      hands wall bounds to navigation: 2% wrong in the interior costs
                      nothing, 2% wrong at the wall line moves the boundary. IoU cannot
                      tell those apart.
      worst-field IoU the metric that actually gates deployment.

  With two or more candidates a paired bootstrap reports whether each differs from the
  baseline. Resampling is stratified by field and shared across candidates, so every field
  stays represented in every resample (a macro-average over a resample that dropped a small
  field is not the same estimator) and per-frame difficulty cancels.

  Checkpoints are graded as `.pth` directly, engine conversion is only needed for the
  deployment candidate, and a per-arm conversion would add ~20 runs of overhead to measure
  a quantisation effect this experiment is not asking about.

  Usage:
      python3 training/deeplab/score_masks.py         training/data/deeplab_field_2026-07-28/eval         --candidate deployed=data/models/field_deeplabv3p_r50_2026-04-29.pth         --by-field --boundary-tol 2,5,10 --bootstrap 1000         --output training/data/deeplab_field_2026-07-28/scores/anchor
  ```

- `training/deeplab/semantic_train.py`, `autocast`:
  bf16 autocast is ~1.67x on an A6000 at this resolution and needs no GradScaler.
  Verify it is loss-neutral before spending a full budget on it, run the same seed both ways for 20 epochs and compare (Phase A's amp_check arms).
- `training/deeplab/semantic_train.py`, `paths_from_list`:
  Read an arm's image list. Relative entries resolve against the corpus root.
  Arms are lists rather than directories of copies because the Phase-B ladder is 18 arms over ~29 k images; physical copies would be ~350 GB against 282 GB free.
