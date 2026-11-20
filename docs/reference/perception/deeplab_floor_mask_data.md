# DeepLab floor-mask data and sidecar

## Filename parsing

`auto_battlebot/segmentation/field_labels.py` parses a floor-mask filename into (field, scene, source). The corpus is a flat pile of `.jpg` + `_mask.png` pairs whose only provenance is the filename.

- field: the physical arena the frame was shot in, the unit of diversity for the DeepLab data question. Cage-1..7 are NHRL's cages. MobileCam and SkyCam are NHRL broadcast rigs whose cage is not named in the file. `zed2023` and `own_recent` are our own recordings. `mini_bot` is the test rig.
- scene: the recording. Splits are drawn scene-disjoint, so this is what must not cross the train/eval line.
- source: the labelled frame identity. Offline `_augment-N` copies share it with their original, which is how the leak in the shipped split is found (`test_augment_copies_share_source_and_scene`).

Field is inferred, never verified. See the "Risks" section of `docs/experiments/perception/deeplab_field_data_plan.md`.

- The camera token appears either as a bare filename prefix (`Cage-6-Red-2024-...`) or inside a BrettZone export (`..._tournamentID-nhrl_oct24_3lb_Cage-6-Overhead-High_...`, `BZ-nhrl_feb26_3lb-cole-anubis-W-53-Cage-2-Overhead-High_...`). Matching anywhere in the stem covers both without enumerating the export formats (`test_cage_token_in_bare_and_brettzone_names`).
- A cage token wins over a camera-rig token: a BrettZone export names both the tournament and the cage, and the cage is the arena (`test_cage_token_beats_camera_rig`).
- Frame-index tails come in two conventions, `_yolo_seg__frame_<N>` and Roboflow exports (`<scene>-<frame>_jpg.rf.<hash>` or `<scene>_repaired_<frame>_jpg.rf.<hash>`). Stripping them puts every frame of a recording on one scene key (`test_frame_tails_collapse_to_one_scene`, `test_roboflow_tail_stripped`).

## Checkpoint sidecar

Every trained model writes a sibling `<stem>.toml` next to the `.pth` (or `.engine`) that records what loading and preprocessing need. `auto_battlebot/segmentation/model_config.py` reads and writes it. Training, export, inference and the C++ `DeepLabMaskModel` all read the same file.

```toml
[model]
backbone = "mbv3"
image_size = 256
pad_size = 10
num_classes = 2
decoder = "v3"
```

`test_sidecar_round_trips_and_matches_cpp_keys` checks that the keys the C++ loader reads are the keys Python writes.

## Hull mask blob filter

`training/deeplab/convex_hull_masks.py` regenerates a segmask dataset with every mask blob replaced by its convex hull. Each `*_mask.png` is read as class IDs (0 is background). Each connected blob of a non-background class gets its own hull, so three blobs of one class come out as three hulls.

- Speck removal: a blob at or below `--min-blob-fraction` (default 0.10) of the largest blob of its class in that image is deleted, not hulled. Otherwise a stray handful of pixels survives as its own hull.
- `--min-blob-image-fraction` (default 0.01) keeps a blob that covers at least that fraction of the whole image even when it fails the relative test. An occluder can split the field unevenly, and the smaller side is a real half of the field. The threshold comes from `docs/experiments/perception/2026-11-15_deeplab_field_corpus.md`.
- Hulls are painted largest first, so a small hull inside a big one stays visible. The surviving original blobs are stamped back on top afterwards, so hull expansion only claims background pixels and never overwrites an annotation.
- The rest of the dataset is reproduced as is: source images are hardlinked (copied when the output is on another filesystem), metadata files are copied, symlinks are recreated.
- `--samples N` writes before/after comparison renders outside the output dataset (default `<output>_samples`), so they stay out of the dataset tree.

## Field splits

`training/deeplab/make_field_splits.py` holds out `PROBE_FIELDS` (`cage2`, `cage5`, `zed2023`) for the field-coverage probes. Cage-6 is not a probe field: it is a third of the corpus, so removing it would make the base arm differ in size as well as field coverage.

## Per-field scoring

`training/deeplab/score_masks.py` scores DeepLab checkpoints against held-out ground truth per field type. Cage-6 is a third of the corpus, so the global mean IoU is roughly a Cage-6 score. A checkpoint that maximizes it can regress the small cages the whole way. Results are macro-averaged across field types and reported as worst field, with the global mean kept for continuity.

- Field IoU is the primary number.
- Boundary F1 is computed at each `--boundary-tol`, in pixels at the model's native resolution. The mask feeds `PointCloudFieldFilter`, which fits the field plane and gives wall bounds to navigation. A 2% error in the interior costs nothing. A 2% error at the wall line moves the boundary. IoU cannot tell the two apart.
- Worst-field IoU gates deployment.
- With two or more candidates, a paired bootstrap tests each against the baseline. Resampling is stratified by field and shared across candidates, so every field stays represented in every resample (a macro-average over a resample that dropped a small field is a different estimator) and per-frame difficulty cancels.
- Checkpoints are graded as `.pth`. Engine conversion is only needed for the deployment candidate, and converting every arm would add about 20 runs of overhead to measure a quantization effect these comparisons do not ask about.

## Training

- `autocast` in `training/deeplab/semantic_train.py` uses bf16, which is about 1.67x faster on an A6000 at this resolution and needs no GradScaler. Check that it is loss-neutral before spending a full budget on it: run the same seed both ways for 20 epochs and compare.
- `paths_from_list` reads an arm's image list. Relative entries resolve against the corpus root. Arms are lists, not directories of copies, because the field ladder is 18 arms over about 29k images and physical copies would take about 350 GB against 282 GB free.

## Training seed

`training/deeplab/semantic_train.py --seed` does not make a run reproducible. `seed_everything` leaves `cudnn.benchmark` on, so kernel selection varies run to run and two runs at the same seed diverge. A seed pair therefore measures seed variation plus that nondeterminism, which together are the real run-to-run noise floor.

## score_masks.py `--exclude-field`

`unclassified` is a grab-bag of scenes whose arena could not be read off the filename. By default it is excluded so it does not get a vote in a macro-average over field types. Pass `''` to keep it.
