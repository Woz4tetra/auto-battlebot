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

## Input preprocessing

The network takes RGB, scaled to [0, 1] and normalized with the training corpus statistics: mean
(0.4611, 0.4359, 0.3905), std (0.2193, 0.2150, 0.2109). Camera frames arrive as 8-bit BGR, so
both paths convert channel order first.

| Step | Training (`semantic_train.py`, `score_masks.py`) | C++ `DeepLabMaskModel` |
| --- | --- | --- |
| Channel order | `cv2.COLOR_BGR2RGB` | `deeplab_input_nchw()`: `COLOR_BGR2RGB`, `BGRA2RGB` or `GRAY2RGB` |
| Geometry | Nearest resize to `image_size`, then zero pad `pad_size` | Replicate pad `pad_size` at frame resolution, then bilinear resize to the engine input |
| Normalization | `common_transforms` in `load_deeplabv3.py` | `kDeepLabTrainingMeanRgb`, `kDeepLabTrainingStdRgb` |
| Output | Argmax, crop the pad | Argmax, nearest resize to the padded frame, crop the pad |

The geometry differs on purpose: on the 2026-11-19 eval it scored the same IoU as the training
geometry with sharper boundaries at 2 px
(`docs/experiments/perception/2026-11-19_deeplab_preprocessing_fix.md`).
`auto_battlebot/segmentation/preprocess.py` reproduces either path in Python, and
`score_masks.py --preprocess NAME=channels,normalization,geometry` scores a checkpoint through
either one. `DeepLabInputTest` pins the C++ channel order and constants.

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

## `training/deeplab/field_overrides.toml`

`training/deeplab/field_labels.py` applies this file on top of the filename parser. Each `[scenes]` entry maps a scene key to a field label.

The parser reads the arena off the filename, which is all the provenance the corpus has. The Phase-0 spot-check (contact sheets of about 30 frames per field, spread across that field's scenes) found three places where the parser groups physically different arenas together or splits one arena across several labels. The overrides fix those, because they are observations about specific recordings, not rules about names. Every entry was checked by eye against the recording's own frames. Anything not listed keeps the parser's label.

| Label | Meaning |
| --- | --- |
| `own_arena` | Our own ZED recordings at an event: a real cage, camera outside the wall. After `zed2023`, the most deployment-relevant frames in the corpus. |
| `own_room_floor` | An office or workshop room floor with desks, chairs, backpacks, cables and AprilTags, and no cage in frame. The "field" here means the floor of the room, a different task from arena-floor segmentation. Left inside `own_recent`, these 1,284 frames would drag that field's IoU down and could make it the worst field, gating deployment on footage the robot never sees. |
| `own_practice_table` | One home-built practice table, black-rimmed, in a stone-walled workshop. The parser scattered it across three labels because the four recordings are named three ways: one own-recording timestamp and three bare epoch stamps. |

Deliberately not corrected:

- Three NHRL broadcast scenes still under `unclassified` (`12lb-3-smeezus...`, `30lb-1-waddles...`, `3lb-48-silentspring...`). They are plainly NHRL cages, but nothing identifies which cage, and guessing one would fold them into a cage's score under false pretenses. They stay unclassified and, by `score_masks.py`'s default `--exclude-field`, out of the macro-average.
- `mini_bot` spans at least three distinct rigs: a home floor in 2024-02, an event cage in 2024-03 and 2024-10-26, and the workshop table in 2024-10-06 and 2024-10-20. Splitting it would change what the corpus's second-largest non-cage bucket means, and it is not a probe field, so it is left intact and flagged.
- `cage6` has 13 scenes spanning jun24 to dec25 across four camera mounts, all visibly the same floor. No repaint, so no split.

## `training/deeplab/remap_robots.toml`

The label remap for a multi-class segmask dataset rendered by `render_scenes.py`, collapsed into binary floor segmentation. Any source label not listed in `[label_map]` maps to `default_label`.

| Label | Meaning |
| --- | --- |
| Source `0` | Background (ground plane and floor). Maps to target `0`. |
| Source `1`, `2` | Robot 1 and robot 2. Map to target `1`. `render_scenes.py` labels `2` a distractor, but the map treats it as a robot. |
| Source `24`, `25`, `26` | House bot. Map to target `0`, so the house bot counts as not floor. |
| Source `3` to `23`, `27` to `99` | Other robots. Map to target `1`. |
| Target `0` | Background, not floor. |
| Target `1` | Field, the floor surface. |

## `training/deeplab/remap_segmask_floor.toml`

The label remap for a SegmentFlow per-robot segmask export, collapsed into binary floor segmentation. Source labels are 1-based indices into the export's `classes.txt`.

| Label | Meaning |
| --- | --- |
| Source `0` | Unlabelled. Maps to target `0`. |
| Source `46` | `Floor`. Falls through to `default_label = 1`. |
| Source `47` | `Floor letter word`. Falls through to `default_label = 1`. |
| Every other source label | An individual robot (competitor, house bot or exhibition bot), 151 classes in all. |
| Target `0` | Background, not field. |
| Target `1` | Floor, the field surface. |

Robots sit on the field, so their pixels are field pixels that happen to be occluded. Mapping them to floor gives the unbroken field region the model should predict, and matches what `convex_hull_masks.py` assumes downstream. `default_label = 1` collapses every robot class to floor without listing all 151.
