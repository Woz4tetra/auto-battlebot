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
