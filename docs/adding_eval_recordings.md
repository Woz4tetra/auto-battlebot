# Adding a recording to the NHRL eval dataset

End-to-end steps to turn a fight recording into a labeled sub dataset of
`training/data/nhrl_keypoints_eval_test`, complete with camera intrinsics and
camera-to-field transforms.

Every command runs from the repo root with the venv active:

```bash
source scripts/activate_python.sh
```

## 1. Build the combined MCAP

Skip this if you already have a `<raw>__<svo_stem>.mcap`.

```bash
python scripts/combine_mcap_svo.py data/recordings/<raw>.mcap
```

The raw MCAP from a fight carries no images. This merges in the SVO's frames on
`/camera/image` and writes `data/recordings/<raw>__<svo_stem>.mcap`. Requires
`ZED_SVO_Editor` on PATH.

`ZED_SVO_Editor` prompts for its output path and defaults to a file next to its input, so if
you ever run it by hand, pipe the path you want on stdin or it writes into `data/svo/`.

## 2. Register the SVO in the playback config

Add the recording to `config/playback/_playback.toml` under `[rgbd_camera]`, commented out
alongside the others:

```toml
# svo_file_path = "data/svo/2026-05-02T10-06-06.svo2"
# svo_start_frame = 8000
```

`make_eval_dataset.py` parses this file for `svo_start_frame` and skips every frame before it,
reading commented-out entries too. Do this before step 3. Without an entry the start frame is 0
and you sample the pre-match setup footage instead of the fight.

Pick the start frame by scrubbing the SVO in playback until the match begins.

## 3. Extract frames

```bash
python training/model_eval/make_eval_dataset.py \
    data/recordings/auto_battlebot_main_<...>__<...>.mcap \
    --per-video 100
```

Produces `training/data/nhrl_keypoints_eval_test/<recording>/` containing `images/`, an empty
`labels/`, and a `data.yaml`. Frames are sampled evenly across the recording rather than
consecutively, so they decorrelate.

Images are named by `/camera/image` header stamp. Later steps match on that name, so do not
rename them.

Two defaults are already right for NHRL and rarely need changing:

- `--extra-classes opponent house_bot` adds the classes absent from the training set.
- `--playback-config config/playback/_playback.toml` is what step 2 edited.

## 4. Label

```bash
python training/model_eval/edit_labels.py \
    training/data/nhrl_keypoints_eval_test/<recording>
```

Only frames marked reviewed count as ground truth when scoring. Label at the finest
granularity you care about, one box per robot instance.

## 5. Export camera geometry

```bash
python training/model_eval/export_camera_transforms.py --dry-run
python training/model_eval/export_camera_transforms.py
```

Writes `camera_info.json` and `camera_transforms/<stamp_ns>.json` per sub dataset. Reads
`source_mcap` out of the `data.yaml` from step 3, so if you move or rename the MCAP after
extraction, this step fails.

Run `--dry-run` first and read the `exact` count. It is the number of images that map to a
frame the perception loop actually processed. Expect roughly 75 to 82 out of 100. A much lower
number means that recording's SVO dropped frames while recording, which is unfixable after the
fact, and you want to know that before spending time on step 4. The 2026-05-01 17-42 recording
scores 38 of 100 for exactly this reason.

The script rewrites `camera_transforms/` for every sub dataset in the directory, not just the
new one. That is idempotent and never touches `images/` or `labels/`, but it is not
incremental. Point `--dataset-dir` at a directory holding only the new recording to narrow it.

See the Camera geometry section of `training/model_eval/README.md` for what the `method` field
means and why `field_size_m` is not arena ground truth.

## 6. Verify

```bash
cd training/data/nhrl_keypoints_eval_test/<recording>
ls images | wc -l && ls labels | wc -l && ls camera_transforms | wc -l
```

All three counts must match, and `camera_info.json` must exist.

## 7. Push to megamind

The dataset lives at the same path on megamind as locally.

```bash
rsync -a --no-perms --omit-dir-times --stats \
    training/data/nhrl_keypoints_eval_test/ \
    megamind:/home/ben/auto-battlebot/training/data/nhrl_keypoints_eval_test/
```

Never add `--delete`. Megamind's copy of this directory also holds two dozen `scores_*`
directories that do not exist locally, and `--delete` destroys all of them.

Check that no training is running first. Heavy IO during a run evicts its page cache and spikes
epoch time:

```bash
ssh megamind 'nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader; uptime'
```

## 8. Score against it

```bash
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
    --candidate deployed=data/models/<engine> \
    --labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3 \
    --conf 0.6 --taxonomy training/model_eval/taxonomy.yaml
```

`--labels` must have one entry per engine class, in the engine's own index order. A wrong count
misparses the output tensor and recall collapses to near zero, which looks like a broken engine
rather than a bad argument.

## Requirements on the recording

Step 5 needs `/tf`, `/tf_static`, `/field_markers`, and `/camera/camera_info` in the MCAP. A
recording made with the ROS publisher disabled has none of these and cannot get transforms.

Recordings made after 2026-08-04 also carry `/camera/frame_meta`, giving the SVO frame index
per processed frame directly. Older recordings, including all of the 2026-05 NHRL set, predate
it, and their frames are matched by the timestamp interval rule instead.
