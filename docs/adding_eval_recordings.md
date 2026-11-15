# Adding a recording to the NHRL eval dataset

End-to-end steps to turn a fight recording into a labeled sub dataset of
`training/data/nhrl_keypoints_eval_test`, complete with camera intrinsics and
camera-to-field transforms.

Every command runs from the repo root with the venv active:

```bash
source scripts/activate_python.sh
```

## 1. Get a recording that carries frames

An RGB-camera recording already does: it carries H.264 on `/camera/video` in the same file as
the pipeline output, and `make_eval_dataset.py` reads it directly. Skip to step 2.

A ZED-era recording does not. The frames live in a separate `.svo2` and the MCAP carries neither
JPEG nor video. `scripts/combine_mcap_svo.py` used to merge the two; it was deleted when video
moved into the recording, and it is still reachable in git history for the SVO corpus:

```bash
git log --diff-filter=D -1 --format=%H -- scripts/combine_mcap_svo.py   # the commit that removed it
git show <that commit>^:scripts/combine_mcap_svo.py > /tmp/combine_mcap_svo.py
python /tmp/combine_mcap_svo.py data/recordings/<raw>.mcap
```

That writes `data/recordings/<raw>__<svo_stem>.mcap` and requires `ZED_SVO_Editor` on PATH.
`ZED_SVO_Editor` prompts for its output path and defaults to a file next to its input, so if you
run it by hand, pipe the path you want on stdin or it writes into `data/svo/`.

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
recording made with the publisher disabled has none of these and cannot get transforms.

Recordings made after 2026-08-04 also carry `/camera/frame_meta`, giving the SVO frame index
per processed frame directly. Older recordings, including all of the 2026-05 NHRL set, predate
it, and their frames are matched by the timestamp interval rule instead.

## Eval dataset scripts

### `playground/depth_cache/cache_gt_depth.py`

`cache_gt_depth.py`:

Extract ZED depth for every ground-truth frame in an eval dataset, into a cache.

Depth is not recorded in the MCAP: the ZED SDK computes it on device and the pipeline never publishes it. The stereo pair is in the SVO though, so replaying the SVO through pyzed with the same DEPTH_MODE the runtime uses reproduces what the camera would have measured.

The join is by SVO frame index, never by timestamp. Each GT frame's `camera_transforms/<stamp_ns>.json` carries `image_stream_index`, which is the absolute index into the SVO. SVO image stamps sit about half a frame before the pipeline stamps that name the GT images, so a nearest-timestamp match silently picks the wrong frame.

Frames whose transform method is `ambiguous` or `unavailable` are skipped: the first has two processed frames claiming one index and the second has no pose at all, so neither can be placed in the field frame later.

Output is one `<subdataset>.npz` per sub dataset, keyed by stamp_ns, holding float16 depth in metres at the SVO's native resolution. Non-finite depth (no stereo match) is preserved as NaN, because which pixels the camera failed to measure is the result this experiment turns on.

```
Usage:
    python playground/cache_gt_depth.py training/data/nhrl_keypoints_eval_test         --svo-roots data/svo -o <cache dir>
```
