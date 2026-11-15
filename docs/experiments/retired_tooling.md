# Retired tooling, 2026-09-08

Reports under `docs/experiments/` cite scripts by the path they had when the experiment
ran. Some of those scripts have since been deleted. This is the index, so a dead path in
an old report leads somewhere instead of nowhere.

Everything below was removed in `ab2c8186`. The last commit that still has all of it is
its parent, `1887db41`. Restore any single file with:

```bash
git checkout 1887db41 -- <path>
```

`playground/control_stage0/` has its own record in
[control_improvement/control_stage0_retired.md](control_improvement/control_stage0_retired.md).

## Figure generators

Seven mosaic scripts under `training/model_eval/`. Each produced one figure for one
experiment, and those figures are committed alongside the write-ups that cite them, so
the images are not lost. Deleting them also removed five of `score.py`'s eight
sibling importers, which is part of what made the library extraction clean.

| Script | Figure it produced | Cited by |
| --- | --- | --- |
| `make_centroid_mosaic.py` | Where each candidate position estimate lands on a robot crop | `perception_performance/mask_centroid_vs_box_2026-08-03.md` |
| `make_cutpaste_mosaic.py` | The cut-paste context-swap figure | `perception_performance/synthetic_plus_bbox_2026-07-22.md` |
| `make_pose_arms_mosaic.py` | Each pose arm's heading on the same robots | `perception_performance/pose_model_size_2026-09-05.md` |
| `make_pose_corpus_mosaic.py` | Domain gap across the pose corpus, one band per source | `perception_performance/pose_model_size_2026-09-05.md` |
| `make_geometry_arms_mosaic.py` | What each input-geometry arm detects | nothing |
| `make_meshy_fidelity_mosaic.py` | NHRL thumbnail vs Meshy render vs real fight robot | nothing |
| `make_dataset_mosaic.py` | Annotated GT frames from a `score.py` eval dataset | nothing |

## Superseded dataset scripts

| Script | Why it went |
| --- | --- |
| `training/deeplab/split_by_scene.py` | Superseded by `build_field_dataset.py`. `deeplab_field_data_plan.md` already records it as "not used on this corpus". |
| `training/deeplab/split_segmask_dataset.py` | Frame-level shuffle leaked frames between train and val. |
| `training/deeplab/merge_segmask_datasets.py` | Same, folded into `build_field_dataset.py`. |
| `training/deeplab/convert_yolo_seg_dataset.py` | The YOLO-seg to DeepLab converter. The corpus is built through `build_field_manifest` to `make_field_splits` to `build_field_dataset` instead. |
| `training/deeplab/remap_masks.py`, `test_on_video.py` | Unreferenced. |
| `training/yolo/convert_to_torchscript.py` | The TorchScript export path lost to ONNX and TensorRT. |
| `training/yolo/downscale_images.py`, `test_on_unlabeled_video.py` | Unreferenced. |

## Deleted experiment directories

| Directory | Why |
| --- | --- |
| `opponent_embedding/` | Self-tombstoned 2026-09-05; the probe code was already gone. |
| `robot_filter_decay/` | Finished stub, Tiers 1 and 2 never ran. Its verdict, `hold_window` 300 to 500 ms, is live in `config/_common.toml`. |
| `navigation_controller/` | Superseded by `control_improvement/`; its `config/experiment1*.toml` no longer exist. |
| `baseline_latency/` | Superseded by `perception_performance/assets/2026-09-06_int8_quantization/`. The generator, `scripts/mcap_latency_report.py`, is still here. |
| `parallel_yolo_batch/` raw dirs | 9.2 MB of measurement dumps. Both write-ups are kept; `ParallelModelBatch` is live in `src/perception_batch/`. |

## C++

`YoloSegRobotBlobModel` and `YoloSegMaskModel` were registered in their factories but no
config selected either. `YoloBboxRobotBlobModel` handles robot blobs, per
[perception_performance/seg_vs_bbox_2026-07-18.md](perception_performance/seg_vs_bbox_2026-07-18.md),
and `DeepLabMaskModel` handles the field.

## Vendored Stereolabs sample license

`training/svo_export.py` derives from a Stereolabs sample. Its file header keeps the copyright line; the license text that followed it moved here unchanged.

```text
Copyright (c) 2022, STEREOLABS.

All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
