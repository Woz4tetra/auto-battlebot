# TensorRT engine wrapper (`TrtEngine`)

`include/tensorrt_inference/trt_engine.hpp` wraps one serialized `.engine` file. `EngineSelector` probes candidate files with it and the three TensorRT models (`YoloKeypointModel`, `YoloBboxRobotBlobModel`, `DeeplabMaskModel`) run inference through it.

## Loading

- `load()` streams the file into TensorRT instead of buffering it. A plan built for another GPU is rejected after the header, not after reading the whole file. That keeps `EngineSelector` probing cheap even for the 149 MB DeepLab engine.
- Every failure path leaves the object clean (`isLoaded()` false, no buffers held), so a caller may try another path. `TrtEngineTest.FailedLoadLeavesObjectClean` and `FailedLoadLeavesObjectReusable` cover it.

## Execution

- Each engine owns a non-blocking CUDA stream, so two engines running on different threads overlap on the GPU instead of serializing on the default stream.
- Host IO goes through pinned staging buffers, so the copies are async on that stream.
- `execute()` and `execute_multi()` take `std::span` buffers and return false if a buffer holds fewer floats than the engine's tensor.
- `execute_borrowing_pinned_output_after_device_input()` skips the host input copy: the caller writes `device_input()` on `stream()` first. It returns a span over the engine's pinned output buffer, valid until the next call on that engine. An empty span means failure.

## CUDA graph

- The graph is captured after the first successful run. It replaces TensorRT's per-layer kernel launches with one launch.
- The measured gain on Orin NX is in `docs/experiments/perception/cuda_graph_orin_nx_2026-11-15.md`.
- `uses_cuda_graph()` is true once inference replays from the graph. A failed capture logs a warning and the engine keeps running without one.

## CUDA handle types

`include/tensorrt_inference/cuda_handles.hpp` forward declares `cudaStream_t`, `cudaGraphExec_t` and `DeviceTensorView{data, height, width}`, so headers name the real CUDA types without including CUDA.

## Engine filename tags

`engine_path_with_platform_tag` in `auto_battlebot/tensorrt_build.py` appends `_<arch>_sm<XX>` to the engine stem, for example `model_x86_64_sm89.engine`. The `sm` tag stops an engine built for a different GPU architecture from loading silently. A precision tag goes ahead of it (`model_int8_x86_64_sm89.engine`) so the trailing `_<arch>_sm<XX>.engine` shape that the config candidate lists match on is unchanged, and FP16 filenames stay byte-identical to what `config/_jetson.toml` already names (`test_engine_name_tags`).

## YOLO and DeepLab build options

- `--int8` (YOLO): calibrates on `--calib-dir` frames. FP16 stays enabled so layers TensorRT will not quantize fall back to FP16 rather than FP32. The output filename gains an `_int8` tag ahead of the platform tag.
- `--calib-dir` (YOLO): stratified across scenes by `--calib-count`. Use the val split of the training corpus, never the eval set being scored.
- `--calib-partition I/N` (YOLO): draws every Nth frame of each scene starting at I. Two partitions of the same N share no frames, which builds a disjoint second calibration sample from one directory.
- `--calib-cache` (YOLO): platform independent but keyed to the TensorRT version and algorithm name.
  - The cache holds one scale per tensor plus a header such as `TRT-101401-EntropyCalibration2`. It records nothing about the GPU (no architecture, sm tag or CPU arch), so it carries to another machine and keeps a Jetson follow-up short. TensorRT writes its own version string and the algorithm name into the header and rejects a mismatch. A JetPack that ships a different TensorRT recalibrates from frames.
  - `resolve_int8_config()` writes the list of calibration frames next to the cache (`<cache>.frames.txt`). It is the audit trail for which frames produced which scales, and the only way to check that a second sample is disjoint from the first.
- Calibration is implicit, which TensorRT deprecates in favour of explicit Q/DQ. Q/DQ is a different export pipeline and out of scope. `_configure_int8()` suppresses the one deprecation warning on that assignment.
- The calibrator feeds the network its real geometry. `_configure_int8()` reads the size from the parsed network input shape, not from `--imgsz`, so 384x640 arms need no second flag.
- `fair_scene_allocation()` splits `--calib-count` frames across scenes by max-min fairness. It is deterministic and uses no RNG. The val split is heavily imbalanced (its three largest cage cameras hold a quarter of the frames), so a plain stride would calibrate mostly on them. Water-filling gives every scene an equal share and re-divides what a scene cannot use among scenes that still have frames.
- `--calib-algo` (YOLO): `entropy2` clips outliers to keep resolution on the common case. `minmax` never clips.
- `--timing-cache` (YOLO and DeepLab): tactic timings are reusable across models of the same architecture, so the first build pays the autotuning cost and the rest do not. The cache lives at `.cache/tensorrt/timing.cache` in the checkout (`DEFAULT_TIMING_CACHE` in `auto_battlebot/tensorrt_build.py`). It is gitignored and disposable, and it stays out of `$HOME` so it cannot outlive the project and go stale unnoticed after a TensorRT or driver upgrade. `convert_to_tensorrt.py` builds sequentially in one process: concurrent builders each pay a full tactic search, and sequential builds keep the cache warm in memory between models.
