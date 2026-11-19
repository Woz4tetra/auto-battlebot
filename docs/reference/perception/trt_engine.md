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
