# Shared YOLO input preprocess: tried, measured, reverted

I shared the preprocessed input tensor (BGR->RGB, letterbox, 1/255 normalize, CHW pack)
between the keypoint and robot blob models so it was computed once per frame instead of
once per model. It worked and was correct, but the win was too small to justify the
plumbing: 0.5 ms on the desktop, 1.1 ms on the Jetson. The change is reverted; this
report records the numbers and the lesson.

## Hypothesis

Per-model update time on the Jetson is ~12 ms while the GPU inference itself is ~1-3 ms
(trtexec, seg_vs_bbox_2026-07-18), so CPU work dominates. Both models run an identical
preprocess on the identical frame every tick, concurrently on two worker threads. Sharing
it should reclaim roughly one preprocess pass per tick.

## Implementation (reverted)

`YoloInputCache`: compute-once cache keyed by frame (data pointer + stamp) and spec
(input size + letterbox padding bits). Concurrent callers for the same frame shared one
computation via a condition variable with a bounded wait. Injected optionally into the
three YOLO models through the factories; models without a cache preprocessed locally.
Correctness was covered by unit tests (identical output to the per-model path, frame
invalidation, concurrent sharing).

## Results

| metric | before | after | saved |
| --- | ---: | ---: | ---: |
| desktop replay (15-35), perception batch mean | 7.85 ms | 7.33 ms | 0.52 ms |
| Jetson live, perception batch mean | 12.86 ms | 11.74 ms | 1.12 ms |
| Jetson live, per-model means | 12.22 / 11.90 ms | 11.48 / 11.14 ms | ~0.7 ms each |

Jetson runs: 15-18-13 (before) vs 16-15-54 (after), both parallel + streams,
max_loop_rate 60, `--after-field-init`. The 16-15-54 run's large end-to-end improvement
(75.2 -> 61.6 ms mean) came from the publish_camera_data guard + reorder that landed in
the same build, not from preprocess sharing.

## Why the win was small

The input side was already cheap: preprocess is one 720p->640 letterbox + normalize +
CHW pack, ~1 ms on desktop and evidently only ~1 ms on the Jetson too. The ~11 ms that
remains per model is the output side: decoding 8,400 anchor rows and CPU NMS per model,
plus the execute sync. The hypothesis pointed at the wrong end of the pipeline.

## Decision

Reverted. The cache threaded a shared object through three model constructors, two
factories, and main.cpp, and coupled models that are otherwise independent, for ~1 ms.
Not worth the architectural complexity.

## Next steps

1. Attack the output side instead: vectorize the anchor decode, or export engines with
   NMS baked in (Ultralytics `nms=True` / TensorRT EfficientNMS) so the engine returns
   final detections and the CPU decode/NMS disappears entirely.
2. If postprocess-in-engine lands, re-measure; the perception batch should approach the
   raw GPU inference time.
