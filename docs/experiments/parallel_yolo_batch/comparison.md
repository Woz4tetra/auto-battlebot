# Parallel YOLO batch: latency A/B on NHRL May recordings

Parallelizing the keypoint and robot-blob models needed two changes to pay off. Threading
alone (ParallelModelBatch, both engines on the default CUDA stream) saved 0.66 ms per tick
on average. Adding a per-engine non-blocking CUDA stream with pinned-memory async IO
raised that to 1.73 ms per tick: the parallel section runs at 7.85 ms mean vs 9.58 ms for
the sequential sum, and every recording improved. Tick mean is flat (28.1 to 27.8 ms)
because perception is a small share of the desktop tick; the Jetson, where the two models
are 71% of the tick, is where this should matter.

## Setup

- Branch: `benw/performance-evaluation`, 2026-07-24.
- Three phases on the same machine (x86 dev box, desktop GPU), same binary except the
  perception section:
  1. `sequential/`: models called back to back in `Runner::tick` (temporary rollback).
  2. `parallel/`: `ParallelModelBatch`, two worker threads, default CUDA stream.
  3. `parallel_streams/`: `ParallelModelBatch` plus per-engine `cudaStream_t`
     (non-blocking), pinned host staging buffers, async H2D/D2H, `enqueueV3(stream)`,
     one stream sync per execute.
- 7 NHRL May recordings replayed via SVO playback, `playback/mrs_buff_mk3_playback` base,
  real-time mode, prescribed `svo_start_frame` per recording from
  `config/playback/_playback.toml`.
- Frame stamps are rebased to wall clock at replay start (fix added this session), so
  `pipeline.latency` is finite in replay. It still carries replay pacing artifacts, so
  use it only for phase-vs-phase deltas, not as an absolute latency measurement.

## Results (mean ms)

| Recording | seq sum | par batch | streams batch | saved vs seq | tick seq/par/streams |
| --- | ---: | ---: | ---: | ---: | ---: |
| 2026-05-01T17-42-24 | 9.61 | 9.00 | 7.90 | 1.71 | 37.69 / 37.53 / 37.16 |
| 2026-05-02T10-06-06 | 9.89 | 8.79 | 7.72 | 2.16 | 26.24 / 26.33 / 26.33 |
| 2026-05-02T11-45-08 | 10.36 | 9.24 | 8.17 | 2.18 | 26.03 / 26.22 / 26.29 |
| 2026-05-02T14-12-27 | 8.83 | 8.99 | 7.84 | 0.98 | 27.40 / 26.00 / 26.19 |
| 2026-05-02T15-35-04 | 9.37 | 8.92 | 7.85 | 1.52 | 26.73 / 26.14 / 26.30 |
| 2026-05-02T16-18-07 | 9.45 | 8.72 | 7.57 | 1.88 | 26.18 / 25.85 / 26.16 |
| 2026-05-02T17-26-14 | 9.56 | 8.76 | 7.89 | 1.67 | 26.24 / 26.21 / 26.16 |
| **mean** | **9.58** | **8.92** | **7.85** | **1.73** | **28.07 / 27.75 / 27.80** |

Tick p95 means across recordings: 46.94 / 47.61 / 46.98 (unchanged within noise).

Per-model means under each phase:

- Sequential (models run alone): keypoint 5.48 ms, blob 4.10 ms.
- Parallel, default stream: keypoint 8.23 ms, blob 8.08 ms. The default stream serializes
  the GPU work, so each model waits behind the other and the batch (8.92 ms) barely beats
  the sequential sum.
- Parallel, per-engine streams: keypoint 7.33 ms, blob 7.05 ms, batch 7.85 ms. Concurrent
  models still stretch vs running alone (shared GPU compute), but copies and kernels now
  interleave across streams and the TensorRT default-stream warning is gone.

## Comparison to the Jetson baselines

The reports in `docs/experiments/baseline_latency/` are live Jetson runs from the May
event (tick mean 42.5 ms, keypoint 11.4 ms, seg blob 19.0 ms sequential). They are not
directly comparable to these desktop replays: different GPU, different blob model (seg vs
bbox), and replay pacing offsets. On the Jetson the sequential perception section is 30.4
of 42.5 ms per tick, so even a partial overlap there is worth several milliseconds of the
60 ms budget.

## Conclusion

- Bug/Issue: threading alone does not parallelize the models; both engines serialized on
  the default CUDA stream and each model's latency inflated under concurrency.
- Fix/Solution: per-engine non-blocking CUDA streams with pinned staging buffers and
  async memcpy in `TrtEngine`. Perception section now 7.85 ms vs 9.58 ms sequential on
  the desktop, an 18% cut with no accuracy-relevant changes.

## Next steps

1. Rerun this A/B on the Jetson, where the perception share of the tick (71%) makes the
   expected win several times larger.
2. If the Jetson still shows heavy compute contention, evaluate running the two engines
   at different priorities (`cudaStreamCreateWithPriority`) or merging the two models
   into one engine.
