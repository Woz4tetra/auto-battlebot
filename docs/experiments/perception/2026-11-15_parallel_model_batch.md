# Parallel model batch

`ParallelModelBatch` runs the keypoint model and the robot blob model in parallel on two persistent worker threads, joining on both before returning. It depends only on the model interfaces, so any implementation the factories select (TensorRT, noop, ...) stays swappable. Worker threads are created once at construction and woken per tick via a condition variable, mirroring the ZED producer/consumer handshake, to avoid per-frame thread creation in the hot loop.
