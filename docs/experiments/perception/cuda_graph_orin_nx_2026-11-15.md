# Cuda graph orin nx

## CUDA graph capture

A graph replays every kernel with one launch instead of one per layer. On the Orin NX, trtexec put the yolo26l-pose engine at 12.02 ms of GPU time without one and 10.62 ms with. One attempt per engine. TensorRT needs one ordinary enqueueV3 before a capture, so this runs after the first successful inference (the model's warmup), with the stream idle.
