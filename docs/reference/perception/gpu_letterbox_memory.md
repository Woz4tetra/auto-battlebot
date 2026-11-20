# Gpu letterbox memory

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/tensorrt_inference/gpu_letterbox.cpp`

Beside `const cudaError_t err =`:

From pageable memory the copy has finished reading `image` by the time it returns.

## Test tolerances

- `expect_close` in `tests/tensorrt_inference/test_gpu_letterbox.cpp` allows one grey level per sample and a tighter bound on the mean. `cv::resize` rounds each 8-bit output and quantizes its weights to 11 bits. The kernel does neither. Unverified: the 11-bit figure comes from the old comment.
