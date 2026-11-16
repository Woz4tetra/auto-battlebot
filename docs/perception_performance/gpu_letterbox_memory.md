# Gpu letterbox memory

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/tensorrt_inference/gpu_letterbox.cpp`

Beside `const cudaError_t err =`:

From pageable memory the copy has finished reading `image` by the time it returns.
