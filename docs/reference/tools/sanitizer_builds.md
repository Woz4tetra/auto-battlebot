# Sanitizer builds

`AUTO_BATTLEBOT_SANITIZER` (`none`, `address`, `thread`; default `none`) adds `-fsanitize` and `-fno-omit-frame-pointer` to every target. `scripts/build_and_test.sh` drives it:

```bash
./scripts/build_and_test.sh --asan                        # build-asan/, AddressSanitizer
./scripts/build_and_test.sh --tsan                        # build-tsan/, ThreadSanitizer
./scripts/build_and_test.sh --tsan --gtest_filter=Foo.*   # GTest arguments pass through
```

Both runtimes ship with GCC (`libgcc-13-dev`); nothing extra is installed.

## What the script sets

| Setting | Why |
| --- | --- |
| Builds only `auto_battlebot_test` | `viz_relay` includes Crow's asio headers, which call `atomic_thread_fence`. GCC rejects that under `-fsanitize=thread` with `-Werror=tsan` |
| `ASAN_OPTIONS=protect_shadow_gap=0:replace_intrin=0` | The CUDA libraries map memory in ASan's shadow gap and break with its intrinsic replacements |
| `setarch -R` (TSan) | TSan cannot map its shadow memory with ASLR on under this kernel (6.8) |
| `TSAN_OPTIONS=suppressions=tsan.supp` | OpenCV is not instrumented, so TSan cannot see its thread pool's synchronisation. Every report from `cv::ThreadPool` and `cv::aruco::ArucoDetector` is a false positive of that kind |
| `thread:test_zed_device_seams.cpp` in `tsan.supp` | `ZedDevice::open` leaks a wedged open thread by design so shutdown does not block. The `ZedDeviceOpenTimeoutTest` cases in that file check exactly that, so TSan's thread-leak report there is the behaviour under test |

Your own `ASAN_OPTIONS` and `TSAN_OPTIONS` are appended to these.

## Tests skipped under TSan

NVIDIA's codec libraries start their own threads, and TSan's `calloc` interceptor segfaults inside them (`libnvcuvid.so.1`). `tests/camera/nvidia_codec_under_tsan.hpp` defines `kNvidiaCodecCrashesUnderThisBuild`; these tests skip on it:

| Test | Library |
| --- | --- |
| `VideoEncoderTest` real-encoder cases | NVENC |
| `VideoPlaybackCameraTest.*`, `VideoPlaybackCameraJoinTest.*` | NVENC to record, NVDEC to play back |
| `ZedSvoPlaybackCameraTest.*` | ZED SDK SVO decode (NVDEC) |
| `Backends/H264BackendTest.*/ffmpeg_nvenc` | FFmpeg `h264_nvenc` |

The libx264 and GStreamer cases of `H264BackendTest` still run under TSan. Nothing is skipped under ASan.
