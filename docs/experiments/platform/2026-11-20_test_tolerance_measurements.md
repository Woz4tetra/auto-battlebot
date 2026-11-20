# Test tolerance measurements

Dev box (x86_64, A6000 and CUDA 13, load average about 3), Debug build in `build-test/`.

## RtNeural full-size actor median

`RtNeuralRuntimeTest.FullSizeActorInferenceTime`, 138-256-256-128-2 ELU, 5000 runs per invocation, 5 invocations.

- Medians: 8.603, 8.412, 8.5, 8.445, 8.452 us. p99 9.6 to 10.6 us.
- The ceiling `kLoadedBoxMedianCeilingUs` stays at 2000 us. That is 235x the measured median, so a loaded box (load average 25+) still passes and a 100x slowdown still fails.

## GPU letterbox against the CPU letterbox

`GpuLetterboxTest`, 25 source shapes (widths 1920, 1280, 2208, 1000, 777 by heights 1200, 720, 1242, 1080, 555) times 3 random images.

- Worst per-sample difference: 0.822 grey levels. The bound is 1.01 grey levels.
- Worst mean difference: 0.261 grey levels. The CPU path rounds to uint8, which alone contributes 0.25 grey levels of mean absolute error on random data. The bound is 0.25 plus 0.05 headroom.
- The 11-bit weight claim was not isolated. The bound comes from the measurement, and `ManySourceShapesStayWithinOneGreyLevel` keeps the 25 shapes under it.

## Jig plant golden fixture

`JigPlantModelTest.GoldenTickByTickMatchesPythonReference` and `GoldenChunkedPropagateMatchesPythonReference` with the tolerance at 0.

- Worst difference: 4.09e-14 (x86_64, libstdc++ libm).
- `kGoldenTol` is now 1e-12, 24x the measured worst case. It was 1e-9.
- aarch64 libm was not measured.
