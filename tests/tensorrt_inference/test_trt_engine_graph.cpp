#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <random>
#include <vector>

#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
namespace {

// The shipped keypoint engine for this machine, if the dev box has it. Engines are per GPU
// architecture and TensorRT version, so the test skips rather than fails without one.
std::filesystem::path find_engine() {
    const std::filesystem::path models =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "data" /
        "models";
    for (const char *name :
         {"yolo26s-pose_d50000_cagehigh_rect384x640_2026-09-20_x86_64_sm89.engine",
          "yolo26s-pose_d50000_cagehigh_rect384x640_2026-09-20_aarch64_sm87.engine"}) {
        if (std::filesystem::exists(models / name)) return models / name;
    }
    return {};
}

// The CUDA graph must reproduce the plain enqueue exactly, and the device-input path must match
// the host-input path, or inference changes silently when the graph takes over.
TEST(TrtEngineGraphTest, GraphAndDeviceInputMatchPlainExecute) {
    const std::filesystem::path engine_path = find_engine();
    if (engine_path.empty()) GTEST_SKIP() << "no keypoint engine for this machine";

    TrtEngine engine;
    {
        TrtEngine::ScopedQuietLogging quiet;
        if (!engine.load(engine_path.string())) GTEST_SKIP() << "engine does not load here";
    }

    std::vector<float> input(static_cast<size_t>(engine.getInputNumElements()));
    std::mt19937 rng(0);
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    for (float &v : input) v = dist(rng);

    // First run is plain enqueue; the graph is captured right after it.
    std::vector<float> plain(static_cast<size_t>(engine.getOutputNumElements()));
    ASSERT_TRUE(engine.execute(input.data(), plain.data()));
    EXPECT_TRUE(engine.uses_cuda_graph());

    std::vector<float> graphed(plain.size());
    ASSERT_TRUE(engine.execute(input.data(), graphed.data()));
    for (size_t i = 0; i < plain.size(); ++i) ASSERT_FLOAT_EQ(graphed[i], plain[i]) << "at " << i;

    ASSERT_EQ(cudaMemcpyAsync(engine.device_input(), input.data(), engine.getInputSizeBytes(),
                              cudaMemcpyHostToDevice, static_cast<cudaStream_t>(engine.stream())),
              cudaSuccess);
    const float *device_path = engine.execute_device_input();
    ASSERT_NE(device_path, nullptr);
    for (size_t i = 0; i < plain.size(); ++i) {
        ASSERT_FLOAT_EQ(device_path[i], plain[i]) << "at " << i;
    }
}

}  // namespace
}  // namespace auto_battlebot
