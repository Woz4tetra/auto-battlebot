#include <NvInfer.h>
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <vector>

#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
namespace {

class SilentLogger : public nvinfer1::ILogger {
   public:
    void log(Severity, nvinfer1::AsciiChar const *) noexcept override {}
};

constexpr int kTinyChannels = 3;
constexpr int kTinySide = 16;

bool build_tiny_relu_engine(const std::filesystem::path &path) {
    SilentLogger logger;
    std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(logger));
    if (!builder) return false;
    std::unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(0));
    std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
    if (!network || !config) return false;
    nvinfer1::ITensor *input =
        network->addInput("images", nvinfer1::DataType::kFLOAT,
                          nvinfer1::Dims4{1, kTinyChannels, kTinySide, kTinySide});
    nvinfer1::IActivationLayer *relu =
        network->addActivation(*input, nvinfer1::ActivationType::kRELU);
    if (!relu) return false;
    relu->getOutput(0)->setName("output0");
    network->markOutput(*relu->getOutput(0));
    std::unique_ptr<nvinfer1::IHostMemory> serialized(
        builder->buildSerializedNetwork(*network, *config));
    if (!serialized) return false;
    std::ofstream file(path, std::ios::binary);
    file.write(static_cast<const char *>(serialized->data()),
               static_cast<std::streamsize>(serialized->size()));
    return file.good();
}

TEST(TrtEngineGraphTest, GraphAndDeviceInputMatchPlainExecute) {
    const std::filesystem::path engine_path =
        std::filesystem::temp_directory_path() /
        ("trt_engine_graph_test_" + std::to_string(::getpid()) + ".engine");
    ASSERT_TRUE(build_tiny_relu_engine(engine_path));
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() {
            std::error_code ec;
            std::filesystem::remove(path, ec);
        }
    } cleanup{engine_path};

    TrtEngine engine;
    ASSERT_TRUE(engine.load(engine_path.string()));

    std::vector<float> input(static_cast<size_t>(engine.getInputNumElements()));
    std::mt19937 rng(0);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (float &v : input) v = dist(rng);

    std::vector<float> plain(static_cast<size_t>(engine.getOutputNumElements()));
    ASSERT_TRUE(engine.execute(input, plain));
    EXPECT_TRUE(engine.uses_cuda_graph());
    for (size_t i = 0; i < plain.size(); ++i) ASSERT_FLOAT_EQ(plain[i], std::max(input[i], 0.0f));

    std::vector<float> graphed(plain.size());
    ASSERT_TRUE(engine.execute(input, graphed));
    for (size_t i = 0; i < plain.size(); ++i) ASSERT_FLOAT_EQ(graphed[i], plain[i]) << "at " << i;

    ASSERT_EQ(cudaMemcpyAsync(engine.device_input(), input.data(), engine.getInputSizeBytes(),
                              cudaMemcpyHostToDevice, engine.stream()),
              cudaSuccess);
    const std::span<const float> device_path =
        engine.execute_borrowing_pinned_output_after_device_input();
    ASSERT_EQ(device_path.size(), plain.size());
    for (size_t i = 0; i < plain.size(); ++i) {
        ASSERT_FLOAT_EQ(device_path[i], plain[i]) << "at " << i;
    }
}

}  // namespace
}  // namespace auto_battlebot
