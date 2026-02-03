/**
 * TensorRT ONNX engine builder — builds a TensorRT engine from an ONNX file
 * and writes the serialized plan to disk. Exposed to Python via nanobind.
 */

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <fstream>
#include <stdexcept>
#include <cstdint>

namespace nb = nanobind;

// TensorRT 10+ removed destroy(); use delete. Older TRT used ptr->destroy().
#if NV_TENSORRT_MAJOR >= 10
#  define TRT_DESTROY(ptr) do { delete (ptr); (ptr) = nullptr; } while (0)
#else
#  define TRT_DESTROY(ptr) do { if (ptr) { (ptr)->destroy(); (ptr) = nullptr; } } while (0)
#endif

namespace {

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(nvinfer1::ILogger::Severity severity, char const* msg) noexcept override {
        (void)severity;
        if (msg)
            fprintf(stderr, "[TensorRT] %s\n", msg);
    }
};

}  // namespace

/** Build a TensorRT engine from an ONNX file and write it to engine_path. */
static void build_engine_from_onnx_impl(
    const std::string& onnx_path,
    const std::string& engine_path,
    bool fp16,
    int workspace_gib
) {
    TrtLogger logger;
    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(logger);
    if (!builder) {
        throw std::runtime_error("Failed to create TensorRT builder");
    }

    const auto explicit_batch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(explicit_batch);
    if (!network) {
        TRT_DESTROY(builder);
        throw std::runtime_error("Failed to create network");
    }

    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
    if (!config) {
        TRT_DESTROY(network);
        TRT_DESTROY(builder);
        throw std::runtime_error("Failed to create builder config");
    }

    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, logger);
    if (!parser) {
        TRT_DESTROY(config);
        TRT_DESTROY(network);
        TRT_DESTROY(builder);
        throw std::runtime_error("Failed to create ONNX parser");
    }

    if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        for (int i = 0; i < parser->getNbErrors(); ++i) {
            fprintf(stderr, "%s\n", parser->getError(i)->desc());
        }
        TRT_DESTROY(parser);
        TRT_DESTROY(config);
        TRT_DESTROY(network);
        TRT_DESTROY(builder);
        throw std::runtime_error("Failed to parse ONNX file: " + onnx_path);
    }

    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, static_cast<size_t>(workspace_gib) << 30);
    if (fp16 && builder->platformHasFastFp16()) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
#if defined(NV_TENSORRT_MAJOR) && NV_TENSORRT_MAJOR >= 8
    config->setFlag(nvinfer1::BuilderFlag::kVERSION_COMPATIBLE);
#endif

    nvinfer1::IHostMemory* plan = builder->buildSerializedNetwork(*network, *config);
    TRT_DESTROY(parser);
    TRT_DESTROY(config);
    TRT_DESTROY(network);
    TRT_DESTROY(builder);

    if (!plan) {
        throw std::runtime_error("Failed to build TensorRT engine");
    }

    std::ofstream out(engine_path, std::ios::binary | std::ios::out);
    if (!out) {
        TRT_DESTROY(plan);
        throw std::runtime_error("Failed to open engine output path: " + engine_path);
    }
    out.write(static_cast<const char*>(plan->data()), static_cast<std::streamsize>(plan->size()));
    TRT_DESTROY(plan);
    if (!out.good()) {
        out.close();
        throw std::runtime_error("Failed to write engine file: " + engine_path);
    }
    out.close();
}

NB_MODULE(trt_onnx_builder, m) {
    m.def(
        "build_engine_from_onnx",
        [](const std::string& onnx_path, const std::string& engine_path,
           bool fp16, int workspace_gib) {
            build_engine_from_onnx_impl(onnx_path, engine_path, fp16, workspace_gib);
        },
        nb::arg("onnx_path"),
        nb::arg("engine_path"),
        nb::arg("fp16") = true,
        nb::arg("workspace_gib") = 16,
        "Build a TensorRT engine from an ONNX file and write it to engine_path.\n"
        "Uses the same TensorRT C++ API as typical C++ inference runtimes for compatibility.\n"
        "fp16: use FP16 if True (default True). workspace_gib: workspace size in GiB (default 16)."
    );
}
