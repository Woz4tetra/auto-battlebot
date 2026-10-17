#include "rl_policy/runtimes/rtneural_runtime.hpp"

#include <RTNeural/RTNeural.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace auto_battlebot::rl_policy {
namespace {

/** Layer types RTNeural::json_parser::parseJson builds. It skips any other type without a word,
 * which would leave a shorter network than the file describes. */
constexpr std::array<std::string_view, 10> kLayerTypes = {"dense",       "time-distributed-dense",
                                                          "conv1d",      "conv2d",
                                                          "gru",         "lstm",
                                                          "prelu",       "batchnorm",
                                                          "batchnorm2d", "activation"};

/** Activations RTNeural's createActivation knows. For any other name it returns null and the
 * model crashes on its first forward pass. Empty means no activation. */
constexpr std::array<std::string_view, 6> kActivations = {"",        "tanh",    "relu",
                                                          "sigmoid", "softmax", "elu"};

bool contains(std::span<const std::string_view> names, std::string_view name) {
    return std::find(names.begin(), names.end(), name) != names.end();
}

[[noreturn]] void fail(const NetworkSpec &spec, const std::string &message) {
    throw PolicyBundleError("network " + spec.name + " (" + spec.file.string() + "): " + message);
}

/** What parseJson would accept silently but build wrong. Shape errors it would only catch as an
 * out-of-range exception, or not at all for a short kernel (zero-padded), are named here. */
void check_layers(const NetworkSpec &spec, const nlohmann::json &model) {
    if (!model.contains("in_shape") || !model.at("in_shape").is_array() ||
        model.at("in_shape").empty()) {
        fail(spec, "in_shape is missing or empty");
    }
    if (!model.contains("layers") || !model.at("layers").is_array() || model.at("layers").empty()) {
        fail(spec, "layers is missing or empty");
    }
    const nlohmann::json &in_shape = model.at("in_shape");
    int previous_size = in_shape.back().is_number_integer() ? in_shape.back().get<int>() : -1;
    const nlohmann::json &layers = model.at("layers");
    for (size_t i = 0; i < layers.size(); ++i) {
        const nlohmann::json &layer = layers.at(i);
        const std::string at = "layers[" + std::to_string(i) + "] ";
        if (!layer.contains("type") || !layer.at("type").is_string()) {
            fail(spec, at + "has no type");
        }
        const std::string type = layer.at("type").get<std::string>();
        if (!contains(kLayerTypes, type)) {
            fail(spec, at + "type '" + type + "' is not a layer RTNeural builds");
        }
        if (layer.contains("activation")) {
            const std::string activation = layer.at("activation").get<std::string>();
            if (!contains(kActivations, activation)) {
                fail(spec, at + "activation '" + activation + "' is not one RTNeural builds");
            }
        }
        if (type != "dense") {
            previous_size = -1;  // Only Dense layers are shape-checked here.
            continue;
        }
        const int out_size = layer.at("shape").back().get<int>();
        const nlohmann::json &weights = layer.at("weights");
        if (!weights.is_array() || weights.size() != 2) {
            fail(spec, at + "Dense weights must be [kernel, bias]");
        }
        const nlohmann::json &kernel = weights.at(0);
        const nlohmann::json &bias = weights.at(1);
        if (previous_size >= 0 && static_cast<int>(kernel.size()) != previous_size) {
            fail(spec, at + "Dense kernel has " + std::to_string(kernel.size()) +
                           " rows; the layer's input has " + std::to_string(previous_size));
        }
        for (const auto &row : kernel) {
            if (static_cast<int>(row.size()) != out_size) {
                fail(spec, at + "Dense kernel row has " + std::to_string(row.size()) +
                               " entries; the layer's shape says " + std::to_string(out_size));
            }
        }
        if (static_cast<int>(bias.size()) != out_size) {
            fail(spec, at + "Dense bias has " + std::to_string(bias.size()) +
                           " entries; the layer's shape says " + std::to_string(out_size));
        }
        previous_size = out_size;
    }
}

}  // namespace

struct RtNeuralRuntime::Model {
    std::unique_ptr<RTNeural::Model<float>> network;
    /** RTNeural maps a layer's input as aligned memory, so the input is staged here rather than
     * read from the caller's tensor. */
    std::vector<float, Eigen::aligned_allocator<float>> input;
};

RtNeuralRuntime::RtNeuralRuntime() = default;
RtNeuralRuntime::~RtNeuralRuntime() = default;

void RtNeuralRuntime::load(const NetworkSpec &spec) {
    if (spec.inputs.size() != 1 || spec.outputs.size() != 1) {
        fail(spec,
             "RTNeural runs one input tensor to one output tensor; the manifest lists inputs " +
                 specs_to_string(spec.inputs) + " and outputs " + specs_to_string(spec.outputs));
    }
    std::ifstream file(spec.file);
    if (!file) fail(spec, "cannot open the file");

    auto model = std::make_unique<Model>();
    try {
        const nlohmann::json json = nlohmann::json::parse(file);
        check_layers(spec, json);
        model->network = RTNeural::json_parser::parseJson<float>(json);
    } catch (const PolicyBundleError &) {
        throw;
    } catch (const std::exception &e) {
        fail(spec, std::string("RTNeural could not load it: ") + e.what());
    }
    if (!model->network || model->network->layers.empty()) {
        fail(spec, "RTNeural built an empty model");
    }

    const auto in_size = static_cast<size_t>(model->network->getInSize());
    const auto out_size = static_cast<size_t>(model->network->getOutSize());
    if (in_size != spec.inputs.front().size()) {
        fail(spec, "the model takes " + std::to_string(in_size) + " inputs; the manifest says " +
                       spec.inputs.front().to_string());
    }
    if (out_size != spec.outputs.front().size()) {
        fail(spec, "the model gives " + std::to_string(out_size) + " outputs; the manifest says " +
                       spec.outputs.front().to_string());
    }

    model->input.assign(in_size, 0.0f);
    model->network->reset();
    model_ = std::move(model);
    inputs_ = TensorSet(spec.inputs);
    outputs_ = TensorSet(spec.outputs);
}

void RtNeuralRuntime::run() {
    const std::vector<float> &in = inputs_[0].data;
    std::copy(in.begin(), in.end(), model_->input.begin());
    model_->network->forward(model_->input.data());
    const float *out = model_->network->getOutputs();
    std::vector<float> &dst = outputs_[0].data;
    std::copy(out, out + dst.size(), dst.begin());
}

void RtNeuralRuntime::reset() {
    if (model_) model_->network->reset();
}

}  // namespace auto_battlebot::rl_policy
