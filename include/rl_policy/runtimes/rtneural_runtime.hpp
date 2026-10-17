#pragma once

#include <memory>

#include "rl_policy/network_runtime_interface.hpp"

namespace auto_battlebot::rl_policy {

/**
 * NetworkRuntimeInterface on RTNeural's dynamic model, loaded with
 * `RTNeural::json_parser::parseJson<float>` from the JSON auto_battlebot/rl/export/rtneural.py
 * writes. A sequential stack of the layers RTNeural supports, one input tensor and one output
 * tensor.
 *
 * This header does not include RTNeural; rtneural_runtime.cpp is the only file that does.
 */
class RtNeuralRuntime : public NetworkRuntimeInterface {
   public:
    RtNeuralRuntime();
    ~RtNeuralRuntime() override;

    /** Throws PolicyBundleError when the file does not parse, uses a layer type or activation
     * RTNeural would skip or crash on, has Dense weights of the wrong shape, or has input and
     * output sizes other than the spec's. */
    void load(const NetworkSpec &spec) override;

    TensorSet &inputs() override { return inputs_; }
    const TensorSet &outputs() const override { return outputs_; }

    void run() override;
    void reset() override;

   private:
    struct Model;
    std::unique_ptr<Model> model_;
    TensorSet inputs_;
    TensorSet outputs_;
};

}  // namespace auto_battlebot::rl_policy
