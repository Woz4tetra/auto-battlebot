#pragma once

#include <memory>

#include "rl_policy/network_runtime_interface.hpp"

namespace auto_battlebot::rl_policy {

class RtNeuralRuntime : public NetworkRuntimeInterface {
   public:
    RtNeuralRuntime();
    ~RtNeuralRuntime() override;

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
