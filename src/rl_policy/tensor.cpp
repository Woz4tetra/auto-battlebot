#include "rl_policy/tensor.hpp"

#include <algorithm>
#include <utility>

namespace auto_battlebot::rl_policy {

size_t TensorSpec::size() const {
    size_t count = 1;
    for (int dim : shape) count *= static_cast<size_t>(std::max(dim, 0));
    return count;
}

std::string TensorSpec::to_string() const {
    std::string text = name + " [";
    for (size_t i = 0; i < shape.size(); ++i) {
        if (i > 0) text += ", ";
        text += std::to_string(shape[i]);
    }
    return text + "]";
}

Tensor::Tensor(TensorSpec tensor_spec) : spec(std::move(tensor_spec)), data(spec.size(), 0.0f) {}

TensorSet::TensorSet(const std::vector<TensorSpec> &specs) {
    tensors_.reserve(specs.size());
    for (const auto &spec : specs) tensors_.emplace_back(spec);
}

Tensor *TensorSet::find(std::string_view name) {
    for (auto &tensor : tensors_) {
        if (tensor.spec.name == name) return &tensor;
    }
    return nullptr;
}

const Tensor *TensorSet::find(std::string_view name) const {
    for (const auto &tensor : tensors_) {
        if (tensor.spec.name == name) return &tensor;
    }
    return nullptr;
}

std::vector<TensorSpec> TensorSet::specs() const {
    std::vector<TensorSpec> specs;
    specs.reserve(tensors_.size());
    for (const auto &tensor : tensors_) specs.push_back(tensor.spec);
    return specs;
}

void TensorSet::fill(float value) {
    for (auto &tensor : tensors_) std::fill(tensor.data.begin(), tensor.data.end(), value);
}

std::string specs_to_string(const std::vector<TensorSpec> &specs) {
    if (specs.empty()) return "(none)";
    std::string text;
    for (size_t i = 0; i < specs.size(); ++i) {
        if (i > 0) text += ", ";
        text += specs[i].to_string();
    }
    return text;
}

}  // namespace auto_battlebot::rl_policy
