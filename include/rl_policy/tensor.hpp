#pragma once

#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace auto_battlebot::rl_policy {

struct TensorSpec {
    std::string name;
    std::vector<int> shape;

    size_t size() const;
    std::string to_string() const;

    bool operator==(const TensorSpec &other) const = default;
};

/** One named float tensor. Its storage is sized once, at construction, and never resized. */
struct Tensor {
    explicit Tensor(TensorSpec tensor_spec);

    TensorSpec spec;
    std::vector<float> data;

    std::span<float> values() { return data; }
    std::span<const float> values() const { return data; }
};

/** Named tensors in a fixed order, allocated at load. Lookups by name do not allocate, so the
 * control loop can use them every tick. */
class TensorSet {
   public:
    TensorSet() = default;
    explicit TensorSet(const std::vector<TensorSpec> &specs);

    Tensor *find(std::string_view name);
    const Tensor *find(std::string_view name) const;

    size_t size() const { return tensors_.size(); }
    Tensor &operator[](size_t index) { return tensors_[index]; }
    const Tensor &operator[](size_t index) const { return tensors_[index]; }

    std::vector<TensorSpec> specs() const;

    void fill(float value);

   private:
    std::vector<Tensor> tensors_;
};

std::string specs_to_string(const std::vector<TensorSpec> &specs);

}  // namespace auto_battlebot::rl_policy
