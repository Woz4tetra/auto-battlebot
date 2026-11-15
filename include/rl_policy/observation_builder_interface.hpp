#pragma once

#include <memory>
#include <vector>

#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "data_structures/target_selection.hpp"
#include "rl_policy/action_history.hpp"
#include "rl_policy/tensor.hpp"

namespace auto_battlebot::rl_policy {

struct ObservationContext {
    const RobotDescription &our_robot;
    const RobotDescription *target_opponent = nullptr;
    const FieldDescription &field;
    const TargetSelection &target;
    const ActionHistory &history;
};

class ObservationBuilderInterface {
   public:
    virtual ~ObservationBuilderInterface() = default;

    virtual int spec_version() const = 0;

    virtual std::vector<TensorSpec> output_specs() const = 0;

    virtual size_t history_policy_steps() const = 0;

    /** Writes every tensor output_specs() names into `observation`. Does not allocate once its
     * scratch space has grown to the field's hazard count. */
    virtual void build(const ObservationContext &context, TensorSet &observation) = 0;
};

std::unique_ptr<ObservationBuilderInterface> make_observation_builder(int obs_spec_version);

}  // namespace auto_battlebot::rl_policy
