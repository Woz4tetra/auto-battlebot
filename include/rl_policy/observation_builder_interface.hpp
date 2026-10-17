#pragma once

#include <memory>
#include <vector>

#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "data_structures/target_selection.hpp"
#include "rl_policy/action_history.hpp"
#include "rl_policy/tensor.hpp"

namespace auto_battlebot::rl_policy {

/** Everything an observation is built from on one policy step. The navigation picks the two
 * robots; the builder reads them, the field and the history. */
struct ObservationContext {
    const RobotDescription &our_robot;
    /** Null when there is no target opponent. */
    const RobotDescription *target_opponent = nullptr;
    const FieldDescription &field;
    const TargetSelection &target;
    const ActionHistory &history;
};

/**
 * One observation spec version: robots, field, target and history to named tensors. The output
 * depends on the context and nothing else. A new spec version is a new builder; old bundles keep
 * loading because each bundle names the version it was trained on.
 */
class ObservationBuilderInterface {
   public:
    virtual ~ObservationBuilderInterface() = default;

    virtual int spec_version() const = 0;

    /** The tensors build() writes, by name and shape. */
    virtual std::vector<TensorSpec> output_specs() const = 0;

    /** Policy steps of history the spec reads. */
    virtual size_t history_length() const = 0;

    /** Writes every tensor output_specs() names into `observation`. Does not allocate once its
     * scratch space has grown to the field's hazard count. */
    virtual void build(const ObservationContext &context, TensorSet &observation) = 0;
};

/** The builder for `obs_spec_version`, or null when this build has none. */
std::unique_ptr<ObservationBuilderInterface> make_observation_builder(int obs_spec_version);

}  // namespace auto_battlebot::rl_policy
