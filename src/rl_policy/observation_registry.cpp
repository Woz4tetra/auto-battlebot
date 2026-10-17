#include "rl_policy/observation_builder_interface.hpp"
#include "rl_policy/observations/observation_v1.hpp"

namespace auto_battlebot::rl_policy {

std::unique_ptr<ObservationBuilderInterface> make_observation_builder(int obs_spec_version) {
    switch (obs_spec_version) {
        case obs_v1::kSpecVersion:
            return std::make_unique<ObservationBuilderV1>();
        default:
            return nullptr;
    }
}

}  // namespace auto_battlebot::rl_policy
