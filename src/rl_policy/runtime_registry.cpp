#include "rl_policy/network_runtime_interface.hpp"
#include "rl_policy/runtimes/rtneural_runtime.hpp"

namespace auto_battlebot::rl_policy {

std::unique_ptr<NetworkRuntimeInterface> make_network_runtime(NetworkRuntimeKind kind) {
    switch (kind) {
        case NetworkRuntimeKind::RTNEURAL:
            return std::make_unique<RtNeuralRuntime>();
    }
    throw PolicyBundleError("no network runtime for this kind");
}

}  // namespace auto_battlebot::rl_policy
