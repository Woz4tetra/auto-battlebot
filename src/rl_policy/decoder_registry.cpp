#include "rl_policy/action_decoder_interface.hpp"
#include "rl_policy/decoders/squashed_decoder.hpp"

namespace auto_battlebot::rl_policy {

std::unique_ptr<ActionDecoderInterface> make_action_decoder(const DecoderSpec &spec) {
    switch (spec.kind) {
        case ActionDecoderKind::SQUASHED:
            return std::make_unique<SquashedDecoder>(spec.scale);
    }
    throw PolicyBundleError("no action decoder for this kind");
}

}  // namespace auto_battlebot::rl_policy
