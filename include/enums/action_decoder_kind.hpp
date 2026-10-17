#pragma once

namespace auto_battlebot {
/** How a policy network's raw output becomes a stick command. SQUASHED is tanh, then a scale. */
enum class ActionDecoderKind { SQUASHED };
}  // namespace auto_battlebot
