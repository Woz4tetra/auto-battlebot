#pragma once

namespace auto_battlebot {
/** How a policy bundle turns an observation into an action. NETWORK runs the bundle's networks.
 * CONSTANT returns the fixed action in its manifest, for the bench and the lockstep sim before a
 * trained bundle exists. */
enum class PolicyKind { NETWORK, CONSTANT };
}  // namespace auto_battlebot
