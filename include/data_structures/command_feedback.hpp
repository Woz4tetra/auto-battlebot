#pragma once

#include <map>

#include "data_structures/velocity.hpp"
#include "enums/frame_id.hpp"

namespace auto_battlebot {

/**
 * What the transmitter reports as currently commanded to each robot, in two spaces:
 *
 * - `commands`: body-frame velocity in physical units (m/s, rad/s), the transmitter's
 *   full-stick kinematic scaling applied. The dead-reckoning estimators integrate these
 *   directly.
 * - `stick_commands`: the same command in normalized [-1, 1] stick space, before physical
 *   scaling. The our-robot EKF pushes these into its command history: JigPlantModel maps
 *   stick to velocity through the fitted gains itself, so feeding it physical velocities
 *   applies the gains twice (a 40x yaw-rate error at the Mrs Buff Mk3 scaling).
 *
 * OpenTx and playback transmitters fill both maps for the same frames. SimTransmitter fills
 * only `stick_commands`: the sim plant consumes stick directly, so it has no physical
 * scaling to report.
 */
struct CommandFeedback {
    std::map<FrameId, VelocityCommand> commands = std::map<FrameId, VelocityCommand>();
    std::map<FrameId, VelocityCommand> stick_commands = std::map<FrameId, VelocityCommand>();
};

}  // namespace auto_battlebot
