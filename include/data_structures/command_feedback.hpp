#pragma once

#include <map>

#include "data_structures/velocity.hpp"
#include "enums/frame_id.hpp"

namespace auto_battlebot {

/**
 * What the transmitter reports as currently commanded to each robot, in normalized [-1, 1] stick
 * space -- the same space the command was sent in, read back after mixing, deadzones, and
 * saturation.
 *
 * Physical units never appear here. Every consumer that needs m/s and rad/s converts with the
 * fitted [plant] gains at the point of use (plant_stick_to_body_velocity, or JigPlantModel for the
 * EKF), so the drivetrain scaling is defined once and a refit moves estimator and controller
 * together. This struct used to carry a second map in physical units, scaled from
 * transmitter-local wheel geometry; the EKF read it and applied the fitted gains a second time, a
 * 40x yaw-rate error at the Mrs Buff Mk3 scaling.
 *
 * VelocityCommand's field comments name m/s and rad/s; in this struct they are stick units.
 */
struct CommandFeedback {
    std::map<FrameId, VelocityCommand> stick_commands = std::map<FrameId, VelocityCommand>();
};

}  // namespace auto_battlebot
