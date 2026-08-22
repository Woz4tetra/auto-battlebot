#pragma once

#include <Eigen/Dense>
#include <span>

#include "data_structures/velocity.hpp"

namespace auto_battlebot {

/** Our-robot filter state: field-frame pose plus body-frame velocities (tank drive, no
 * lateral term). Mirrors the state in auto_battlebot/plant.py. */
struct PlantState {
    double x = 0.0;      // meters, field frame
    double y = 0.0;      // meters, field frame
    double theta = 0.0;  // radians, field frame, wrapped (-pi, pi]
    double v = 0.0;      // m/s, body forward
    double w = 0.0;      // rad/s, yaw rate
};

/** One commanded velocity with the control-loop time it was issued at. */
struct TimedCommand {
    double stamp = 0.0;
    VelocityCommand command{0.0, 0.0, 0.0};
};

/**
 * Process model for the our-robot EKF: commands in, predicted state and Jacobian out.
 *
 * The production implementation mirrors auto_battlebot/plant.py exactly (deadzone, per-sign
 * gain, transport delay, asymmetric first-order lag, arc integration with 2 ms substeps) and is
 * configured by the fitted plant_params.toml plus a model-structure name. None is registered
 * yet: the plant fit has not passed its acceptance criteria and no ladder rung is selected, so
 * this slot stays empty on purpose. Tests drive the EKF with a stub implementation instead.
 */
class PlantModelInterface {
   public:
    virtual ~PlantModelInterface() = default;

    virtual void reset() = 0;

    /**
     * Propagate from t0 to t1. `commands` is the command history covering at least
     * [t0 - transport_delay, t1], oldest first; the model reads it at its own delay offset.
     * Writes the state-transition Jacobian d(state at t1)/d(state at t0) for the covariance
     * propagation.
     */
    virtual PlantState propagate(const PlantState &state, std::span<const TimedCommand> commands,
                                 double t0, double t1, Eigen::Matrix<double, 5, 5> &jacobian) = 0;

    /** Continuous-time process noise mapped over [t0, t1], state- and input-dependent. */
    virtual Eigen::Matrix<double, 5, 5> process_noise(const PlantState &state,
                                                      std::span<const TimedCommand> commands,
                                                      double dt) = 0;
};

}  // namespace auto_battlebot
