#pragma once

#include <Eigen/Dense>
#include <span>

#include "data_structures/velocity.hpp"

namespace auto_battlebot {

struct PlantState {
    double x = 0.0;      // meters, field frame
    double y = 0.0;      // meters, field frame
    double theta = 0.0;  // radians, field frame, wrapped (-pi, pi]
    double v = 0.0;      // m/s, body forward
    double w = 0.0;      // rad/s, yaw rate
};

struct TimedCommand {
    double stamp = 0.0;
    StickCommand command{0.0, 0.0, 0.0};
};

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

    virtual Eigen::Matrix<double, 5, 5> process_noise(const PlantState &state,
                                                      std::span<const TimedCommand> commands,
                                                      double dt) = 0;
};

}  // namespace auto_battlebot
