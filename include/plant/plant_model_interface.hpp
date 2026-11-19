#pragma once

#include <Eigen/Dense>
#include <span>

#include "data_structures/velocity.hpp"

namespace auto_battlebot {

struct PlantState {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;
    double w = 0.0;
};

struct TimedCommand {
    double stamp = 0.0;
    StickCommand command{0.0, 0.0, 0.0};
};

class PlantModelInterface {
   public:
    virtual ~PlantModelInterface() = default;

    virtual void reset() = 0;

    virtual PlantState propagate(const PlantState &state, std::span<const TimedCommand> commands,
                                 double t0, double t1, Eigen::Matrix<double, 5, 5> &jacobian) = 0;

    virtual Eigen::Matrix<double, 5, 5> process_noise(const PlantState &state,
                                                      std::span<const TimedCommand> commands,
                                                      double dt) = 0;
};

}  // namespace auto_battlebot
