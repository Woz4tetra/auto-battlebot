#pragma once

#include <span>

#include "plant/plant_model_interface.hpp"
#include "plant/plant_params.hpp"

namespace auto_battlebot {

/**
 * PlantModelInterface backed by the jig-fitted grey-box model. Mirrors plant.py exactly:
 * per-sign deadzone and gain, transport delay read from the command span, asymmetric
 * first-order lag, exact arc integration at 2 ms substeps.
 *
 * Commands are the normalized [-1, 1] stick commands the transmitter sends, carried in
 * VelocityCommand as linear_x (forward) and angular_z (yaw); linear_y is ignored (tank
 * drive). The gains map them to m/s and rad/s, so passing physical velocities here would
 * apply the gains twice.
 *
 * The command span must cover [t0 - delay_s, t1]; CommandRingBuffer::gather builds exactly
 * that. The command is re-read every substep, zero-order held, and zero before the first
 * entry, matching Plant.command_at in plant.py.
 *
 * The Jacobian is a central finite difference over the five states. The model is piecewise
 * (deadzone, coupling clamp, sign-selected tau), so an analytic Jacobian is
 * branch-following; numeric is the choice until profiling argues otherwise.
 */
class JigPlantModel final : public PlantModelInterface {
   public:
    explicit JigPlantModel(const JigPlantParams &params,
                           const JigPlantNoiseParams &noise = JigPlantNoiseParams{});

    void reset() override;

    PlantState propagate(const PlantState &state, std::span<const TimedCommand> commands, double t0,
                         double t1, Eigen::Matrix<double, 5, 5> &jacobian) override;

    Eigen::Matrix<double, 5, 5> process_noise(const PlantState &state,
                                              std::span<const TimedCommand> commands,
                                              double dt) override;

    const JigPlantParams &params() const { return params_; }
    const JigPlantNoiseParams &noise() const { return noise_; }

    /** Propagate without wrapping theta, exposed for the finite difference and for tests
     * that accumulate heading past pi. */
    PlantState propagate_unwrapped(const PlantState &state, std::span<const TimedCommand> commands,
                                   double t0, double t1) const;

   private:
    JigPlantParams params_;
    JigPlantNoiseParams noise_;
};

}  // namespace auto_battlebot
