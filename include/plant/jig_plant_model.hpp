#pragma once

#include <span>

#include "plant/plant_model_interface.hpp"
#include "plant/plant_params.hpp"

namespace auto_battlebot {

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

    PlantState propagate_unwrapped(const PlantState &state, std::span<const TimedCommand> commands,
                                   double t0, double t1) const;

   private:
    JigPlantParams params_;
    JigPlantNoiseParams noise_;
};

}  // namespace auto_battlebot
