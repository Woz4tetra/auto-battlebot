#pragma once

#include <toml++/toml.h>

#include <optional>
#include <string>
#include <vector>

#include "plant/plant_params.hpp"

namespace auto_battlebot {

/**
 * The jig-fitted drivetrain plant, parsed from the top-level [plant] table.
 *
 * Two subsystems run on the same fit: the our-robot EKF propagates through JigPlantModel, and
 * MotionProfileNavigation inverts the same gains, time constants, and coupling terms for its
 * feedforward. They read one table so a refit lands in one place. Before this table existed the
 * navigation half carried its own copy of the numbers under controller-flavored names, and the
 * two drifted apart.
 */
struct PlantConfiguration {
    /** Absent when the config carries no [plant] table. Consumers that need it throw. */
    std::optional<JigPlantParams> params;

    /**
     * Process noise PSDs, from [plant.process_noise]. Fields are optional; the defaults are the
     * fit_process_noise.py output baked into JigPlantNoiseParams. Only the EKF reads these.
     */
    JigPlantNoiseParams noise;
};

/**
 * Parse the optional top-level [plant] section. Absence is not an error here: a profile that
 * runs neither the EKF nor MotionProfileNavigation needs no plant, and those that do raise
 * their own error when they find it missing.
 */
PlantConfiguration load_plant_from_toml(const toml::table &toml_data,
                                        std::vector<std::string> &parsed_sections);

}  // namespace auto_battlebot
