#pragma once

#include <toml++/toml.h>

#include <string>
#include <vector>

#include "plant/plant_params.hpp"

namespace auto_battlebot {

struct PlantConfiguration {
    JigPlantParams params;

    JigPlantNoiseParams noise;
};

PlantConfiguration load_plant_from_toml(const toml::table &toml_data,
                                        std::vector<std::string> &parsed_sections);

}  // namespace auto_battlebot
