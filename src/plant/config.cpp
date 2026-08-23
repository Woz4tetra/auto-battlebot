#include "plant/config.hpp"

#include "config/config_parser.hpp"

namespace auto_battlebot {
namespace {
/**
 * Every plant field is required: these are fitted physical parameters, and a partly defaulted
 * plant would predict with a mix of fit and zero that no experiment produced. A ladder rung
 * disables a term by writing it as zero in the config, same as the fit output files.
 */
JigPlantParams parse_params(ConfigParser &parser) {
    JigPlantParams params;
    params.dz_lin_fwd = parser.get_required_double("dz_lin_fwd");
    params.dz_lin_rev = parser.get_required_double("dz_lin_rev");
    params.dz_ang_l = parser.get_required_double("dz_ang_l");
    params.dz_ang_r = parser.get_required_double("dz_ang_r");
    params.k_fwd = parser.get_required_double("k_fwd");
    params.k_rev = parser.get_required_double("k_rev");
    params.k_ang = parser.get_required_double("k_ang");
    params.tau_lin_a = parser.get_required_double("tau_lin_a");
    params.tau_lin_d = parser.get_required_double("tau_lin_d");
    params.tau_ang_a = parser.get_required_double("tau_ang_a");
    params.tau_ang_d = parser.get_required_double("tau_ang_d");
    params.delay_s = parser.get_required_double("delay_s");
    params.c_sb = parser.get_required_double("c_sb");
    params.c_ad = parser.get_required_double("c_ad");
    params.c_drift = parser.get_required_double("c_drift");
    params.c_drift_bias = parser.get_required_double("c_drift_bias");
    return params;
}

/** Optional over the baked-in fit values, unlike the params above. */
void parse_noise(ConfigParser &plant_parser, JigPlantNoiseParams &noise) {
    const toml::table *table_ptr = plant_parser.get_table("process_noise");
    if (!table_ptr) {
        return;
    }
    ConfigParser parser(*table_ptr, "plant.process_noise");
    noise.q_along = parser.get_optional_double("q_along", noise.q_along);
    noise.q_cross = parser.get_optional_double("q_cross", noise.q_cross);
    noise.q_heading = parser.get_optional_double("q_heading", noise.q_heading);
    noise.scale_factor = parser.get_optional_double("scale_factor", noise.scale_factor);
    noise.heading_scale_factor =
        parser.get_optional_double("heading_scale_factor", noise.heading_scale_factor);
    noise.heading_random_walk =
        parser.get_optional_double("heading_random_walk", noise.heading_random_walk);
    noise.delay_jitter_s = parser.get_optional_double("delay_jitter_s", noise.delay_jitter_s);
    parser.validate_no_extra_fields();
}
}  // namespace

PlantConfiguration load_plant_from_toml(const toml::table &toml_data,
                                        std::vector<std::string> &parsed_sections) {
    PlantConfiguration config;
    const toml::table *section = toml_data["plant"].as_table();
    if (!section) {
        return config;
    }
    ConfigParser parser(*section, "plant");
    config.params = parse_params(parser);
    parse_noise(parser, config.noise);
    parser.validate_no_extra_fields();
    parsed_sections.push_back("plant");
    return config;
}

}  // namespace auto_battlebot
