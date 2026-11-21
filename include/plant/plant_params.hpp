#pragma once

#include "data_structures/velocity.hpp"

namespace auto_battlebot {

struct JigPlantParams {
    double dz_lin_fwd = 0.0;
    double dz_lin_rev = 0.0;
    double dz_ang_l = 0.0;
    double dz_ang_r = 0.0;
    double k_fwd = 0.0;
    double k_rev = 0.0;
    double k_ang = 0.0;
    double tau_lin_a = 0.0;
    double tau_lin_d = 0.0;
    double tau_ang_a = 0.0;
    double tau_ang_d = 0.0;
    double delay_s = 0.0;
    double c_sb = 0.0;
    double c_ad = 0.0;
    double c_drift = 0.0;
    double c_drift_bias = 0.0;
};

struct JigPlantNoiseParams {
    double q_along = 0.37125;
    double q_cross = 0.351167;
    double q_heading = 0.0;
    double scale_factor = 0.541056;
    double heading_scale_factor = 1.7783;
    double heading_random_walk = 0.0703417;
    double delay_jitter_s = 0.0;
};

double plant_effective_command(double u, double dz_pos, double dz_neg);

void plant_steady_state(double u_lin, double u_ang, const JigPlantParams &params, double &v_target,
                        double &w_target);

}  // namespace auto_battlebot
