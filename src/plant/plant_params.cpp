#include "plant/plant_params.hpp"

#include <algorithm>
#include <cmath>

namespace auto_battlebot {
namespace {
double sign_of(double value) { return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0); }
}  // namespace

double plant_effective_command(double u, double dz_pos, double dz_neg) {
    double dz = std::clamp(u >= 0.0 ? dz_pos : dz_neg, 0.0, 0.95);
    double mag = std::max(std::abs(u) - dz, 0.0) / (1.0 - dz);
    return sign_of(u) * mag;
}

void plant_steady_state(double u_lin, double u_ang, const JigPlantParams &params, double &v_target,
                        double &w_target) {
    const double lin_eff = plant_effective_command(u_lin, params.dz_lin_fwd, params.dz_lin_rev);
    const double ang_eff = plant_effective_command(u_ang, params.dz_ang_l, params.dz_ang_r);
    v_target = (lin_eff >= 0.0 ? params.k_fwd : params.k_rev) * lin_eff;
    w_target = params.k_ang * ang_eff;
    if (params.c_sb != 0.0) {
        v_target *= std::max(0.0, 1.0 - params.c_sb * std::abs(ang_eff));
    }
    if (params.c_ad != 0.0) {
        w_target *= std::max(0.0, 1.0 - params.c_ad * std::abs(lin_eff));
    }
    // Drift is additive: yaw the driver never asked for, not a scaling of the commanded yaw.
    if (params.c_drift != 0.0) {
        w_target += params.c_drift * lin_eff;
    }
    if (params.c_drift_bias != 0.0) {
        w_target += params.c_drift_bias * sign_of(lin_eff);
    }
}

VelocityCommand plant_stick_to_body_velocity(const VelocityCommand &stick,
                                             const JigPlantParams &params) {
    return VelocityCommand{
        .linear_x = (stick.linear_x >= 0.0 ? params.k_fwd : params.k_rev) * stick.linear_x,
        .linear_y = 0.0,
        .angular_z = params.k_ang * stick.angular_z,
    };
}

}  // namespace auto_battlebot
