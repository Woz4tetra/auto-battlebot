#pragma once

#include "data_structures/velocity.hpp"

namespace auto_battlebot {

struct JigPlantParams {
    double dz_lin_fwd = 0.0;
    double dz_lin_rev = 0.0;
    double dz_ang_l = 0.0;
    double dz_ang_r = 0.0;
    double k_fwd = 0.0;  // m/s at full effective command
    double k_rev = 0.0;
    double k_ang = 0.0;  // rad/s at full effective command
    double tau_lin_a = 0.0;
    double tau_lin_d = 0.0;
    double tau_ang_a = 0.0;
    double tau_ang_d = 0.0;
    double delay_s = 0.0;
    double c_sb = 0.0;     // steer-brake: fraction of forward authority lost per unit |u_ang_eff|
    double c_ad = 0.0;     // angular droop: fraction of yaw authority lost per unit |u_lin_eff|
    double c_drift = 0.0;  // rad/s per unit effective linear command
    double c_drift_bias = 0.0;  // rad/s while moving, signed by the linear command
};

struct JigPlantNoiseParams {
    double q_along = 0.37125;                // (m/s^2)^2/Hz, white-noise accel on v
    double q_cross = 0.351167;               // (m/s^2)^2/Hz, cross-track, no state to feed
    double q_heading = 0.0;                  // (rad/s^2)^2/Hz, white-noise accel on w
    double scale_factor = 0.541056;          // fractional speed error, sigma = |v| h eps
    double heading_scale_factor = 1.7783;    // fractional yaw-rate error, sigma = |w| h eps
    double heading_random_walk = 0.0703417;  // rad^2/s
    double delay_jitter_s = 0.0;             // s, along-track sigma = |v| * jitter, flat in h
};

/** Deadzone removal and rescale, per sign: u_eff = sign(u) * max(|u| - dz, 0) / (1 - dz).
 * Mirrors effective_command in plant.py, including the 0.95 deadzone clip. */
double plant_effective_command(double u, double dz_pos, double dz_neg);

/** Target body speed and yaw rate for a held command, coupling and drift included. Mirrors
 * steady_state in plant.py; the steer-brake and droop factors clamp at zero, which is
 * load-bearing at the fitted c_sb of 2.70 (the factor crosses zero at |u_ang_eff| 0.37). */
void plant_steady_state(double u_lin, double u_ang, const JigPlantParams &params, double &v_target,
                        double &w_target);

VelocityCommand plant_stick_to_body_velocity(const StickCommand &stick,
                                             const JigPlantParams &params);

}  // namespace auto_battlebot
