#pragma once

#include "plant/plant_golden_data.hpp"
#include "plant/plant_params.hpp"

namespace auto_battlebot {

/**
 * The Mrs Buff Mk3 jig fit as a JigPlantParams, for tests that need a plant that behaves like
 * the real drivetrain. Sourced from the generated golden constants rather than hand-copied, so
 * a refit moves the tests with it.
 *
 * Production code gets this from the [plant] config table; nothing in C++ defaults to a fit
 * any more, which is why a test that default-constructs a config gets a zero plant and has to
 * ask for this one explicitly.
 */
inline JigPlantParams mrs_buff_mk3_plant() {
    JigPlantParams params;
    params.dz_lin_fwd = plant_golden::kParam_dz_lin_fwd;
    params.dz_lin_rev = plant_golden::kParam_dz_lin_rev;
    params.dz_ang_l = plant_golden::kParam_dz_ang_l;
    params.dz_ang_r = plant_golden::kParam_dz_ang_r;
    params.k_fwd = plant_golden::kParam_k_fwd;
    params.k_rev = plant_golden::kParam_k_rev;
    params.k_ang = plant_golden::kParam_k_ang;
    params.tau_lin_a = plant_golden::kParam_tau_lin_a;
    params.tau_lin_d = plant_golden::kParam_tau_lin_d;
    params.tau_ang_a = plant_golden::kParam_tau_ang_a;
    params.tau_ang_d = plant_golden::kParam_tau_ang_d;
    params.delay_s = plant_golden::kParam_delay_s;
    params.c_sb = plant_golden::kParam_c_sb;
    params.c_ad = plant_golden::kParam_c_ad;
    params.c_drift = plant_golden::kParam_c_drift;
    params.c_drift_bias = plant_golden::kParam_c_drift_bias;
    return params;
}

}  // namespace auto_battlebot
