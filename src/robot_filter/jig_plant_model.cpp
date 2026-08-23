#include "robot_filter/jig_plant_model.hpp"

#include <algorithm>
#include <cmath>

namespace auto_battlebot {
namespace {

// Mirrors SUBSTEP_S and STRAIGHT_W in auto_battlebot/plant.py. The substep bounds the
// held-velocity arc error; the yaw floor keeps the arc radius division away from 0/0.
constexpr double kSubstepS = 0.002;
constexpr double kStraightW = 1e-6;

double sign_of(double value) { return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0); }

/** Exact first-order discretization over dt. tau = 0 means no lag. */
double lag_alpha(double dt, double tau) { return tau > 1e-9 ? std::exp(-dt / tau) : 0.0; }

/** Wrap to [-pi, pi), matching wrap_angle in plant.py (numpy mod semantics). */
double wrap_angle(double angle) {
    double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
    }
    return wrapped - M_PI;
}

/** The command in effect at `stamp`: newest entry at or before it, zero before the first.
 * Mirrors Plant.command_at in plant.py. The span is stamped nondecreasing, oldest first. */
void command_at(std::span<const TimedCommand> commands, double stamp, double &u_lin,
                double &u_ang) {
    u_lin = 0.0;
    u_ang = 0.0;
    for (auto it = commands.rbegin(); it != commands.rend(); ++it) {
        if (it->stamp <= stamp) {
            u_lin = it->command.linear_x;
            u_ang = it->command.angular_z;
            return;
        }
    }
}

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

JigPlantModel::JigPlantModel(const JigPlantParams &params, const JigPlantNoiseParams &noise)
    : params_(params), noise_(noise) {}

void JigPlantModel::reset() {
    // Stateless besides the parameters: the state lives in the caller's EKF.
}

PlantState JigPlantModel::propagate_unwrapped(const PlantState &state,
                                              std::span<const TimedCommand> commands, double t0,
                                              double t1) const {
    const double span_s = t1 - t0;
    if (span_s <= 0.0) {
        return state;
    }
    const int substeps = std::max(1, static_cast<int>(std::lround(span_s / kSubstepS)));
    const double sub_dt = span_s / static_cast<double>(substeps);

    PlantState out = state;
    for (int step = 0; step < substeps; ++step) {
        // Re-read the delayed command every substep, so a command edge landing inside a long
        // propagate span takes effect at its own time rather than at the span boundary.
        const double now = t0 + static_cast<double>(step) * sub_dt;
        double u_lin = 0.0;
        double u_ang = 0.0;
        command_at(commands, now - params_.delay_s, u_lin, u_ang);
        double v_target = 0.0;
        double w_target = 0.0;
        plant_steady_state(u_lin, u_ang, params_, v_target, w_target);

        // Pose along the arc with the start-of-substep velocity, then the velocity update,
        // the ordering integrate_step in plant.py defines.
        const double theta_next = out.theta + out.w * sub_dt;
        double dx = 0.0;
        double dy = 0.0;
        if (std::abs(out.w) > kStraightW) {
            const double radius = out.v / out.w;
            dx = radius * (std::sin(theta_next) - std::sin(out.theta));
            dy = -radius * (std::cos(theta_next) - std::cos(out.theta));
        } else {
            dx = out.v * sub_dt * std::cos(out.theta);
            dy = out.v * sub_dt * std::sin(out.theta);
        }

        // Accel or decel per channel by whether the target is further from zero than the
        // current speed: friction first, then torque, when braking into a reversal.
        const double tau_lin =
            std::abs(v_target) > std::abs(out.v) ? params_.tau_lin_a : params_.tau_lin_d;
        const double tau_ang =
            std::abs(w_target) > std::abs(out.w) ? params_.tau_ang_a : params_.tau_ang_d;
        const double a_lin = lag_alpha(sub_dt, tau_lin);
        const double a_ang = lag_alpha(sub_dt, tau_ang);

        out.x += dx;
        out.y += dy;
        out.theta = theta_next;
        out.v = a_lin * out.v + (1.0 - a_lin) * v_target;
        out.w = a_ang * out.w + (1.0 - a_ang) * w_target;
    }
    return out;
}

PlantState JigPlantModel::propagate(const PlantState &state, std::span<const TimedCommand> commands,
                                    double t0, double t1, Eigen::Matrix<double, 5, 5> &jacobian) {
    // Central differences. The model is piecewise, so at a branch boundary (tau selection,
    // coupling clamp) this returns the two-sided average slope, which is what the covariance
    // propagation can best do with a kink anyway.
    constexpr double kEps = 1e-6;
    for (int col = 0; col < 5; ++col) {
        PlantState plus = state;
        PlantState minus = state;
        double *plus_fields[] = {&plus.x, &plus.y, &plus.theta, &plus.v, &plus.w};
        double *minus_fields[] = {&minus.x, &minus.y, &minus.theta, &minus.v, &minus.w};
        *plus_fields[col] += kEps;
        *minus_fields[col] -= kEps;
        // Divide by the perturbation as actually realized in doubles, not the nominal
        // 2*eps: x + eps rounds, and for |x| near 1 the difference is order 1e-10 of slope,
        // which would otherwise be the Jacobian's noise floor.
        const double step = *plus_fields[col] - *minus_fields[col];
        const PlantState f_plus = propagate_unwrapped(plus, commands, t0, t1);
        const PlantState f_minus = propagate_unwrapped(minus, commands, t0, t1);
        jacobian(0, col) = (f_plus.x - f_minus.x) / step;
        jacobian(1, col) = (f_plus.y - f_minus.y) / step;
        // Unwrapped on purpose: a wrapped difference near +/-pi would put a 2*pi jump into
        // the derivative.
        jacobian(2, col) = (f_plus.theta - f_minus.theta) / step;
        jacobian(3, col) = (f_plus.v - f_minus.v) / step;
        jacobian(4, col) = (f_plus.w - f_minus.w) / step;
    }

    PlantState out = propagate_unwrapped(state, commands, t0, t1);
    out.theta = wrap_angle(out.theta);
    return out;
}

Eigen::Matrix<double, 5, 5> JigPlantModel::process_noise(
    const PlantState &state, [[maybe_unused]] std::span<const TimedCommand> commands, double dt) {
    // Growth-law mechanisms from fit_process_noise.py, mapped per predict step. The
    // white-noise acceleration PSDs use the standard discrete block (position picks up the
    // h^3/3 growth through the Jacobian as the EKF accumulates F P F^T + Q). The
    // scale-factor and delay-jitter mechanisms grow h^2 and h^0, which additive per-step Q
    // cannot reproduce, so they are injected at a rate matched at the 400 ms design horizon:
    // exact at a full coast, conservative below it. q = 3c assumes undamped velocity-error
    // integration; the plant's lag damps it, so the h^3 terms come out somewhat narrow over
    // long coasts. Cross-track has no velocity state, so its whole budget is
    // horizon-matched, and heading noise also reaches cross-track through the Jacobian: a
    // known double count in the wide direction.
    constexpr double kDesignHorizonS = 0.4;  // the max-coast timeout
    const double sf_v = noise_.scale_factor * state.v;
    const double sf_w = noise_.heading_scale_factor * state.w;
    const double jitter = noise_.delay_jitter_s * state.v;
    const double p_along = noise_.q_along * dt * dt / 3.0 + sf_v * sf_v * kDesignHorizonS +
                           (kDesignHorizonS > 0.0 ? jitter * jitter / kDesignHorizonS : 0.0);
    const double p_cross = noise_.q_cross * kDesignHorizonS * kDesignHorizonS / 3.0;
    const double p_theta = noise_.heading_random_walk + sf_w * sf_w * kDesignHorizonS +
                           noise_.q_heading * dt * dt / 3.0;

    Eigen::Matrix<double, 5, 5> noise = Eigen::Matrix<double, 5, 5>::Zero();
    const double c = std::cos(state.theta);
    const double s = std::sin(state.theta);
    noise(0, 0) = (c * c * p_along + s * s * p_cross) * dt;
    noise(1, 1) = (s * s * p_along + c * c * p_cross) * dt;
    noise(0, 1) = noise(1, 0) = c * s * (p_along - p_cross) * dt;
    noise(0, 3) = noise(3, 0) = c * noise_.q_along * dt * dt / 2.0;
    noise(1, 3) = noise(3, 1) = s * noise_.q_along * dt * dt / 2.0;
    noise(2, 2) = p_theta * dt;
    noise(2, 4) = noise(4, 2) = noise_.q_heading * dt * dt / 2.0;
    noise(3, 3) = noise_.q_along * dt;
    noise(4, 4) = noise_.q_heading * dt;
    return noise;
}

}  // namespace auto_battlebot
