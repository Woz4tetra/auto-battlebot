#include "plant/jig_plant_model.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace auto_battlebot {
namespace {

constexpr double kMaxArcSubstepS = 0.002;
constexpr double kMinArcYawRateRadPerS = 1e-6;

double select_lag_tau(double target, double current, double tau_accel, double tau_decel) {
    return std::abs(target) > std::abs(current) ? tau_accel : tau_decel;
}

double lag_alpha(double dt, double tau) { return tau > 1e-9 ? std::exp(-dt / tau) : 0.0; }

double wrap_angle(double angle) {
    double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
    }
    return wrapped - M_PI;
}

bool stamped_oldest_first(std::span<const TimedCommand> commands) {
    return std::is_sorted(
        commands.begin(), commands.end(),
        [](const TimedCommand &a, const TimedCommand &b) { return a.stamp < b.stamp; });
}

void newest_command_at_or_before(std::span<const TimedCommand> commands, double stamp,
                                 double &u_lin, double &u_ang) {
    assert(stamped_oldest_first(commands));
    u_lin = 0.0;
    u_ang = 0.0;
    for (auto it = commands.rbegin(); it != commands.rend(); ++it) {
        if (it->stamp <= stamp) {
            u_lin = it->command.forward;
            u_ang = it->command.yaw;
            return;
        }
    }
}

}  // namespace

JigPlantModel::JigPlantModel(const JigPlantParams &params, const JigPlantNoiseParams &noise)
    : params_(params), noise_(noise) {}

void JigPlantModel::reset() {}

PlantState JigPlantModel::propagate_unwrapped(const PlantState &state,
                                              std::span<const TimedCommand> commands, double t0,
                                              double t1) const {
    const double span_s = t1 - t0;
    if (span_s <= 0.0) {
        return state;
    }
    const int substeps = std::max(1, static_cast<int>(std::lround(span_s / kMaxArcSubstepS)));
    const double sub_dt = span_s / static_cast<double>(substeps);

    PlantState out = state;
    for (int step = 0; step < substeps; ++step) {
        const double now = t0 + static_cast<double>(step) * sub_dt;
        double u_lin = 0.0;
        double u_ang = 0.0;
        newest_command_at_or_before(commands, now - params_.delay_s, u_lin, u_ang);
        double v_target = 0.0;
        double w_target = 0.0;
        plant_steady_state(u_lin, u_ang, params_, v_target, w_target);

        const double theta_next = out.theta + out.w * sub_dt;
        double dx = 0.0;
        double dy = 0.0;
        if (std::abs(out.w) > kMinArcYawRateRadPerS) {
            const double radius = out.v / out.w;
            dx = radius * (std::sin(theta_next) - std::sin(out.theta));
            dy = -radius * (std::cos(theta_next) - std::cos(out.theta));
        } else {
            dx = out.v * sub_dt * std::cos(out.theta);
            dy = out.v * sub_dt * std::sin(out.theta);
        }

        const double tau_lin =
            select_lag_tau(v_target, out.v, params_.tau_lin_a, params_.tau_lin_d);
        const double tau_ang =
            select_lag_tau(w_target, out.w, params_.tau_ang_a, params_.tau_ang_d);
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
    constexpr double kEps = 1e-6;
    for (int col = 0; col < 5; ++col) {
        PlantState plus = state;
        PlantState minus = state;
        double *plus_fields[] = {&plus.x, &plus.y, &plus.theta, &plus.v, &plus.w};
        double *minus_fields[] = {&minus.x, &minus.y, &minus.theta, &minus.v, &minus.w};
        *plus_fields[col] += kEps;
        *minus_fields[col] -= kEps;
        const double realized_step = *plus_fields[col] - *minus_fields[col];
        const PlantState f_plus = propagate_unwrapped(plus, commands, t0, t1);
        const PlantState f_minus = propagate_unwrapped(minus, commands, t0, t1);
        jacobian(0, col) = (f_plus.x - f_minus.x) / realized_step;
        jacobian(1, col) = (f_plus.y - f_minus.y) / realized_step;
        jacobian(2, col) = (f_plus.theta - f_minus.theta) / realized_step;
        jacobian(3, col) = (f_plus.v - f_minus.v) / realized_step;
        jacobian(4, col) = (f_plus.w - f_minus.w) / realized_step;
    }

    PlantState out = propagate_unwrapped(state, commands, t0, t1);
    out.theta = wrap_angle(out.theta);
    return out;
}

Eigen::Matrix<double, 5, 5> JigPlantModel::process_noise(
    const PlantState &state, [[maybe_unused]] std::span<const TimedCommand> commands, double dt) {
    constexpr double kDesignHorizonS = 0.4;
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
