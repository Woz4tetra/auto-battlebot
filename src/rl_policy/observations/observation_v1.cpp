#include "rl_policy/observations/observation_v1.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "transform_utils.hpp"

namespace auto_battlebot::rl_policy {
namespace {

using namespace obs_v1;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/** obs_spec._cap: torch.nan_to_num(x, nan=1, posinf=1) then clamp to [0, 1]. -inf becomes the
 * lowest finite double first, so it lands on 0. */
double cap(double x) {
    if (std::isnan(x) || x == kInf) return 1.0;
    return std::clamp(x, 0.0, 1.0);
}

/** obs_spec.wrap: torch.remainder(a + pi, 2 pi) - pi. torch.remainder takes the divisor's sign,
 * so a negative input wraps up into [0, 2 pi) the way std::fmod alone would not. */
double wrap(double a) {
    const double divisor = 2.0 * M_PI;
    double r = std::fmod(a + M_PI, divisor);
    if (r < 0.0) r += divisor;
    return r - M_PI;
}

/** torch.minimum: NaN if either side is. */
double nan_min(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return kNaN;
    return std::min(a, b);
}

struct Body {
    double c;
    double s;
    /** A field-frame vector into our body frame (obs_spec build_v1's `body`). */
    void rotate(double vx, double vy, double &bx, double &by) const {
        bx = c * vx + s * vy;
        by = -s * vx + c * vy;
    }
};

/** obs_spec.ray_box: distance from the origin along the ray to the field wall, 0 for an origin
 * on or outside it. */
double ray_box(double ox, double oy, double dx, double dy, double half_x, double half_y) {
    double t = kInf;
    const double origin[2] = {ox, oy};
    const double direction[2] = {dx, dy};
    const double half[2] = {half_x, half_y};
    for (int axis = 0; axis < 2; ++axis) {
        const double d = direction[axis];
        const double sign = d > 0.0 ? 1.0 : -1.0;
        const double ta = std::abs(d) > 1e-12 ? (sign * half[axis] - origin[axis]) / d : kInf;
        t = nan_min(t, ta);
    }
    return std::isnan(t) ? t : std::max(t, 0.0);
}

/** obs_spec.ray_discs for one ray: nearest hit on any disc `use(kind)` accepts, inf when none.
 * An origin inside a disc hits it at 0. */
template <typename Use>
double ray_discs(double ox, double oy, double dx, double dy,
                 std::span<const ObservationHazardV1> hazards, Use use) {
    double nearest = kInf;
    for (const auto &hazard : hazards) {
        if (!hazard.present || !use(hazard.kind)) continue;
        const double mx = ox - hazard.x;
        const double my = oy - hazard.y;
        const double b = mx * dx + my * dy;
        const double c = (mx * mx + my * my) - hazard.radius * hazard.radius;
        const double disc = b * b - c;
        const bool inside = c <= 0.0;
        const double t = -b - std::sqrt(std::max(disc, 0.0));
        const bool hit = disc >= 0.0 && t >= 0.0;
        const double dist = inside ? 0.0 : (hit ? t : kInf);
        nearest = nan_min(nearest, dist);
    }
    return nearest;
}

}  // namespace

void build_observation_v1(const ObservationInputsV1 &in, std::span<double, obs_v1::kSize> out) {
    const Body body{std::cos(in.our_yaw), std::sin(in.our_yaw)};
    size_t n = 0;

    // Our robot.
    out[n++] = in.our_speed / kSpeedScale;
    out[n++] = in.our_yaw_rate / kYawRateScale;
    out[n++] = cap(in.our_age_s / kAgeScaleS);
    out[n++] = in.our_stale ? 1.0 : 0.0;
    out[n++] = in.run_away ? 0.0 : 1.0;
    out[n++] = in.run_away ? 1.0 : 0.0;

    // History, newest first: the actions, then the measured speed and yaw rate.
    for (double value : in.hist_action) out[n++] = value;
    for (int k = 0; k < kHistory; ++k) {
        out[n++] = in.hist_speed[k] / kSpeedScale;
        out[n++] = in.hist_yaw_rate[k] / kYawRateScale;
    }

    // Target opponent. Absent, it reads zeros except the sigmas, age and stale flag, which read 1.
    double rx = 0.0;
    double ry = 0.0;
    body.rotate(in.opp_x - in.our_x, in.opp_y - in.our_y, rx, ry);
    const double distance = std::hypot(rx, ry);
    const double bearing = std::atan2(ry, rx);
    const double heading = wrap(in.opp_yaw - in.our_yaw);
    double vx = 0.0;
    double vy = 0.0;
    body.rotate(in.opp_vx, in.opp_vy, vx, vy);
    const double known = in.opp_present ? 1.0 : 0.0;
    out[n++] = known * rx / kPositionScaleM;
    out[n++] = known * ry / kPositionScaleM;
    out[n++] = known * distance / kPositionScaleM;
    out[n++] = known * std::sin(bearing);
    out[n++] = known * std::cos(bearing);
    out[n++] = known * std::sin(heading);
    out[n++] = known * std::cos(heading);
    out[n++] = known * vx / kSpeedScale;
    out[n++] = known * vy / kSpeedScale;
    out[n++] = known * in.opp_yaw_rate / kYawRateScale;
    out[n++] = in.opp_present ? cap(in.opp_position_sigma / kPositionSigmaScaleM) : 1.0;
    out[n++] = in.opp_present ? cap(in.opp_heading_sigma / kHeadingSigmaScaleRad) : 1.0;
    out[n++] = in.opp_present ? cap(in.opp_yaw_rate_sigma / kYawRateScale) : 1.0;
    out[n++] = known * in.opp_half_diagonal / kFootprintScaleM;
    out[n++] = in.opp_present ? cap(in.opp_age_s / kAgeScaleS) : 1.0;
    out[n++] = in.opp_present ? (in.opp_stale ? 1.0 : 0.0) : 1.0;

    // Range rays from heading 0 counter-clockwise. Solid sees walls, wall blocks and tracked
    // robots; hole sees holes.
    const auto solid_kind = [](HazardKind kind) { return kind != HazardKind::HOLE; };
    const auto hole_kind = [](HazardKind kind) { return kind == HazardKind::HOLE; };
    for (int r = 0; r < kNumRays; ++r) {
        const double angle = in.our_yaw + static_cast<double>(r) * (2.0 * M_PI / kNumRays);
        const double dx = std::cos(angle);
        const double dy = std::sin(angle);
        const double walls = ray_box(in.our_x, in.our_y, dx, dy, in.field_half_x, in.field_half_y);
        const double solid =
            nan_min(walls, ray_discs(in.our_x, in.our_y, dx, dy, in.hazards, solid_kind));
        const double hole = ray_discs(in.our_x, in.our_y, dx, dy, in.hazards, hole_kind);
        out[n++] = (std::isnan(solid) ? solid : std::min(solid, kRayRangeM)) / kRayRangeM;
        out[n++] = (std::isnan(hole) ? hole : std::min(hole, kRayRangeM)) / kRayRangeM;
    }

    // The 4 nearest hazards by edge distance, absent ones last. Selection by repeated scan in
    // the stable sort's order, so no index buffer is needed.
    const size_t count = in.hazards.size();
    const auto edge_of = [&](size_t i) {
        const auto &hazard = in.hazards[i];
        if (!hazard.present) return kInf;
        const double dx = hazard.x - in.our_x;
        const double dy = hazard.y - in.our_y;
        return std::sqrt(dx * dx + dy * dy) - hazard.radius;
    };
    // torch.sort(stable=True) order: ascending, NaN last, ties keep input order.
    const auto before = [&](size_t i, size_t j) {
        const double edge_i = edge_of(i);
        const double edge_j = edge_of(j);
        const bool nan_i = std::isnan(edge_i);
        const bool nan_j = std::isnan(edge_j);
        if (nan_i != nan_j) return nan_j;
        if (!nan_i && edge_i != edge_j) return edge_i < edge_j;
        return i < j;
    };

    size_t previous = count;  // none yet
    for (int k = 0; k < kMaxHazards; ++k) {
        size_t best = count;
        for (size_t i = 0; i < count; ++i) {
            if (previous != count && !before(previous, i)) continue;
            if (best == count || before(i, best)) best = i;
        }
        if (best == count) {
            for (int v = 0; v < 9; ++v) out[n++] = 0.0;
            continue;
        }
        previous = best;
        const auto &hazard = in.hazards[best];
        const double ok = hazard.present ? 1.0 : 0.0;
        double hx = 0.0;
        double hy = 0.0;
        body.rotate(hazard.x - in.our_x, hazard.y - in.our_y, hx, hy);
        double hvx = 0.0;
        double hvy = 0.0;
        body.rotate(hazard.vx, hazard.vy, hvx, hvy);
        out[n++] = ok * hx / kPositionScaleM;
        out[n++] = ok * hy / kPositionScaleM;
        out[n++] = ok * hazard.radius / kHazardRadiusScaleM;
        out[n++] = ok * (hazard.kind == HazardKind::HOLE ? 1.0 : 0.0);
        out[n++] = ok * (hazard.kind == HazardKind::WALL_BLOCK ? 1.0 : 0.0);
        out[n++] = ok * (hazard.kind == HazardKind::ROBOT ? 1.0 : 0.0);
        out[n++] = ok * hvx / kSpeedScale;
        out[n++] = ok * hvy / kSpeedScale;
        out[n++] = ok;
    }
}

ObservationInputsV1 observation_inputs_v1(const ObservationContext &context,
                                          std::vector<ObservationHazardV1> &hazard_storage) {
    ObservationInputsV1 in;

    const RobotDescription &our = context.our_robot;
    const Pose2D our_pose = pose_to_pose2d(our.pose);
    in.our_x = our_pose.x;
    in.our_y = our_pose.y;
    in.our_yaw = our_pose.yaw;
    in.our_speed =
        our.velocity.vx * std::cos(our_pose.yaw) + our.velocity.vy * std::sin(our_pose.yaw);
    in.our_yaw_rate = our.velocity.omega;
    in.our_age_s = our.measurement_age_s;
    in.our_stale = our.is_stale;
    in.run_away = context.target.mode == BehaviorMode::RUN_AWAY;

    const size_t steps = std::min<size_t>(kHistory, context.history.capacity());
    for (size_t k = 0; k < steps; ++k) {
        const HistoryEntry &entry = context.history.entry(k);
        in.hist_action[2 * k] = entry.action_linear;
        in.hist_action[2 * k + 1] = entry.action_angular;
        in.hist_speed[k] = entry.measured_speed;
        in.hist_yaw_rate[k] = entry.measured_yaw_rate;
    }

    if (context.target_opponent != nullptr) {
        const RobotDescription &opponent = *context.target_opponent;
        const Pose2D pose = pose_to_pose2d(opponent.pose);
        in.opp_present = true;
        in.opp_x = pose.x;
        in.opp_y = pose.y;
        in.opp_yaw = pose.yaw;
        in.opp_vx = opponent.velocity.vx;
        in.opp_vy = opponent.velocity.vy;
        in.opp_yaw_rate = opponent.velocity.omega;
        in.opp_position_sigma = opponent.uncertainty.position_sigma_m;
        in.opp_heading_sigma = opponent.uncertainty.heading_sigma_rad;
        in.opp_yaw_rate_sigma = opponent.uncertainty.yaw_rate_sigma_rad_s;
        in.opp_half_diagonal = 0.5 * std::hypot(opponent.size.x, opponent.size.y);
        in.opp_age_s = opponent.measurement_age_s;
        in.opp_stale = opponent.is_stale;
    }

    in.field_half_x = 0.5 * context.field.size.size.x;
    in.field_half_y = 0.5 * context.field.size.size.y;

    hazard_storage.clear();
    for (const FieldHazard &hazard : context.field.hazards) {
        hazard_storage.push_back(ObservationHazardV1{hazard.center.x, hazard.center.y,
                                                     hazard.object_radius, hazard.kind,
                                                     hazard.velocity.vx, hazard.velocity.vy, true});
    }
    in.hazards = hazard_storage;
    return in;
}

std::vector<TensorSpec> ObservationBuilderV1::output_specs() const {
    return {TensorSpec{obs_v1::kTensorName, {obs_v1::kSize}}};
}

void ObservationBuilderV1::build(const ObservationContext &context, TensorSet &observation) {
    const ObservationInputsV1 inputs = observation_inputs_v1(context, hazards_);
    build_observation_v1(inputs, values_);
    Tensor *tensor = observation.find(obs_v1::kTensorName);
    if (tensor == nullptr || tensor->data.size() != values_.size()) return;
    for (size_t i = 0; i < values_.size(); ++i) tensor->data[i] = static_cast<float>(values_[i]);
}

}  // namespace auto_battlebot::rl_policy
