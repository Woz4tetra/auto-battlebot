#pragma once

#include <cmath>
#include <functional>

#include "enums/behavior_mode.hpp"
#include "navigation/config.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot::nav_golden {

inline JigPlantParams mr_stabs_mk2_plant() {
    JigPlantParams plant;
    plant.dz_lin_fwd = 0.00938353;
    plant.k_fwd = 5.7302;
    plant.k_rev = 3.75736;
    plant.k_ang = 38.9057;
    plant.tau_lin_a = 0.182874;
    plant.tau_lin_d = 0.0684365;
    plant.tau_ang_a = 0.0588077;
    plant.tau_ang_d = 0.0610599;
    plant.c_sb = 1.36475;
    plant.c_ad = 0.195197;
    return plant;
}

inline MotionProfileNavigationConfiguration scenario_config() {
    MotionProfileNavigationConfiguration config;
    config.plant = mr_stabs_mk2_plant();
    config.wall_reverse_distance_m = 0.25;
    config.wall_heading_threshold_rad = 1.047;
    config.wall_reverse_min_speed = 0.25;
    config.max_linear_command = 0.8;
    config.compensate_angular_droop = true;
    return config;
}

struct ScenarioTick {
    double time_s;
    RobotDescriptionsStamped robots;
    FieldDescription field;
    TargetSelection target;
};

constexpr int kScenarioTicks = 60;

inline ScenarioTick scenario_tick(int i) {
    ScenarioTick tick;
    tick.time_s = 1.0 + 0.02 * i;
    tick.field.size.size.x = 6.0;
    tick.field.size.size.y = 6.0;
    FieldHazard hazard;
    hazard.center.x = 0.6;
    hazard.center.y = 0.25;
    hazard.hard_radius = 0.12;
    hazard.inflated_radius_m = 0.18;
    hazard.object_radius = 0.12;
    tick.field.hazards.push_back(hazard);

    const bool near_wall = i >= 45;
    const double x = near_wall ? 2.82 : -1.0 + 0.04 * i;
    const double y = near_wall ? 0.0 : 0.1 * std::sin(0.3 * i);
    const double yaw = near_wall ? 0.05 * (i - 45) : 0.6 * std::sin(0.25 * i);
    const double vx = 2.0 * std::sin(0.2 * i);
    if (i % 13 != 7) {
        RobotDescription robot;
        robot.label = Label::MR_STABS_MK2;
        robot.frame_id = FrameId::OUR_ROBOT_1;
        robot.group = Group::OURS;
        robot.pose.position.x = x;
        robot.pose.position.y = y;
        robot.pose.rotation = euler_to_quaternion(0.0, 0.0, yaw);
        robot.velocity = Velocity2D{vx * std::cos(yaw), vx * std::sin(yaw), 0.0};
        tick.robots.descriptions.push_back(robot);
    }
    tick.target.pose.x = near_wall ? 2.95 : 1.5;
    tick.target.pose.y = near_wall ? 0.1 : 0.6 * std::cos(0.3 * i);
    tick.target.label = Label::OPPONENT;
    tick.target.mode = (i >= 30 && i < 40) ? BehaviorMode::RUN_AWAY : BehaviorMode::ATTACK;
    return tick;
}

}  // namespace auto_battlebot::nav_golden
