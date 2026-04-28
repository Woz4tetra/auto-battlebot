#pragma once

#include <spdlog/spdlog.h>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "navigation/config.hpp"

namespace auto_battlebot {

class FixedVelocityNavigation : public NavigationInterface {
   public:
    explicit FixedVelocityNavigation(const FixedVelocityNavigationConfiguration &config)
        : linear_x_(config.linear_x),
          linear_y_(config.linear_y),
          angular_z_(config.angular_z),
          logger_(DiagnosticsLogger::get_logger("fixed_velocity_navigation")) {}

    bool initialize() override {
        spdlog::info(
            "FixedVelocityNavigation initialized with: linear_x={}, linear_y={}, angular_z={}",
            linear_x_, linear_y_, angular_z_);
        logger_->info(
            {{"linear_x", linear_x_}, {"linear_y", linear_y_}, {"angular_z", angular_z_}});
        return true;
    }

    VelocityCommand update(RobotDescriptionsStamped robots,
                           [[maybe_unused]] FieldDescription field,
                           [[maybe_unused]] const TargetSelection &target) override {
        logger_->debug(
            "command",
            {{"linear_x", linear_x_}, {"linear_y", linear_y_}, {"angular_z", angular_z_}});
        VelocityCommand cmd{linear_x_, linear_y_, angular_z_};
        last_visualization_ = NavigationVisualization{};
        last_visualization_.header = robots.header;
        last_visualization_.command = cmd;
        last_visualization_.robots = std::move(robots);
        return cmd;
    }

    const NavigationVisualization &get_last_visualization() const override {
        return last_visualization_;
    }

   private:
    double linear_x_;
    double linear_y_;
    double angular_z_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
