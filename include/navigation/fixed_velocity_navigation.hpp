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

    VelocityCommand update([[maybe_unused]] RobotDescriptionsStamped robots,
                           [[maybe_unused]] FieldDescription field,
                           [[maybe_unused]] const TargetSelection &target) override {
        logger_->debug(
            "command",
            {{"linear_x", linear_x_}, {"linear_y", linear_y_}, {"angular_z", angular_z_}});
        return VelocityCommand{linear_x_, linear_y_, angular_z_};
    }

   private:
    double linear_x_;
    double linear_y_;
    double angular_z_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
};

}  // namespace auto_battlebot
