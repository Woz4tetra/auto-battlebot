#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "data_structures.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "keypoint_filter/config.hpp"

namespace auto_battlebot {

class StaticDetectionGate {
   public:
    struct Stats {
        int groups_in = 0;
        int groups_passed = 0;
        int suppressed = 0;
        int tracked_clusters = 0;
        int static_clusters = 0;
    };

    explicit StaticDetectionGate(const StaticDetectionGateConfiguration &config);

    ModelResultStamped filter(const ModelResultStamped &model_results,
                              const CameraInfo &camera_info, const FieldDescription &field,
                              double stamp);

    void reset();

    const Stats &last_stats() const { return stats_; }

   private:
    struct Cluster {
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        double first_seen = 0.0;
        double last_seen = 0.0;
        int observations = 0;
        double mean_deviation = 0.0;

        void observe(const Eigen::Vector2d &position,
                     const StaticDetectionGateConfiguration &config, double stamp);
        bool is_static(const StaticDetectionGateConfiguration &config, double now) const;
    };

    StaticDetectionGateConfiguration config_;
    std::vector<Cluster> clusters_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    Stats stats_;
};

}  // namespace auto_battlebot
