#pragma once

#include <Eigen/Dense>
#include <array>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "plant/jig_plant_model.hpp"
#include "robot_filter/command_ring_buffer.hpp"
#include "robot_filter/ekf_math.hpp"
#include "robot_filter/motion_estimator_config.hpp"
#include "robot_filter/motion_estimator_interface.hpp"

namespace auto_battlebot {

class KalmanMotionEstimator : public MotionEstimatorInterface {
   public:
    explicit KalmanMotionEstimator(const KalmanMotionEstimatorConfiguration &config);

    void reset() override;

    void predict(double now, const CommandFeedback &command_feedback) override;

    std::vector<RobotDescription> update(std::vector<RobotDescription> measurements,
                                         double timestamp, FrameIdAssigner &frame_id_assigner,
                                         const FieldDescription &field,
                                         const MotionEstimatorContext &context) override;

    std::optional<std::vector<RobotDescription>> coast(double now) override;

    double render_lead_s() const override { return config_.plant->delay_s; }

    struct OpponentFilterState {
        ekf::Vec<6> state;
        ekf::Mat<6, 6> covariance;
        double stamp = 0.0;
        bool heading_initialized = false;
    };
    std::optional<OpponentFilterState> opponent_filter_state(FrameId frame_id) const;

   private:
    static constexpr size_t kSnapshotCapacity = 64;
    static constexpr double kMaxIntegrationStepS = 1.0;

    static constexpr int kOpponentStates = 6;
    using OpponentVec = ekf::Vec<kOpponentStates>;
    using OpponentMat = ekf::Mat<kOpponentStates, kOpponentStates>;

    struct Snapshot {
        double stamp = 0.0;
        OpponentVec state = OpponentVec::Zero();
        OpponentMat covariance = OpponentMat::Zero();
        bool heading_initialized = false;
    };

    struct OpponentTrack {
        OpponentVec state = OpponentVec::Zero();
        OpponentMat covariance = OpponentMat::Zero();
        bool heading_initialized = false;
        int consecutive_heading_flips = 0;
        double state_stamp = 0.0;
        double last_measured_stamp = 0.0;
        int consecutive_rejects = 0;
        bool output_stale = false;
        RobotDescription output_template;
        std::array<Snapshot, kSnapshotCapacity> snapshots{};
        size_t snapshot_start = 0;
        size_t snapshot_count = 0;
    };

    struct HeldTrack {
        RobotDescription output_template;
        double last_update_stamp = 0.0;
        double last_measured_stamp = 0.0;
        bool output_stale = false;
    };

    struct OurEkfTrack {
        PlantState state;
        ekf::Mat<5, 5> covariance = ekf::Mat<5, 5>::Zero();
        double stamp = 0.0;
        double last_measured_stamp = 0.0;
        int consecutive_rejects = 0;
        bool output_stale = false;
        RobotDescription output_template;
        double speed_ref_x = 0.0;
        double speed_ref_y = 0.0;
        double speed_ref_yaw = 0.0;
        double speed_ref_stamp = 0.0;
        bool has_speed_ref = false;
    };

    void advance_opponent(OpponentTrack &track, double target_stamp);
    void advance_our_ekf(OurEkfTrack &track, double target_stamp);
    void init_our_ekf(OurEkfTrack &track, const RobotDescription &measurement, double stamp,
                      double position_variance);
    RobotDescription render_our_ekf(const OurEkfTrack &track, const PlantState &state,
                                    bool stale) const;
    void push_snapshot(OpponentTrack &track);
    bool rewind_opponent(OpponentTrack &track, double target_stamp);
    void init_opponent(OpponentTrack &track, const RobotDescription &measurement, double stamp,
                       double position_variance);
    void init_opponent_heading(OpponentTrack &track, double yaw) const;
    ekf::Mat<2, 2> measurement_noise_for(const RobotDescription &measurement,
                                         const MotionEstimatorContext &context) const;
    RobotDescription render_opponent(const OpponentTrack &track, const FieldDescription &field,
                                     bool stale) const;

    KalmanMotionEstimatorConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    std::map<FrameId, OpponentTrack> opponent_tracks_;
    std::map<FrameId, HeldTrack> held_opponent_tracks_;
    std::map<FrameId, HeldTrack> our_tracks_;
    std::map<FrameId, OurEkfTrack> our_ekf_tracks_;

    std::unique_ptr<JigPlantModel> plant_model_;

    JigPlantParams our_dr_plant_;

    CommandFeedback command_feedback_;
    CommandRingBuffer command_history_;
    FieldDescription last_field_;
    bool has_field_ = false;
    double last_field_margin_ = 0.0;
    double last_clock_gap_warning_ = -std::numeric_limits<double>::infinity();
    double min_coast_span_s_ = 0.0;
};

}  // namespace auto_battlebot
