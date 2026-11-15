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

/**
 * Kalman-filter propagation behind the motion estimator seam.
 *
 * Non-OURS tracks run a 6-state linear Kalman filter [px, py, vx, vy, theta, omega]: a
 * constant-velocity translation block and an independent constant-yaw-rate heading block, each
 * driven by its own continuous white-noise acceleration. Keypoint measurements correct position
 * and heading, blob centroids position only. Position is chi-square gated, with
 * reinitialization after repeated rejects; a heading more than pi/2 off the track is a front/back
 * flip and corrects nothing, and a run of flips re-initializes the heading block. Unmeasured
 * tracks predict forward with growing covariance up to max_coast_s of track age, then hold
 * position and heading. Measurements older than the track state (perception latency) are handled
 * by rewinding to a stored snapshot, correcting at the measurement's own stamp, and
 * re-propagating. The opponent coast horizon is opponent_max_coast_s. The Python mirror in
 * auto_battlebot/perception/opponent_filter.py must change with this arm;
 * tests/test_kalman_motion_estimator.cpp holds the two to a golden fixture.
 *
 * The our-robot track has two modes. DEAD_RECKONING applies the last commanded velocity
 * directly, matching DeadReckoningMotionEstimator for that track. EKF runs a 5-state EKF
 * [x, y, theta, v, w] propagated through JigPlantModel with the issued-command history
 * (normalized stick space, from CommandFeedback::stick_commands) read at t - delay, corrected by
 * keypoint poses (position + heading) or blob centroids (position only). A heading innovation past
 * pi/2 falls back to the position rows: the front/back keypoint converter can flip, and folding a
 * flipped heading in would spin the whole state.
 *
 * coast() renders every track advanced to the control-loop clock plus render_lead_s(), so the
 * emitted state moves between perception frames instead of freezing at the last correct(), and
 * describes where each robot will be when the command computed from it reaches the wheels.
 */
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

    /**
     * The plant's transport delay. A command computed from this render reaches the wheels
     * `delay_s` later, so the render has to describe the field at that moment. Config validation
     * guarantees the plant table is present, so this is always the fitted value.
     */
    double render_lead_s() const override { return config_.plant->delay_s; }

    struct OpponentFilterState {
        ekf::Vec<6> state;
        ekf::Mat<6, 6> covariance;
        double stamp = 0.0;
        bool heading_initialized = false;
    };
    std::optional<OpponentFilterState> opponent_filter_state(FrameId frame_id) const;

   private:
    /**
     * At the 250 Hz control rate this is roughly 240 ms of rewind history: coast() commits an
     * opponent snapshot every tick, so a 30 Hz frame burns about nine slots. That covers the
     * measured 63 ms perception latency with room to spare, and num_rewind_missed reports when
     * it does not.
     */
    static constexpr size_t kSnapshotCapacity = 64;
    /** A single propagation step longer than this freezes position: a clock gap that large is
     * not motion to integrate. Covariance still grows, capped at this step. */
    static constexpr double kMaxIntegrationStepS = 1.0;
    /**
     * coast() flags a track is_stale once it has gone unmeasured this long. Three 30 Hz frame
     * periods: between-frame coasting in normal operation stays "fresh", a real perception
     * dropout reads as stale even though correct() never ran to say so.
     */
    static constexpr double kCoastStaleAgeS = 0.1;
    /** Throttle on the clock-gap warning: it fires every tick once tripped. */
    static constexpr double kClockGapWarnPeriodS = 5.0;

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
        /** Time the state refers to. */
        double stamp = 0.0;
        double last_measured_stamp = 0.0;
        int consecutive_rejects = 0;
        bool output_stale = false;
        /** Non-kinematic fields (label, group, size, rotation, z) of the latest accepted
         * measurement, used as the output template. */
        RobotDescription description;
        std::array<Snapshot, kSnapshotCapacity> snapshots{};
        size_t snapshot_start = 0;
        size_t snapshot_count = 0;
    };

    struct HeldTrack {
        RobotDescription description;
        /** Time of the last update() that touched this track; dead-reckoning dt spans from
         * here. */
        double stamp = 0.0;
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
        /** Non-kinematic fields of the latest accepted measurement, the output template. */
        RobotDescription description;
        double speed_ref_x = 0.0;
        double speed_ref_y = 0.0;
        double speed_ref_yaw = 0.0;
        double speed_ref_stamp = 0.0;
        bool has_speed_ref = false;
    };

    void advance_opponent(OpponentTrack &track, double target_stamp);
    /** Propagates the our-robot EKF track through the plant model to target_stamp. Pose
     * integration stops once the unmeasured age crosses max_coast_s; covariance keeps
     * growing over the whole step either way. */
    void advance_our_ekf(OurEkfTrack &track, double target_stamp);
    void init_our_ekf(OurEkfTrack &track, const RobotDescription &measurement, double stamp,
                      double position_variance);
    RobotDescription render_our_ekf(const OurEkfTrack &track, const PlantState &state,
                                    bool stale) const;
    void push_snapshot(OpponentTrack &track);
    /**
     * Restores the newest snapshot at or before target_stamp and drops the ones after it.
     * Returns false (track untouched) when no snapshot is old enough, in which case the
     * measurement is folded in at the track's current state instead.
     */
    bool rewind_opponent(OpponentTrack &track, double target_stamp);
    void init_opponent(OpponentTrack &track, const RobotDescription &measurement, double stamp,
                       double position_variance);
    void init_opponent_heading(OpponentTrack &track, double yaw) const;
    bool measures_heading(const RobotDescription &measurement) const;
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

    /** Present only in EKF mode; the concrete type so coast() can propagate a copy without
     * paying for the finite-difference Jacobian. */
    std::unique_ptr<JigPlantModel> plant_model_;

    /** Fitted gains for the DEAD_RECKONING our-robot arm, which scales stick to body velocity
     * without the plant model. Zero-valued only when a test skipped the plant; config validation
     * requires the table for both modes. */
    JigPlantParams our_dr_plant_;

    CommandFeedback command_feedback_;
    CommandRingBuffer command_history_;
    FieldDescription last_field_;
    bool has_field_ = false;
    double last_field_margin_ = 0.0;
    /** Control-clock time of the last clock-gap warning. -inf so the first one always fires,
     * whatever epoch the clock is on. */
    double last_clock_gap_warning_ = -std::numeric_limits<double>::infinity();
    double min_coast_span_s_ = 0.0;
};

}  // namespace auto_battlebot
