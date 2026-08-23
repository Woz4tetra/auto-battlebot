#pragma once

#include <Eigen/Dense>
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "robot_filter/command_ring_buffer.hpp"
#include "robot_filter/ekf_math.hpp"
#include "robot_filter/jig_plant_model.hpp"
#include "robot_filter/motion_estimator_config.hpp"
#include "robot_filter/motion_estimator_interface.hpp"

namespace auto_battlebot {

/**
 * Kalman-filter propagation behind the motion estimator seam.
 *
 * Non-OURS tracks run a 4-state linear Kalman filter [px, py, vx, vy] with a constant-velocity
 * model and continuous white-noise acceleration, position-only measurements, chi-square
 * innovation gating, and reinitialization after repeated rejects. Unmeasured tracks predict
 * forward with growing covariance up to max_coast_s of track age, then hold position.
 * Measurements older than the track state (perception latency) are handled by rewinding to a
 * stored snapshot, correcting at the measurement's own stamp, and re-propagating.
 *
 * The our-robot track has two modes. "dead_reckoning" applies the last commanded velocity
 * directly, matching DeadReckoningMotionEstimator for that track. "ekf" runs a 5-state EKF
 * [x, y, theta, v, w] propagated through JigPlantModel with the issued-command history read
 * at t - delay, corrected by keypoint poses (position + heading) or blob centroids
 * (position only). A heading innovation past pi/2 falls back to the position rows: the
 * front/back keypoint converter can flip, and folding a flipped heading in would spin the
 * whole state.
 *
 * coast() renders every track advanced to the control-loop clock, so the emitted state moves
 * between perception frames instead of freezing at the last correct().
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

   private:
    /** 64 snapshots cover well over a second of control ticks, past any perception latency. */
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

    struct Snapshot {
        double stamp = 0.0;
        ekf::Vec<4> state = ekf::Vec<4>::Zero();
        ekf::Mat<4, 4> covariance = ekf::Mat<4, 4>::Zero();
    };

    /** One non-OURS robot: state [px, py, vx, vy] in the field frame. */
    struct OpponentTrack {
        ekf::Vec<4> state = ekf::Vec<4>::Zero();
        ekf::Mat<4, 4> covariance = ekf::Mat<4, 4>::Zero();
        /** Time the state refers to. */
        double stamp = 0.0;
        double last_measured_stamp = 0.0;
        int consecutive_rejects = 0;
        /** is_stale emitted for this track by the most recent update(); coast() reuses it so
         * the flag keeps its measured-at-last-perception-frame meaning between frames. */
        bool output_stale = false;
        /** Non-kinematic fields (label, group, size, rotation, z) of the latest accepted
         * measurement, used as the output template. */
        RobotDescription description;
        std::array<Snapshot, kSnapshotCapacity> snapshots{};
        size_t snapshot_start = 0;
        size_t snapshot_count = 0;
    };

    /** Our robot, dead-reckoned exactly like RobotTemporalMotionFilter. */
    struct OurTrack {
        RobotDescription description;
        /** Time of the last update() that touched this track; dead-reckoning dt spans from
         * here. */
        double stamp = 0.0;
        double last_measured_stamp = 0.0;
        bool output_stale = false;
    };

    /** Our robot in "ekf" mode: 5-state [x, y, theta, v, w] propagated by the plant model. */
    struct OurEkfTrack {
        PlantState state;
        ekf::Mat<5, 5> covariance = ekf::Mat<5, 5>::Zero();
        double stamp = 0.0;
        double last_measured_stamp = 0.0;
        int consecutive_rejects = 0;
        bool output_stale = false;
        /** Non-kinematic fields of the latest accepted measurement, the output template. */
        RobotDescription description;
    };

    /** Propagates the track to target_stamp (no-op when not ahead) and stores a snapshot. */
    void advance_opponent(OpponentTrack &track, double target_stamp);
    /** Propagates the our-robot EKF track through the plant model to target_stamp. Pose
     * integration stops once the unmeasured age crosses max_coast_s; covariance keeps
     * growing over the whole step either way. */
    void advance_our_ekf(OurEkfTrack &track, double target_stamp);
    /** Resets the track to this measurement with a fresh large covariance. */
    void init_our_ekf(OurEkfTrack &track, const RobotDescription &measurement, double stamp,
                      double position_variance);
    /** Builds the output description from the track's current state. */
    RobotDescription render_our_ekf(const OurEkfTrack &track, const PlantState &state,
                                    bool stale) const;
    void push_snapshot(OpponentTrack &track);
    /**
     * Restores the newest snapshot at or before target_stamp and drops the ones after it.
     * Returns false (track untouched) when no snapshot is old enough, in which case the
     * measurement is folded in at the track's current state instead.
     */
    bool rewind_opponent(OpponentTrack &track, double target_stamp);
    /** Resets the track to this measurement with a fresh large covariance. */
    void init_opponent(OpponentTrack &track, const RobotDescription &measurement, double stamp,
                       double position_variance);
    /** Isotropic position noise: base sigma by measurement type (blob vs keypoint) plus the
     * configured growth per meter of camera range. */
    ekf::Mat<2, 2> measurement_noise_for(const RobotDescription &measurement,
                                         const MotionEstimatorContext &context) const;
    /** Builds the output description from the track's current state. */
    RobotDescription render_opponent(const OpponentTrack &track, const FieldDescription &field,
                                     bool stale) const;

    KalmanMotionEstimatorConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    std::map<FrameId, OpponentTrack> opponent_tracks_;
    std::map<FrameId, OurTrack> our_tracks_;
    std::map<FrameId, OurEkfTrack> our_ekf_tracks_;

    /** Present only in "ekf" mode; the concrete type so coast() can propagate a copy without
     * paying for the finite-difference Jacobian. */
    std::unique_ptr<JigPlantModel> plant_model_;

    /** Latest control input, for dead-reckoning our robot and coast extrapolation. */
    CommandFeedback command_feedback_;
    /** Issued-command history for the future plant-model prediction (read at t - delay). */
    CommandRingBuffer command_history_;
    /** Last field seen by update(), for clipping coasted positions to bounds. */
    FieldDescription last_field_;
    bool has_field_ = false;
    double last_field_margin_ = 0.0;
};

}  // namespace auto_battlebot
