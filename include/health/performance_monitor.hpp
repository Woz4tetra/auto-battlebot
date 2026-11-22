#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "data_structures/pose.hpp"
#include "health/link_report.hpp"
#include "health/performance_monitor_config.hpp"

namespace auto_battlebot {

enum class WarningSeverity { WARN, RED };

enum class PerformanceCheck {
    CAMERA_FRAMES_LINK,
    PERCEPTION_LINK,
    FILTER_LINK,
    TRANSMIT_LINK,
    CRSF_TELEMETRY_LINK,
    ESPNOW_LINK,
    IMU_LINK,
    RELAY_PUBLISH_LINK,
    SLEW_BINDING,
    GUARD_BINDING,
    CALIBRATION,
    PHANTOM_TRACK,
    YAW_RATE_DISAGREEMENT,
    THERMAL_TRIP,
    FIRMWARE_PROTOCOL,
    OUR_TRACKING,
    OPPONENT_TRACKING,
};

std::string check_name(PerformanceCheck check);
PerformanceCheck link_check(MonitorLink link);

struct PerformanceWarning {
    PerformanceCheck check = PerformanceCheck::CAMERA_FRAMES_LINK;
    WarningSeverity severity = WarningSeverity::WARN;
    double start_s = 0.0;
    std::optional<double> end_s;
    std::string message;
    std::map<std::string, double> values;
};

struct MonitorTrack {
    std::string name;
    double sigma_m = 0.0;
    double speed_mps = 0.0;
};

struct FieldBox {
    double half_x = 0.0;
    double half_y = 0.0;
};

struct MonitorTick {
    double t = 0.0;
    bool autonomy_driving = false;
    std::optional<double> policy_linear;
    std::optional<double> sent_linear;
    std::optional<double> guard_scale;
    std::optional<int> guard_state;
    std::optional<FieldBox> field;
    std::optional<Pose2D> our_pose;
    bool our_heading_measured = false;
    bool opponent_expected = false;
    bool opponent_present = false;
    std::vector<MonitorTrack> tracks;
    std::optional<double> imu_yaw_rate_ccw_radps;
    std::optional<double> espnow_ack_percent;
    std::optional<double> command_age_ms;
    std::optional<FirmwareInfo> firmware;
    std::optional<uint16_t> host_protocol_version;
};

struct LinkStatus {
    MonitorLink link = MonitorLink::CAMERA_FRAMES;
    bool expected = false;
    uint64_t total = 0;
    std::optional<double> rate_hz;
    double age_s = 0.0;
};

inline constexpr int kGuardStateTripped = 4;

class PerformanceMonitor {
   public:
    explicit PerformanceMonitor(PerformanceMonitorConfiguration config);

    void add_link_events(MonitorLink link, uint64_t count = 1);
    void set_link_total(MonitorLink link, uint64_t total);
    void set_link_source(MonitorLink link, std::function<uint64_t()> source);
    bool has_link_source(MonitorLink link) const;
    void set_link_expected(MonitorLink link, bool expected, double now);

    void record_tick(const MonitorTick &tick);
    void evaluate(double now);

    std::vector<PerformanceWarning> active() const;
    const std::vector<PerformanceWarning> &ended() const { return ended_; }
    std::vector<LinkStatus> link_status(double now) const;
    const PerformanceMonitorConfiguration &config() const { return config_; }

   private:
    struct LinkLimits {
        double min_rate_hz;
        double max_age_s;
    };

    struct LinkState {
        bool expected = false;
        double expected_since = 0.0;
        uint64_t total = 0;
        std::optional<double> last_change_s;
        std::deque<std::pair<double, uint64_t>> samples;
        std::function<uint64_t()> source;
    };

    struct TickRecord {
        double t = 0.0;
        bool autonomy = false;
        std::optional<bool> slew_bound;
        std::optional<bool> guard_bound;
        std::optional<bool> outside_field;
        bool our_present = false;
        std::optional<bool> opponent_present;
        std::optional<double> espnow_ack_percent;
        std::optional<double> command_age_ms;
    };

    struct YawPair {
        double t = 0.0;
        double imu = 0.0;
        double heading_rate = 0.0;
    };

    struct CheckResult {
        PerformanceCheck check;
        WarningSeverity severity;
        std::string message;
        std::map<std::string, double> values;
    };

    LinkLimits limits(MonitorLink link) const;
    void update_link(MonitorLink link, double now);
    std::optional<double> link_rate(const LinkState &state, double now) const;
    double link_age(const LinkState &state, double now) const;
    void check_links(double now, std::vector<CheckResult> &firing) const;
    void check_ticks(double now, std::vector<CheckResult> &firing) const;
    void check_phantoms(double now, std::vector<CheckResult> &firing) const;
    void check_yaw(std::vector<CheckResult> &firing) const;
    void check_latched(std::vector<CheckResult> &firing) const;
    void apply(double now, const std::vector<CheckResult> &firing);
    void prune(double now);

    PerformanceMonitorConfiguration config_;
    std::array<std::atomic<uint64_t>, kMonitorLinkCount> pushed_{};
    std::array<LinkState, kMonitorLinkCount> links_{};
    std::deque<TickRecord> ticks_;
    std::map<std::string, double> still_large_since_;
    std::map<std::string, double> still_large_sigma_;
    std::deque<YawPair> yaw_pairs_;
    std::optional<double> last_heading_t_;
    double last_heading_ = 0.0;
    double imu_sum_ = 0.0;
    int imu_count_ = 0;
    std::optional<int> guard_state_;
    std::optional<FirmwareInfo> firmware_;
    std::optional<uint16_t> host_protocol_version_;
    std::map<PerformanceCheck, PerformanceWarning> active_;
    std::vector<PerformanceWarning> ended_;
};

}  // namespace auto_battlebot
