#pragma once

#include <ArduinoJson.h>
#include <math.h>
#include <stdint.h>

namespace status_doc {

inline void age_or_null(JsonObject out, const char *key, bool happened, uint32_t stamp_ms,
                        uint32_t snapshot_ms) {
    if (happened) {
        out[key] = snapshot_ms - stamp_ms;
    } else {
        out[key] = nullptr;
    }
}

inline float round_tenth(float value) { return roundf(value * 10.0f) / 10.0f; }

template <class Status>
void fill_sensors(JsonObject root, const Status &s, bool scan_pending) {
    root["snapshot_ms"] = s.snapshot_taken_ms;

    JsonObject bus = root["bus"].template to<JsonObject>();
    bus["sda_pin"] = s.sda_pin;
    bus["scl_pin"] = s.scl_pin;
    bus["sda_high"] = s.lines.sda_high;
    bus["scl_high"] = s.lines.scl_high;
    bus["scanned"] = s.scan.scanned;
    age_or_null(bus, "scan_age_ms", s.scan.scanned, s.scan.scanned_at_ms, s.snapshot_taken_ms);
    bus["scan_pending"] = scan_pending;
    JsonArray found = bus["found"].template to<JsonArray>();
    for (uint8_t i = 0; i < s.scan.count; i++) found.add(s.scan.addresses[i]);

    const auto &m = s.imu;
    JsonObject imu = root["imu"].template to<JsonObject>();
    imu["initialized"] = m.initialized;
    imu["begin_attempts"] = m.begin_attempts;
    imu["begin_failures"] = m.begin_failures;
    imu["chip_id"] = m.last_chip_id;
    imu["last_error"] = static_cast<unsigned>(m.chip_id_read_error);
    imu["lost_count"] = m.samples_lost_after_begin;
    imu["samples"] = m.samples;
    age_or_null(imu, "sample_age_ms", m.samples > 0, m.last_sample_ms, s.snapshot_taken_ms);
    age_or_null(imu, "details_age_ms", m.details_ms != 0, m.details_ms, s.snapshot_taken_ms);
    imu["details_error"] = static_cast<unsigned>(m.details_error);
    imu["operation_mode"] = m.operation_mode;
    imu["sys_status"] = m.sys_status;
    imu["self_test"] = m.self_test;
    imu["sys_error"] = m.sys_error;
    imu["calibration"] = m.calibration;
    imu["gyro_agree"] = m.gyro_agree;
    imu["gyro_disagree"] = m.gyro_disagree;
    imu["yaw_rate"] = round_tenth(s.yaw_rate);
    imu["heading_rate"] = round_tenth(s.heading_rate);
    imu["gyro_sign_suspect"] = s.gyro_sign_suspect;

    const auto &n = s.ina;
    JsonObject ina = root["ina"].template to<JsonObject>();
    ina["present"] = n.present;
    ina["device_id"] = n.device_id;
    ina["last_error"] = static_cast<unsigned>(n.last_error);
    ina["reads"] = n.reads;
    ina["read_failures"] = n.read_failures;
    age_or_null(ina, "last_ok_age_ms", n.last_ok_ms != 0, n.last_ok_ms, s.snapshot_taken_ms);
}

}  // namespace status_doc
