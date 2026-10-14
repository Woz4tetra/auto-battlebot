#include "diagnostics_server.h"

#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <web_assets.h>

#include <memory>

static AsyncWebServer server(80);
static AsyncEventSource events("/events");
static bool server_started = false;
static portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
static sensor_status_t shared_status = {};

static void handle_tunable(AsyncWebServerRequest *request, float *ptr) {
    if (!ptr) {
        request->send(500, "text/plain", "n/a");
        return;
    }
    if (request->hasParam("value")) *ptr = request->getParam("value")->value().toFloat();
    request->send(200, "text/plain", String(*ptr, 1));
}

// Age in ms of a millis() stamp at snapshot time, or JSON null when it never happened.
static String age_or_null(bool happened, uint32_t stamp_ms, uint32_t snapshot_ms) {
    return happened ? String(snapshot_ms - stamp_ms) : String("null");
}

static String status_json(const sensor_status_t &s, bool scan_pending) {
    String found = "[";
    for (uint8_t i = 0; i < s.scan.count; i++) {
        if (i) found += ",";
        found += String(s.scan.addresses[i]);
    }
    found += "]";

    const updown_sensor::status_t &m = s.imu;
    const vbat_sensor::status_t &n = s.ina;
    char buf[900];
    snprintf(buf, sizeof(buf),
             "{\"snapshot_ms\":%lu,\"bus\":{\"sda_pin\":%u,\"scl_pin\":%u,\"sda_high\":%s,\"scl_"
             "high\":%s,"
             "\"scanned\":%s,\"scan_age_ms\":%s,\"scan_pending\":%s,\"found\":%s},"
             "\"imu\":{\"initialized\":%s,\"begin_attempts\":%lu,\"begin_failures\":%lu,"
             "\"chip_id\":%u,\"last_error\":%u,\"lost_count\":%lu,\"samples\":%lu,"
             "\"sample_age_ms\":%s,\"details_age_ms\":%s,\"details_error\":%u,"
             "\"operation_mode\":%u,\"sys_status\":%u,\"self_test\":%u,\"sys_error\":%u,"
             "\"calibration\":%u,\"gyro_agree\":%lu,\"gyro_disagree\":%lu,\"yaw_rate\":%.1f,"
             "\"heading_rate\":%.1f,\"gyro_sign_suspect\":%s},"
             "\"ina\":{\"present\":%s,\"device_id\":%u,\"last_error\":%u,\"reads\":%lu,"
             "\"read_failures\":%lu,\"last_ok_age_ms\":%s}}",
             (unsigned long)s.snapshot_ms, s.sda_pin, s.scl_pin,
             s.lines.sda_high ? "true" : "false", s.lines.scl_high ? "true" : "false",
             s.scan.scanned ? "true" : "false",
             age_or_null(s.scan.scanned, s.scan.scan_ms, s.snapshot_ms).c_str(),
             scan_pending ? "true" : "false", found.c_str(), m.initialized ? "true" : "false",
             (unsigned long)m.begin_attempts, (unsigned long)m.begin_failures, m.chip_id,
             m.last_error, (unsigned long)m.lost_count, (unsigned long)m.samples,
             age_or_null(m.samples > 0, m.last_sample_ms, s.snapshot_ms).c_str(),
             age_or_null(m.details_ms != 0, m.details_ms, s.snapshot_ms).c_str(), m.details_error,
             m.operation_mode, m.sys_status, m.self_test, m.sys_error, m.calibration,
             (unsigned long)m.gyro_agree, (unsigned long)m.gyro_disagree, s.yaw_rate,
             s.heading_rate, s.gyro_sign_suspect ? "true" : "false", n.present ? "true" : "false",
             n.device_id, n.last_error, (unsigned long)n.reads, (unsigned long)n.read_failures,
             age_or_null(n.last_ok_ms != 0, n.last_ok_ms, s.snapshot_ms).c_str());
    return String(buf);
}

static const char *STATE_NAMES[] = {"ap", "connecting", "connected", "reconnecting", "fallback_ap"};

static String wifi_json(wifi_manager::WifiManager *wifi) {
    const wifi_manager::snapshot_t s = wifi->snapshot();
    JsonDocument doc;
    doc["mode"] = s.mode == wifi_manager::link_mode_t::CLIENT ? "client" : "ap";
    doc["state"] = STATE_NAMES[static_cast<uint8_t>(s.state)];
    doc["forced_ap"] = s.forced_ap;
    doc["selected"] = s.selected;
    JsonArray networks = doc["networks"].to<JsonArray>();
    for (uint8_t i = 0; i < s.count; i++) networks.add(s.networks[i]);
    doc["ssid"] = s.ssid;
    doc["ip"] = s.ip;
    doc["rssi"] = s.rssi;
    doc["connect_s_left"] = (s.connect_ms_left + 999) / 1000;
    doc["stations"] = s.stations;
    doc["ap_ssid"] = wifi->ap_ssid();
    doc["hostname"] = wifi->hostname();
    String out;
    serializeJson(doc, out);
    return out;
}

// Records go out exactly as the ring holds them, after a TelemetryBatchHeader. No `from` returns
// just the header, which tells a new client where head is without replaying the whole ring.
static void handle_telemetry(AsyncWebServerRequest *request, const telemetry_ring_t *ring,
                             uint32_t boot_id) {
    using espnow_protocol::TelemetryBatchHeader;
    using espnow_protocol::TelemetryPayload;
    uint32_t max = espnow_protocol::TELEMETRY_BATCH_MAX;
    if (request->hasParam("max")) {
        max = std::min<uint32_t>(max,
                                 strtoul(request->getParam("max")->value().c_str(), nullptr, 10));
    }
    const bool has_from = request->hasParam("from");
    const uint32_t from =
        has_from ? strtoul(request->getParam("from")->value().c_str(), nullptr, 10) : 0;
    if (!has_from) max = 0;

    const size_t capacity_bytes = sizeof(TelemetryBatchHeader) + max * sizeof(TelemetryPayload);
    // PSRAM first: a full batch is 30 KB, too much to ask of fragmented internal RAM.
    uint8_t *raw = static_cast<uint8_t *>(psramFound() ? ps_malloc(capacity_bytes) : nullptr);
    if (!raw) raw = static_cast<uint8_t *>(malloc(capacity_bytes));
    if (!raw) return request->send(503, "text/plain", "out of memory");
    std::shared_ptr<uint8_t> body(raw, free);

    TelemetryPayload *records =
        reinterpret_cast<TelemetryPayload *>(raw + sizeof(TelemetryBatchHeader));
    const telemetry_ring::read_result_t r =
        ring->read(has_from ? from : ring->head(), records, max);
    TelemetryBatchHeader header = {};
    header.magic = espnow_protocol::TELEMETRY_BATCH_MAGIC;
    header.version = espnow_protocol::TELEMETRY_BATCH_VERSION;
    header.record_size = sizeof(TelemetryPayload);
    header.boot_id = boot_id;
    header.capacity = ring->capacity();
    header.head = r.head;
    header.oldest = r.oldest;
    header.first_seq = r.first_seq;
    header.count = r.count;
    memcpy(raw, &header, sizeof(header));

    const size_t length = sizeof(TelemetryBatchHeader) + r.count * sizeof(TelemetryPayload);
    AsyncWebServerResponse *response = request->beginResponse(
        "application/octet-stream", length,
        [body, length](uint8_t *out, size_t max_len, size_t index) -> size_t {
            const size_t n = std::min(max_len, length - index);
            memcpy(out, body.get() + index, n);
            return n;
        });
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
}

static String body_param(AsyncWebServerRequest *request, const char *name) {
    return request->hasParam(name, true) ? request->getParam(name, true)->value() : String();
}

void DiagnosticsServer::begin(wifi_manager::WifiManager *wifi, const telemetry_ring_t *ring,
                              uint32_t boot_id, tunable_ptrs_t tunables) {
    _wifi = wifi;
    _ring = ring;
    _boot_id = boot_id;
    _tunables = tunables;

    for (const web_assets::asset_t &asset : web_assets::ASSETS) {
        server.on(asset.path, HTTP_GET, [&asset](AsyncWebServerRequest *request) {
            AsyncWebServerResponse *response =
                request->beginResponse(200, asset.content_type, asset.data, asset.length);
            response->addHeader("Content-Encoding", "gzip");
            response->addHeader("Cache-Control", asset.cacheable ? "max-age=604800" : "no-cache");
            request->send(response);
        });
    }

    server.on("/telemetry", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handle_telemetry(request, _ring, _boot_id);
    });

    server.on("/wifi", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(200, "application/json", wifi_json(_wifi));
    });

    server.on("/wifi/network", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (_armed) return request->send(409, "text/plain", "disarm to change WiFi settings");
        String error;
        if (!_wifi->save_network(body_param(request, "ssid").c_str(),
                                 body_param(request, "password").c_str(), error)) {
            return request->send(400, "text/plain", error);
        }
        request->send(200, "text/plain", "ok");
    });

    server.on("/wifi/network/delete", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (_armed) return request->send(409, "text/plain", "disarm to change WiFi settings");
        String error;
        if (!_wifi->delete_network(body_param(request, "ssid").c_str(), error)) {
            return request->send(400, "text/plain", error);
        }
        request->send(200, "text/plain", "ok");
    });

    server.on("/wifi/config", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (_armed) return request->send(409, "text/plain", "disarm to change WiFi settings");
        const String mode = body_param(request, "mode");
        if (mode != "ap" && mode != "client") {
            return request->send(400, "text/plain", "mode must be ap or client");
        }
        String error;
        if (!_wifi->apply(mode == "client" ? wifi_manager::link_mode_t::CLIENT
                                           : wifi_manager::link_mode_t::AP,
                          body_param(request, "selected").c_str(), error)) {
            return request->send(400, "text/plain", error);
        }
        request->send(200, "text/plain", "ok");
    });

    server.on("/record/start", HTTP_GET, [this](AsyncWebServerRequest *request) {
        _recording = true;
        request->send(200, "text/plain", "ok");
    });

    server.on("/record/stop", HTTP_GET, [this](AsyncWebServerRequest *request) {
        _recording = false;
        request->send(200, "text/plain", "ok");
    });

    server.on("/tune/left_esc_dz", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handle_tunable(request, _tunables.left_esc_deadzone);
    });

    server.on("/tune/right_esc_dz", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handle_tunable(request, _tunables.right_esc_deadzone);
    });

    server.on("/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        sensor_status_t s;
        portENTER_CRITICAL(&status_mux);
        s = shared_status;
        portEXIT_CRITICAL(&status_mux);
        request->send(200, "application/json", status_json(s, _scan_requested));
    });

    server.on("/i2c/scan", HTTP_GET, [this](AsyncWebServerRequest *request) {
        _scan_requested = true;
        request->send(200, "text/plain", "ok");
    });

    events.onConnect(
        [](AsyncEventSourceClient *client) { client->send("connected", NULL, millis(), 1000); });

    server.addHandler(&events);
    server.begin();
    server_started = true;
}

bool DiagnosticsServer::has_clients() { return server_started && events.count() > 0; }

void DiagnosticsServer::set_status(const sensor_status_t &status) {
    portENTER_CRITICAL(&status_mux);
    shared_status = status;
    portEXIT_CRITICAL(&status_mux);
}

void DiagnosticsServer::update(const diag_data_t *data) {
    _armed = data->armed;
    if (!server_started || events.count() == 0) return;

    uint32_t now = millis();
    if (!_recording && (now - _last_send_ms < 100)) return;
    _last_send_ms = now;

    char buf[320];
    snprintf(buf, sizeof(buf),
             "%lu,%d,%d,%.1f,%.1f,%d,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%lu,%u,%.1f,%.1f,%.1f,%.1f,%."
             "2f,%.3f,%.2f,%.1f,%.1f,%lu",
             (unsigned long)data->timestamp_ms, data->radio_connected, data->armed, data->a_percent,
             data->b_percent, data->button_state, data->flip_switch, data->left_cmd,
             data->right_cmd, data->accel_x, data->accel_y, data->accel_z, data->is_upside_down,
             (unsigned long)data->loop_us, data->wifi_clients, data->orientation_x,
             data->orientation_y, data->orientation_z, data->pid_setpoint, data->pid_output,
             data->vbat, data->ibat, data->yaw_rate, data->yaw_rate_cmd, (unsigned long)_seq++);

    events.send(buf, NULL, now);
}
