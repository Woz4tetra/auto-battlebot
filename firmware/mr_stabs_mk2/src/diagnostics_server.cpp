#include "diagnostics_server.h"

#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <tuning_store.h>
#include <web_assets.h>

#include <memory>

static AsyncWebServer server(80);
static AsyncEventSource events("/events");
static bool server_started = false;
static portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE pair_mux = portMUX_INITIALIZER_UNLOCKED;
static sensor_status_t shared_status = {};

// Every runtime setting with its range and unit, and whether NVS holds the current values.
static String tuning_json(TuningStore *store) {
    JsonDocument doc;
    const tuning::Settings now = store->get();
    doc["saved"] = store->saved();
    JsonObject fields = doc["fields"].to<JsonObject>();
    for (size_t i = 0; i < tuning::FIELD_COUNT; i++) {
        const tuning::Field &field = tuning::FIELDS[i];
        JsonObject f = fields[field.key].to<JsonObject>();
        f["value"] = tuning::get(now, field);
        f["unit"] = field.unit;
        f["min"] = field.min;
        f["max"] = field.max;
    }
    String out;
    serializeJson(doc, out);
    return out;
}

// Age in ms of a millis() stamp at snapshot time, or JSON null when it never happened.
static String age_or_null(bool happened, uint32_t stamp_ms, uint32_t snapshot_ms) {
    return happened ? String(snapshot_ms - stamp_ms) : String("null");
}

static String mac_string(const uint8_t *mac) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3],
             mac[4], mac[5]);
    return String(buf);
}

static void espnow_json(JsonObject out, const espnow_status_t &e, uint32_t snapshot_ms) {
    out["enabled"] = e.enabled;
    out["substitution"] = e.substitution;
    out["begin_ok"] = e.begin_ok;
    out["driving"] = e.driving;
    out["channel"] = e.channel;
    out["sweeping"] = e.sweeping;
    out["last_good_channel"] = e.last_good_channel;
    out["heard_dongle"] = e.heard_dongle;
    out["dongle_mac"] = mac_string(e.dongle_mac);
    if (e.heard_dongle) {
        out["frame_age_ms"] = snapshot_ms - e.last_frame_ms;
    } else {
        out["frame_age_ms"] = nullptr;
    }
    out["rx_frames"] = e.rx_frames;
    out["rx_commands_new"] = e.rx_commands_new;
    out["rx_commands_repeat"] = e.rx_commands_repeat;
    out["rx_commands_old"] = e.rx_commands_old;
    out["rx_pings"] = e.rx_pings;
    out["rx_tunables"] = e.rx_tunables;
    out["rx_ota"] = e.rx_ota;
    out["rx_size_errors"] = e.rx_size_errors;
    out["rx_rejected"] = e.rx_rejected;
    out["rejected_mac"] = mac_string(e.rejected_mac);
    out["rx_queue_full"] = e.rx_queue_full;
    out["tx_telemetry"] = e.tx_telemetry;
    out["tx_ok"] = e.tx_ok;
    out["tx_fail"] = e.tx_fail;
    out["tx_errors"] = e.tx_errors;
    out["lq"] = e.lq;
    out["command_fresh"] = e.command_fresh;
    out["command_seq"] = e.command_seq;
    if (e.command_age_ms == UINT32_MAX) {
        out["command_age_ms"] = nullptr;
    } else {
        out["command_age_ms"] = e.command_age_ms;
    }
    out["command_a"] = e.command_a;
    out["command_b"] = e.command_b;
    out["ota_state"] = e.ota_state;
    out["validated"] = e.validated;
    JsonArray allowed = out["allowed"].to<JsonArray>();
    for (uint8_t i = 0; i < e.allowed_count && i < 12; i++) allowed.add(mac_string(e.allowed[i]));
    out["pair_result"] = e.pair_result;
}

static bool parse_mac(const String &text, uint8_t *mac) {
    unsigned int b[6];
    if (text.length() != 17 || sscanf(text.c_str(), "%2x:%2x:%2x:%2x:%2x:%2x", &b[0], &b[1], &b[2],
                                      &b[3], &b[4], &b[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) mac[i] = static_cast<uint8_t>(b[i]);
    return true;
}

static void boot_json(JsonObject out, const crash_report::report_t &b) {
    out["reset_reason"] = crash_report::reset_reason_name(b.reset_reason);
    out["has_dump"] = b.has_dump;
    if (!b.has_dump) return;
    out["dump_valid"] = b.dump_valid;
    out["dump_size"] = b.dump_size;
    out["task"] = b.task;
    char hex[11];
    snprintf(hex, sizeof(hex), "0x%08x", b.pc);
    out["pc"] = hex;
    out["exc_cause"] = b.exc_cause;
    snprintf(hex, sizeof(hex), "0x%08x", b.exc_vaddr);
    out["exc_vaddr"] = hex;
    JsonArray bt = out["backtrace"].to<JsonArray>();
    for (uint8_t i = 0; i < b.depth; i++) {
        snprintf(hex, sizeof(hex), "0x%08x", b.backtrace[i]);
        bt.add(hex);
    }
    out["backtrace_corrupted"] = b.backtrace_corrupted;
    out["elf_sha256"] = b.elf_sha256;
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

    // The ESP-NOW block goes in through ArduinoJson and is spliced in before the closing brace.
    // Serialized into its own String: ArduinoJson 7 clears a String before writing to it.
    JsonDocument doc;
    espnow_json(doc.to<JsonObject>(), s.espnow, s.snapshot_ms);
    String espnow;
    serializeJson(doc, espnow);
    JsonDocument boot_doc;
    boot_json(boot_doc.to<JsonObject>(), s.boot);
    String boot;
    serializeJson(boot_doc, boot);
    String out(buf);
    out.remove(out.length() - 1);
    out += ",\"espnow\":";
    out += espnow;
    out += ",\"boot\":";
    out += boot;
    out += "}";
    return out;
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
    doc["ap_channel"] = wifi->ap_channel();
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

static int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Hex text to bytes. False on an odd length, a non-hex character, or more than max bytes.
static bool parse_hex(const String &text, uint8_t *out, size_t max, size_t &size) {
    if (text.length() % 2 != 0 || text.length() / 2 > max) return false;
    size = text.length() / 2;
    for (size_t i = 0; i < size; i++) {
        const int hi = hex_digit(text[2 * i]);
        const int lo = hex_digit(text[2 * i + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return true;
}

static String esc_status_json(const esc_config::status_t &s) {
    JsonDocument doc;
    doc["state"] = esc_config::state_name(s.state);
    doc["error"] = s.error;
    doc["frames"] = s.frames;
    doc["guard_refusals"] = s.guard_refusals;
    doc["idle_s"] = s.idle_ms / 1000;
    doc["idle_release_s"] = esc_config::IDLE_RELEASE_MS / 1000;
    doc["released"] = s.released;
    doc["pending"] = s.pending;
    doc["armed"] = s.armed;
    doc["driving"] = s.driving;
    doc["failure"] = s.failure;
    String out;
    serializeJson(doc, out);
    return out;
}

// The 4-way relay for tuning mode (docs/plans/esc_settings_dashboard_plan.md). The loop runs one
// frame at a time: POST queues it and returns an id, GET reports it pending or done.
static void setup_esc_routes(AsyncWebServer &server, esc_config::EscConfig *esc,
                             volatile bool *armed) {
    server.on("/esc/status", HTTP_GET, [esc](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response =
            request->beginResponse(200, "application/json", esc_status_json(esc->status()));
        response->addHeader("Cache-Control", "no-store");
        request->send(response);
    });

    server.on("/esc/tuning", HTTP_POST, [esc, armed](AsyncWebServerRequest *request) {
        const String enter = body_param(request, "enter");
        if (enter != "0" && enter != "1") {
            return request->send(400, "text/plain", "enter must be 0 or 1");
        }
        if (enter == "1" && *armed) {
            return request->send(409, "text/plain", "disarm to enter tuning mode");
        }
        esc->request(enter == "1");
        request->send(200, "text/plain", "queued");
    });

    server.on("/esc/4way", HTTP_POST, [esc](AsyncWebServerRequest *request) {
        uint8_t frame[blheli_boot::MAX_REQUEST];
        size_t size = 0;
        if (!parse_hex(body_param(request, "frame"), frame, sizeof(frame), size)) {
            return request->send(400, "text/plain", "frame must be hex, at most 263 bytes");
        }
        blheli_boot::Request parsed;
        const blheli_boot::ParseResult result = blheli_boot::parse_request(frame, size, parsed);
        if (result != blheli_boot::ParseResult::OK) {
            return request->send(400, "text/plain", blheli_boot::parse_result_name(result));
        }
        uint32_t id = 0;
        const char *reason = "";
        if (!esc->submit(frame, size, id, reason)) return request->send(409, "text/plain", reason);
        request->send(200, "application/json", String("{\"id\":") + id + "}");
    });

    server.on("/esc/4way", HTTP_GET, [esc](AsyncWebServerRequest *request) {
        uint8_t frame[blheli_boot::MAX_RESPONSE];
        size_t size = 0;
        uint32_t id = 0;
        const esc_config::Poll state = esc->poll(id, frame, size);
        String out = String("{\"id\":") + id + ",\"state\":\"";
        out += state == esc_config::Poll::DONE      ? "done"
               : state == esc_config::Poll::PENDING ? "pending"
                                                    : "none";
        out += "\"";
        if (state == esc_config::Poll::DONE) {
            static const char HEX_DIGITS[] = "0123456789abcdef";
            out += ",\"frame\":\"";
            out.reserve(out.length() + 2 * size + 2);
            for (size_t i = 0; i < size; i++) {
                out += HEX_DIGITS[frame[i] >> 4];
                out += HEX_DIGITS[frame[i] & 0xF];
            }
            out += "\"";
        }
        out += "}";
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", out);
        response->addHeader("Cache-Control", "no-store");
        request->send(response);
    });
}

void DiagnosticsServer::begin(wifi_manager::WifiManager *wifi, const telemetry_ring_t *ring,
                              uint32_t boot_id, TuningStore *tuning,
                              esc_config::EscConfig *esc_tuning) {
    _wifi = wifi;
    _ring = ring;
    _boot_id = boot_id;
    _tuning = tuning;
    _esc = esc_tuning;

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

    server.on("/tuning", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (!_tuning) return request->send(500, "text/plain", "n/a");
        request->send(200, "application/json", tuning_json(_tuning));
    });

    // Form fields key=value, any subset. Checks every field first and applies none on a bad one.
    server.on("/tuning", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (!_tuning) return request->send(500, "text/plain", "n/a");
        tuning::Settings check = _tuning->get();
        for (size_t i = 0; i < request->params(); i++) {
            const AsyncWebParameter *param = request->getParam(i);
            if (!param->isPost()) continue;
            char *end = nullptr;
            const float value = strtof(param->value().c_str(), &end);
            if (end == param->value().c_str() || *end != '\0' ||
                !tuning::set(check, param->name().c_str(), value)) {
                return request->send(400, "text/plain",
                                     "bad value or unknown field: " + param->name());
            }
        }
        for (size_t i = 0; i < request->params(); i++) {
            const AsyncWebParameter *param = request->getParam(i);
            if (param->isPost()) _tuning->set(param->name().c_str(), param->value().toFloat());
        }
        request->send(200, "application/json", tuning_json(_tuning));
    });

    server.on("/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        sensor_status_t s;
        portENTER_CRITICAL(&status_mux);
        s = shared_status;
        portEXIT_CRITICAL(&status_mux);
        request->send(200, "application/json", status_json(s, _scan_requested));
    });

    for (const char *path : {"/espnow/pair", "/espnow/unpair"}) {
        const uint8_t op = strcmp(path, "/espnow/pair") == 0 ? 1 : 2;
        server.on(path, HTTP_POST, [this, op](AsyncWebServerRequest *request) {
            if (_armed) return request->send(409, "text/plain", "disarm to change dongles");
            pair_request_t req = {};
            req.op = op;
            if (!parse_mac(body_param(request, "mac"), req.mac)) {
                return request->send(400, "text/plain", "mac must be aa:bb:cc:dd:ee:ff");
            }
            portENTER_CRITICAL(&pair_mux);
            _pair_request = req;
            portEXIT_CRITICAL(&pair_mux);
            request->send(200, "text/plain", "queued");
        });
    }

    server.on("/crash/erase", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (_armed) return request->send(409, "text/plain", "disarm to erase the dump");
        _erase_dump_requested = true;
        request->send(200, "text/plain", "queued");
    });

    server.on("/i2c/scan", HTTP_GET, [this](AsyncWebServerRequest *request) {
        _scan_requested = true;
        request->send(200, "text/plain", "ok");
    });

    if (_esc) setup_esc_routes(server, _esc, &_armed);

    events.onConnect(
        [](AsyncEventSourceClient *client) { client->send("connected", NULL, millis(), 1000); });

    server.addHandler(&events);
    server.begin();
    server_started = true;
}

bool DiagnosticsServer::has_clients() { return server_started && events.count() > 0; }

bool DiagnosticsServer::take_pair_request(pair_request_t &out) {
    portENTER_CRITICAL(&pair_mux);
    out = _pair_request;
    _pair_request.op = 0;
    portEXIT_CRITICAL(&pair_mux);
    return out.op != 0;
}

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

    char buf[448];
    snprintf(buf, sizeof(buf),
             "%lu,%d,%d,%.1f,%.1f,%d,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%lu,%u,%.1f,%.1f,%.1f,%.1f,%."
             "2f,%.3f,%.2f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,%u,%u,%u,%u,%lu",
             (unsigned long)data->timestamp_ms, data->radio_connected, data->armed, data->a_percent,
             data->b_percent, data->button_state, data->flip_switch, data->left_cmd,
             data->right_cmd, data->accel_x, data->accel_y, data->accel_z, data->is_upside_down,
             (unsigned long)data->loop_us, data->wifi_clients, data->orientation_x,
             data->orientation_y, data->orientation_z, data->pid_setpoint, data->pid_output,
             data->vbat, data->ibat, data->yaw_rate, data->yaw_rate_cmd, data->motor_rise_left,
             data->motor_rise_right, data->motor_speed_left, data->motor_speed_right,
             data->motor_amps_left, data->motor_amps_right, data->guard_scale, data->guard_gain,
             data->guard_state, data->guard_sensors, data->stall_flags, data->erpm_crc_errors,
             (unsigned long)_seq++);

    events.send(buf, NULL, now);
}
