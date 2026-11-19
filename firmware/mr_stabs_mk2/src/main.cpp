#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include <crash_report.h>
#include <crsf_bridge.h>
#include <diagnostics_server.h>
#include <esc.h>
#include <esc_config.h>
#include <esp_attr.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <espnow_link.h>
#include <i2c_bus.h>
#include <motor_guard.h>
#include <telemetry_ring.h>
#include <tuning_store.h>
#include <updown_sensor.h>
#include <usb_bridge.h>
#include <vbat_sensor.h>
#include <wifi_manager.h>
#include <yaw_control.h>

const char *WIFI_SSID = "MR-STABS";
const char *WIFI_PASSWORD = "havocbots";
const char *HOSTNAME = "mr-stabs";
wifi_manager::WifiManager wifi;

crsf_bridge::CrsfBridge *crsf;
crsf_bridge::radio_data_t *radio_data;

#define LEFT_ESC_PIN ((gpio_num_t)A3)
#define RIGHT_ESC_PIN ((gpio_num_t)A2)

esc::Esc *left_esc;
esc::Esc *right_esc;

esc_config::EscConfig esc_tuning;

updown_sensor::UpdownSensor *accel;
vbat_sensor::VbatSensor vbat_sensor_ina238;

// Stall protection (docs/plans/motor_stall_protection_plan.md): scales both drive commands down
// when a motor's estimated heat says it is cooking. Updated once per INA238 sample, or every
// GUARD_PERIOD_US without one, armed or not, so disarming does not hand back a cold budget.
motor_guard::MotorGuard guard{motor_guard::Config()};
const uint32_t GUARD_PERIOD_US = 10000;
uint32_t last_guard_us = 0;
uint32_t last_guard_reads = 0;
float last_left_scaled_duty = 0.0f;
float last_right_scaled_duty = 0.0f;

struct guard_rtc_t {
    uint32_t magic;
    float rise_c[motor_guard::NUM_MOTORS];
};
const uint32_t GUARD_RTC_MAGIC = 0x4D475244;
RTC_NOINIT_ATTR guard_rtc_t guard_rtc;
DiagnosticsServer diag_server;

const uint32_t TELEMETRY_RING_PSRAM_RECORDS = 9000;
const uint32_t TELEMETRY_RING_HEAP_RECORDS = 500;
telemetry_ring::TelemetryRing<espnow_protocol::TelemetryPayload> telemetry_ring_buffer;
uint32_t last_ring_push_ms = 0;
i2c_bus::scan_t bus_scan = {};
uint32_t last_status_ms = 0;
void request_status_refresh() { last_status_ms = 0; }
const uint32_t SENSOR_PANEL_REFRESH_MS = 1000;
const size_t USB_BRIDGE_RX_BUFFER_BYTES = 4096;
bool last_espnow_driving = false;
crash_report::report_t boot_reset_report = {};

const int NUM_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int rainbow_tick = 0, led_intensity = 20;
uint32_t last_led_ms = 0;

bool is_loading_firmware = false;
bool prev_button_state = false;
uint32_t prev_loop_us = 0;

yaw_control::YawController *yaw_controller;
float steer_differential_pct = 0.0f;
bool yaw_loop_active = false;
bool was_upside_down = false;
const float TURN_DEADBAND_PERCENT = yaw_control::Config{}.turn_threshold;
const uint32_t HEADING_STALE_US = 100000;
uint32_t last_heading_sample_us = 0;

TuningStore tuning_store;

espnow_link::EspNowLink espnow;

const bool ESPNOW_ENABLED = true;

// With SA on and a fresh ESP-NOW command, drive from it instead of the handset sticks; stale
// ESP-NOW falls back to the sticks. False is shadow mode: commands are received and reported but
// never drive. The ZED Box sends over ESP-NOW alone, so with this false the Jetson cannot drive.
// Nothing rate-limits the arbitrated output: a source switch, an ESP-NOW dropout, or an
// orientation flip steps the wheels in one tick.
const bool ESPNOW_COMMAND_SUBSTITUTION = true;

void set_builtin_led(int value) {
    pixels.fill(pixels.Color(value, 0, 0));
    pixels.show();
}

void pulse_led() {
    for (int count = 0; count < 255; count += 5) {
        set_builtin_led(count);
        delay(1);
    }
    for (int count = 255; count > 0; count -= 5) {
        set_builtin_led(count);
        delay(1);
    }
    set_builtin_led(0);
}

void cycle_rainbow_led(int tick, int brightness) {
    for (int i = 0; i < NUM_PIXELS; i++) {
        pixels.setPixelColor(
            i, pixels.ColorHSV(tick * 65536 / 255 + i * 65536 / NUM_PIXELS, 255, 255));
    }
    pixels.setBrightness(brightness);
    pixels.show();
}

void breathe_blue_led(uint32_t now_ms) {
    const float phase = (now_ms % 3000) / 3000.0f;
    const uint8_t level = (uint8_t)(10 + 150 * (0.5f - 0.5f * cosf(2.0f * PI * phase)));
    pixels.setBrightness(255);
    pixels.fill(pixels.Color(0, 0, level));
    pixels.show();
}

void set_led_intensity(float percent) {
    led_intensity = (int)(2.35 * min(100.0f, max(-100.0f, percent))) + 20;
}

void apply_tuning(bool armed) {
    tuning::Settings t;
    if (tuning_store.take_change(t)) {
        left_esc->stop_threshold = t.left_deadzone;
        right_esc->stop_threshold = t.right_deadzone;
        yaw_controller->set_config(t.yaw);
    }
    tuning_store.save(armed);
}

void stop_escs() {
    esc::Esc::stop_pair(*left_esc, *right_esc);
    last_left_scaled_duty = 0.0f;
    last_right_scaled_duty = 0.0f;
    set_led_intensity(0);
}

void advance_fixed_rate_schedule(uint32_t &last_ms, uint32_t period_ms, uint32_t now_ms) {
    last_ms += period_ms;
    if (now_ms - last_ms >= period_ms) last_ms = now_ms;
}

void publish_diag(diag_data_t &diag, bool autonomy_enabled, bool espnow_active, uint32_t now_ms) {
    diag.motor_rise_left_c = guard.rise_c(0);
    diag.motor_rise_right_c = guard.rise_c(1);
    diag.motor_speed_left_rad_s = guard.motor_shaft_speed_signed_rad_s(0);
    diag.motor_speed_right_rad_s = guard.motor_shaft_speed_signed_rad_s(1);
    diag.motor_amps_est_left = guard.amps_estimate(0);
    diag.motor_amps_est_right = guard.amps_estimate(1);
    diag.guard_scale = guard.scale();
    diag.guard_shunt_gain = guard.shunt_gain();
    diag.guard_state = guard.state();
    diag.guard_sensors = guard.sensors();
    diag.stall_flags =
        (guard.stalled(0) ? STALL_FLAG_LEFT : 0) | (guard.stalled(1) ? STALL_FLAG_RIGHT : 0);
    diag.erpm_crc_errors = (uint16_t)min<uint32_t>(
        left_esc->reply_decode_errors() + right_esc->reply_decode_errors(), UINT16_MAX);
    diag_server.update(&diag);
    last_espnow_driving = espnow_active;
    const bool ring_due = now_ms - last_ring_push_ms >= espnow_link::TELEMETRY_PERIOD_MS;
    if (!ring_due && !ESPNOW_ENABLED) return;
    espnow_protocol::TelemetryPayload telemetry = {};
    telemetry.timestamp_ms = diag.timestamp_ms;
    telemetry.loop_us = diag.loop_us;
    telemetry.a_percent = diag.a_percent;
    telemetry.b_percent = diag.b_percent;
    telemetry.left_cmd = diag.left_cmd;
    telemetry.right_cmd = diag.right_cmd;
    telemetry.accel_x = diag.accel_x;
    telemetry.accel_y = diag.accel_y;
    telemetry.accel_z = diag.accel_z;
    telemetry.orientation_x = diag.orientation_x;
    telemetry.orientation_y = diag.orientation_y;
    telemetry.orientation_z = diag.orientation_z;
    telemetry.pid_setpoint = diag.pid_setpoint;
    telemetry.pid_output = diag.pid_output;
    telemetry.vbat = diag.pack_volts;
    telemetry.ibat = diag.pack_amps_discharge;
    telemetry.yaw_rate = diag.yaw_rate_cw_dps;
    telemetry.yaw_rate_cmd = diag.yaw_rate_cmd_cw_dps;
    updown_sensor::vector3_t *linear_accel = accel->get_linear_accel_mps2_chip();
    telemetry.linear_accel_x = linear_accel->x;
    telemetry.linear_accel_y = linear_accel->y;
    telemetry.linear_accel_z = linear_accel->z;
    updown_sensor::vector3_t *gyro = accel->get_gyro_rad_s_chip();
    telemetry.gyro_x = gyro->x * RAD_TO_DEG;
    telemetry.gyro_y = gyro->y * RAD_TO_DEG;
    telemetry.imu_calibration = accel->get_status().calibration;
    telemetry.radio_connected = diag.radio_connected;
    telemetry.armed = diag.armed;
    telemetry.autonomy_enabled = autonomy_enabled;
    telemetry.button_state = diag.button_state;
    telemetry.flip_switch = diag.flip_switch;
    telemetry.is_upside_down = diag.is_upside_down;
    telemetry.espnow_active = espnow_active;
    telemetry.motor_rise_left = diag.motor_rise_left_c;
    telemetry.motor_rise_right = diag.motor_rise_right_c;
    telemetry.motor_speed_left = diag.motor_speed_left_rad_s;
    telemetry.motor_speed_right = diag.motor_speed_right_rad_s;
    telemetry.motor_amps_left = diag.motor_amps_est_left;
    telemetry.motor_amps_right = diag.motor_amps_est_right;
    telemetry.guard_scale = diag.guard_scale;
    telemetry.guard_gain = diag.guard_shunt_gain;
    telemetry.guard_state = static_cast<uint8_t>(diag.guard_state);
    telemetry.guard_sensors = static_cast<uint8_t>(diag.guard_sensors);
    telemetry.stall_flags = diag.stall_flags;
    telemetry.erpm_crc_errors = diag.erpm_crc_errors;
    if (ring_due) {
        advance_fixed_rate_schedule(last_ring_push_ms, espnow_link::TELEMETRY_PERIOD_MS, now_ms);
        telemetry_ring_buffer.push(telemetry);
    }
    if (ESPNOW_ENABLED) espnow.send_telemetry(telemetry, now_ms);
}

void setup_telemetry_ring() {
    uint32_t capacity = TELEMETRY_RING_PSRAM_RECORDS;
    void *storage =
        psramFound() ? ps_malloc(capacity * sizeof(espnow_protocol::TelemetryPayload)) : nullptr;
    if (!storage) {
        capacity = TELEMETRY_RING_HEAP_RECORDS;
        storage = malloc(capacity * sizeof(espnow_protocol::TelemetryPayload));
    }
    if (!storage) capacity = 0;
    telemetry_ring_buffer.init(static_cast<espnow_protocol::TelemetryPayload *>(storage), capacity);
    Serial.printf("telemetry ring: %lu records (%s)\n", (unsigned long)capacity,
                  capacity == TELEMETRY_RING_PSRAM_RECORDS ? "PSRAM" : "internal RAM");
}

void publish_sensor_status(bool armed) {
    if (!diag_server.page_open()) return;
    if (diag_server.scan_requested() && !armed) {
        i2c_bus::scan(&Wire1, &bus_scan, armed);
        diag_server.clear_scan_request();
        request_status_refresh();
    }
    uint32_t now = millis();
    if (last_status_ms != 0 && now - last_status_ms < SENSOR_PANEL_REFRESH_MS) return;
    last_status_ms = now;

    accel->refresh_details();
    sensor_status_t status = {};
    status.imu = accel->get_status();
    status.ina = vbat_sensor_ina238.get_status();
    status.scan = bus_scan;
    status.lines = i2c_bus::read_lines(SDA1, SCL1);
    status.sda_pin = SDA1;
    status.scl_pin = SCL1;
    status.snapshot_taken_ms = millis();
    status.yaw_rate = accel->get_yaw_rate_cw_dps();
    status.heading_rate = accel->get_heading_rate_cw_dps();
    status.gyro_sign_suspect = accel->gyro_sign_suspect();
    if (ESPNOW_ENABLED) espnow.fill_status(status.espnow, status.snapshot_taken_ms);
    status.espnow.enabled = ESPNOW_ENABLED;
    status.espnow.command_substitution = ESPNOW_COMMAND_SUBSTITUTION;
    status.espnow.driving = last_espnow_driving;
    status.boot = boot_reset_report;
    diag_server.set_status(status);
}

void setup_ota() {
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA
        .onStart([]() {
            is_loading_firmware = true;
            stop_escs();
            // handle() runs the whole upload inside one loop() pass.
            disableLoopWDT();
        })
        .onEnd([]() {})
        .onProgress([](unsigned int progress, unsigned int total) {})
        .onError([](ota_error_t error) { enableLoopWDT(); });

    ArduinoOTA.begin();
}

void update_motor_guard(uint32_t now_us) {
    const uint32_t reads = vbat_sensor_ina238.get_status().reads;
    const bool fresh = reads != last_guard_reads;
    if (!fresh && now_us - last_guard_us < GUARD_PERIOD_US) return;
    const float dt_s = (now_us - last_guard_us) / 1000000.0f;
    last_guard_us = now_us;
    last_guard_reads = reads;
    const float volts = fresh ? vbat_sensor_ina238.get_pack_volts() : NAN;
    const float amps = fresh ? vbat_sensor_ina238.get_pack_amps_discharge() : NAN;
    const float left_erpm = left_esc->erpm_valid() ? (float)left_esc->erpm() : -1.0f;
    const float right_erpm = right_esc->erpm_valid() ? (float)right_esc->erpm() : -1.0f;
    const bool was_tripped = guard.tripped();
    guard.update(dt_s, volts, amps, last_left_scaled_duty, last_right_scaled_duty, left_erpm,
                 right_erpm);
    guard_rtc.rise_c[0] = guard.rise_c(0);
    guard_rtc.rise_c[1] = guard.rise_c(1);
    if (guard.tripped() && !was_tripped) {
        Serial.printf("motor guard: TRIPPED at %.1f / %.1f C, drive cut until disarm\n",
                      guard.rise_c(0), guard.rise_c(1));
    }
}

void reset_steering(float sensed_angle_z) {
    yaw_controller->reset(sensed_angle_z);
    steer_differential_pct = 0.0f;
    yaw_loop_active = false;
}

void mix_motor_outputs(crsf_bridge::radio_data_t *radio_data, float sensed_angle_z,
                       bool auto_steer_enabled, bool is_upside_down, uint32_t now_us,
                       float &left_command, float &right_command) {
    float a_percent = radio_data->a_percent;
    float b_percent = radio_data->b_percent;
    // a_percent is the negated throttle channel, so pulling the stick back reads positive, and
    // loop() negates it again when inverted. Either way, stick back drives tail-first.
    float tail_first_throttle = is_upside_down ? -a_percent : a_percent;

    uint32_t sample_us = accel->get_sample_us();
    bool fresh_heading = sample_us != 0 && sample_us != last_heading_sample_us;
    float heading_dt =
        yaw_control::heading_dt_s(sample_us, last_heading_sample_us, HEADING_STALE_US);
    if (fresh_heading) last_heading_sample_us = sample_us;
    bool heading_stale = sample_us == 0 || now_us - sample_us > HEADING_STALE_US;

    bool yaw_usable = auto_steer_enabled && !heading_stale && !accel->gyro_sign_suspect();
    if (!yaw_usable) {
        steer_differential_pct = fabs(b_percent) > TURN_DEADBAND_PERCENT ? b_percent : 0.0f;
        yaw_loop_active = false;
    } else {
        if (!yaw_loop_active || is_upside_down != was_upside_down) {
            yaw_controller->reset(sensed_angle_z);
            yaw_loop_active = true;
        }
        if (fresh_heading) {
            steer_differential_pct =
                yaw_controller->update(b_percent, tail_first_throttle, sensed_angle_z,
                                       accel->get_yaw_rate_cw_dps(), heading_dt);
        }
    }
    was_upside_down = is_upside_down;

    left_command = -1 * a_percent + steer_differential_pct;
    right_command = -1 * a_percent - steer_differential_pct;
    float max_command = max(abs(left_command), abs(right_command));
    if (max_command > 100.0) {
        left_command = left_command / max_command * 100.0;
        right_command = right_command / max_command * 100.0;
    }
}

esp_timer_handle_t setup_frames_timer = nullptr;

void send_setup_frames(void *) { esc::Esc::stop_pair(*left_esc, *right_esc); }

void start_setup_frames() {
    esp_timer_create_args_t args = {};
    args.callback = send_setup_frames;
    args.name = "esc_setup";
    esp_timer_create(&args, &setup_frames_timer);
    esp_timer_start_periodic(setup_frames_timer, 5000);
}

void stop_setup_frames() {
    esp_timer_stop(setup_frames_timer);
    delay(10);  // a callback already running finishes before loop() sends its own frames
    esp_timer_delete(setup_frames_timer);
    setup_frames_timer = nullptr;
}

void setup() {
    Serial.setRxBufferSize(USB_BRIDGE_RX_BUFFER_BYTES);
    Serial.setTxBufferSize(4096);
    Serial.begin(115200);
    crash_report::read(boot_reset_report);
    crash_report::print(boot_reset_report);

#if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

    left_esc = new esc::Esc(LEFT_ESC_PIN, RMT_CHANNEL_3, RMT_CHANNEL_4);
    right_esc = new esc::Esc(RIGHT_ESC_PIN, RMT_CHANNEL_2, RMT_CHANNEL_5);
    tuning_store.begin();
    left_esc->begin();
    right_esc->begin();
    esc_tuning.begin(right_esc, left_esc, RIGHT_ESC_PIN, LEFT_ESC_PIN);
    start_setup_frames();
    const uint32_t esc_start_ms = millis();

    pixels.begin();
    pixels.setBrightness(20);

    for (int count = 0; count < 2; count++) pulse_led();

    Wire1.begin();
    i2c_bus::scan(&Wire1, &bus_scan, false);
    accel = new updown_sensor::UpdownSensor();
    if (!accel->begin()) {
        for (int count = 0; count < 10; count++) pulse_led();
    }
    // INA238 pack voltage and current on the same bus. If it is missing, vbat and ibat report
    // NaN and the robot runs normally.
    vbat_sensor_ina238.begin(&Wire1);
    set_builtin_led(255);

    radio_data = (crsf_bridge::radio_data_t *)malloc(sizeof(crsf_bridge::radio_data_t));
    crsf = new crsf_bridge::CrsfBridge();
    crsf->begin();

    yaw_controller = new yaw_control::YawController(tuning_store.get().yaw);

    if (esp_reset_reason() != ESP_RST_POWERON && guard_rtc.magic == GUARD_RTC_MAGIC) {
        guard.restore(guard_rtc.rise_c[0], guard_rtc.rise_c[1]);
        Serial.printf("motor guard: resumed at %.1f / %.1f C\n", guard.rise_c(0), guard.rise_c(1));
    }
    guard_rtc.magic = GUARD_RTC_MAGIC;
    guard_rtc.rise_c[0] = guard.rise_c(0);
    guard_rtc.rise_c[1] = guard.rise_c(1);

    wifi.begin(HOSTNAME, WIFI_SSID, WIFI_PASSWORD, espnow_protocol::CHANNEL_OVERRIDE,
               ESPNOW_ENABLED);
    setup_ota();
    setup_telemetry_ring();
    diag_server.begin(&wifi, &telemetry_ring_buffer, esp_random(), &tuning_store, &esc_tuning);
    usb_bridge::begin();
    if (ESPNOW_ENABLED) espnow.begin(WIFI_IF_AP, espnow_protocol::CHANNEL_OVERRIDE, &tuning_store);

    while (millis() - esc_start_ms < esc_config::ESC_WAKE_MS) delay(5);
    stop_setup_frames();

    enableLoopWDT();

    prev_loop_us = micros();
    last_guard_us = prev_loop_us;
}

void loop() {
    uint32_t now_us = micros();
    uint32_t loop_us = now_us - prev_loop_us;
    prev_loop_us = now_us;

    // No firmware upload in tuning mode: the ESC pins belong to the bootloader link.
    if (!esc_tuning.owns_escs()) ArduinoOTA.handle();
    if (is_loading_firmware) {
        return;
    }

    bool radio_ok = crsf->update(radio_data);
    uint32_t now_ms = millis();

    vbat_sensor_ina238.update();
    float vbat = vbat_sensor_ina238.get_pack_volts();
    float ibat = vbat_sensor_ina238.get_pack_amps_discharge();
    update_motor_guard(micros());

    // An ESP-NOW firmware transfer owns the loop: motors stopped, no control, same as ArduinoOTA.
    // It only starts disarmed, and arming aborts it inside update(). Tuning mode counts as armed
    // here, so no transfer starts while the ESC pins belong to the bootloader link.
    if (ESPNOW_ENABLED) {
        espnow.update(now_ms, (radio_ok && radio_data->armed) || esc_tuning.owns_escs());
        if (espnow.ota_in_progress()) {
            stop_escs();
            return;
        }
    }
    pair_request_t pair_request;
    // Erasing the core dump writes flash, so it waits for the loop and refuses while armed.
    if (diag_server.take_erase_dump_request() && !(radio_ok && radio_data->armed)) {
        if (crash_report::erase_dump()) {
            boot_reset_report.has_dump = false;
            request_status_refresh();
        }
    }
    apply_tuning(radio_ok && radio_data->armed);
    if (ESPNOW_ENABLED && diag_server.take_pair_request(pair_request)) {
        const bool armed = radio_ok && radio_data->armed;
        if (pair_request.op == PairOp::PAIR) {
            espnow.pair(pair_request.mac, armed);
        } else {
            espnow.unpair(pair_request.mac, armed);
        }
        request_status_refresh();
    }
    esc_tuning.update(now_ms, radio_ok && radio_data->armed, last_espnow_driving,
                      radio_ok && radio_data->button_state);

    bool sensed_upside_down = accel->get_is_upside_down(false);
    updown_sensor::vector3_t *orientation = accel->get_orientation();
    crsf_bridge::telemetry_data_t telemetry = {
        .pack_volts = vbat,
        .pack_amps = ibat,
        .heading_deg = orientation->x,
        .roll_deg = orientation->y,
        .pitch_deg = orientation->z,
    };
    crsf->send_telemetry(&telemetry);
    publish_sensor_status(radio_ok && radio_data->armed);

    if (now_ms - last_led_ms >= 20) {
        last_led_ms = now_ms;
        if (esc_tuning.owns_escs()) {
            breathe_blue_led(now_ms);
        } else {
            cycle_rainbow_led(rainbow_tick, led_intensity);
            rainbow_tick = (rainbow_tick + 1) % 255;
        }
    }

    if (!radio_ok) {
        stop_escs();
        reset_steering(accel->get_orientation()->x);

        updown_sensor::vector3_t *av = accel->get();
        updown_sensor::vector3_t *ori = accel->get_orientation();
        diag_data_t diag = {
            .timestamp_ms = millis(),
            .radio_connected = false,
            .armed = false,
            .a_percent = 0,
            .b_percent = 0,
            .button_state = false,
            .flip_switch = 0,
            .left_cmd = 0,
            .right_cmd = 0,
            .accel_x = av ? av->x : 0,
            .accel_y = av ? av->y : 0,
            .accel_z = av ? av->z : 0,
            .is_upside_down = false,
            .loop_us = loop_us,
            .wifi_clients = wifi.station_count(),
            .orientation_x = ori ? ori->x : 0,
            .orientation_y = ori ? ori->y : 0,
            .orientation_z = ori ? ori->z : 0,
            .pid_setpoint = yaw_controller->setpoint(),
            .pid_output = steer_differential_pct,
            .pack_volts = vbat,
            .pack_amps_discharge = ibat,
            .yaw_rate_cw_dps = accel->get_yaw_rate_cw_dps(),
            .yaw_rate_cmd_cw_dps = yaw_loop_active ? yaw_controller->rate_command() : NAN,
        };
        publish_diag(diag, radio_data->autonomy_enabled, false, now_ms);
        return;
    }

    if (!radio_data->armed || esc_tuning.owns_escs()) {
        // Disarm is the only way out of a thermal trip. A link loss keeps it latched.
        if (!radio_data->armed) guard.clear_trip();
        stop_escs();
        reset_steering(accel->get_orientation()->x);

        updown_sensor::vector3_t *av = accel->get();
        updown_sensor::vector3_t *ori = accel->get_orientation();
        diag_data_t diag = {
            .timestamp_ms = millis(),
            .radio_connected = true,
            .armed = radio_data->armed,
            .a_percent = radio_data->a_percent,
            .b_percent = radio_data->b_percent,
            .button_state = radio_data->button_state,
            .flip_switch = (uint8_t)radio_data->flip_switch_state,
            .left_cmd = 0,
            .right_cmd = 0,
            .accel_x = av ? av->x : 0,
            .accel_y = av ? av->y : 0,
            .accel_z = av ? av->z : 0,
            .is_upside_down = false,
            .loop_us = loop_us,
            .wifi_clients = wifi.station_count(),
            .orientation_x = ori ? ori->x : 0,
            .orientation_y = ori ? ori->y : 0,
            .orientation_z = ori ? ori->z : 0,
            .pid_setpoint = yaw_controller->setpoint(),
            .pid_output = steer_differential_pct,
            .pack_volts = vbat,
            .pack_amps_discharge = ibat,
            .yaw_rate_cw_dps = accel->get_yaw_rate_cw_dps(),
            .yaw_rate_cmd_cw_dps = yaw_loop_active ? yaw_controller->rate_command() : NAN,
        };
        publish_diag(diag, radio_data->autonomy_enabled, false, now_ms);
        return;
    }

    // ESP-NOW command substitution. Runs only after every CRSF gate has passed, so ESP-NOW can
    // never arm the robot or extend its armed window. SA (ch5) down is the pilot's "accept
    // autonomous commands" position; SA up forces the stick values on ch0/ch1.
    bool espnow_active = false;
    if (ESPNOW_ENABLED && ESPNOW_COMMAND_SUBSTITUTION && radio_data->autonomy_enabled &&
        espnow.fresh(now_ms)) {
        const espnow_protocol::CommandPayload &cmd = espnow.latest();
        radio_data->a_percent = -100.0f * cmd.channel_a;
        radio_data->b_percent = 100.0f * cmd.channel_b;
        espnow_active = true;
    }

    set_led_intensity((abs(radio_data->a_percent) + abs(radio_data->b_percent)) / 2.0);

    bool is_upside_down;
    bool auto_steer_enabled = false;
    switch (radio_data->flip_switch_state) {
        case crsf_bridge::UP:
            is_upside_down = true;
            break;
        case crsf_bridge::MIDDLE:
            is_upside_down = false;
            break;
        case crsf_bridge::DOWN:
            auto_steer_enabled = true;
            is_upside_down = sensed_upside_down;
            break;
        default:
            is_upside_down = false;
            break;
    }

    if (is_upside_down) radio_data->a_percent *= -1;

    float sensed_angle_z = orientation->x;

    float left_command, right_command;
    mix_motor_outputs(radio_data, sensed_angle_z, auto_steer_enabled, is_upside_down, micros(),
                      left_command, right_command);

    // After the mixer, so it covers both the trainer and ESP-NOW command sources.
    guard.apply(left_command, right_command);
    esc::Esc::write_pair(*left_esc, left_command, *right_esc, right_command);
    last_left_scaled_duty = left_esc->duty(left_command);
    last_right_scaled_duty = right_esc->duty(right_command);

    updown_sensor::vector3_t *av = accel->get();
    diag_data_t diag = {
        .timestamp_ms = millis(),
        .radio_connected = true,
        .armed = true,
        .a_percent = radio_data->a_percent,
        .b_percent = radio_data->b_percent,
        .button_state = radio_data->button_state,
        .flip_switch = (uint8_t)radio_data->flip_switch_state,
        .left_cmd = left_command,
        .right_cmd = right_command,
        .accel_x = av ? av->x : 0,
        .accel_y = av ? av->y : 0,
        .accel_z = av ? av->z : 0,
        .is_upside_down = is_upside_down,
        .loop_us = loop_us,
        .wifi_clients = wifi.station_count(),
        .orientation_x = orientation ? orientation->x : 0,
        .orientation_y = orientation ? orientation->y : 0,
        .orientation_z = orientation ? orientation->z : 0,
        .pid_setpoint = yaw_controller->setpoint(),
        .pid_output = steer_differential_pct,
        .pack_volts = vbat,
        .pack_amps_discharge = ibat,
        .yaw_rate_cw_dps = accel->get_yaw_rate_cw_dps(),
        .yaw_rate_cmd_cw_dps = yaw_loop_active ? yaw_controller->rate_command() : NAN,
    };
    publish_diag(diag, radio_data->autonomy_enabled, espnow_active, now_ms);
}
