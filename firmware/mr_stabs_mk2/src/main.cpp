#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include <crash_report.h>
#include <crsf_bridge.h>
#include <diagnostics_server.h>
#include <esc.h>
#include <esp_attr.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <espnow_link.h>
#include <i2c_bus.h>
#include <motor_guard.h>
#include <telemetry_ring.h>
#include <updown_sensor.h>
#include <vbat_sensor.h>
#include <wifi_manager.h>
#include <yaw_control.h>

const char *WIFI_SSID = "MR-STABS";
const char *WIFI_PASSWORD = "havocbots";
// DHCP and mDNS name in client mode: http://mr-stabs.local
const char *HOSTNAME = "mr-stabs";
wifi_manager::WifiManager wifi;

crsf_bridge::CrsfBridge *crsf;
crsf_bridge::radio_data_t *radio_data;

#define LEFT_ESC_PIN ((gpio_num_t)A3)
#define RIGHT_ESC_PIN ((gpio_num_t)A2)

esc::Esc *left_esc;
esc::Esc *right_esc;

updown_sensor::UpdownSensor *accel;
vbat_sensor::VbatSensor vbat_sensor_ina238;

// Stall protection (docs/plans/motor_stall_protection_plan.md): scales both drive commands down
// when a motor's estimated heat says it is cooking. Updated once per INA238 sample, or every
// GUARD_PERIOD_US without one, armed or not, so disarming does not hand back a cold budget.
motor_guard::MotorGuard guard{motor_guard::Config()};
const uint32_t GUARD_PERIOD_US = 10000;
uint32_t last_guard_us = 0;
uint32_t last_guard_reads = 0;
// Duties last sent to the ESCs, after the guard's scaling. Every stop_escs() zeroes them.
float last_left_duty = 0.0f;
float last_right_duty = 0.0f;

// The thermal state survives a watchdog, panic, or software reset, so a reset mid-match resumes
// the estimate. A power-on starts cold. Whether it survives a brownout reset is untested.
struct guard_rtc_t {
    uint32_t magic;
    float rise_c[motor_guard::NUM_MOTORS];
};
const uint32_t GUARD_RTC_MAGIC = 0x4D475244;  // "DRGM"
RTC_NOINIT_ATTR guard_rtc_t guard_rtc;
DiagnosticsServer diag_server;

// Every 100 Hz telemetry frame, kept in PSRAM for GET /telemetry, so a WiFi client that drops
// out catches up instead of losing the gap. 9000 records of 155 B is 1.4 MB of the 2 MB PSRAM,
// 90 s. Without PSRAM it falls back to a few seconds in internal RAM.
const uint32_t TELEMETRY_RING_PSRAM_RECORDS = 9000;
const uint32_t TELEMETRY_RING_HEAP_RECORDS = 500;
telemetry_ring::TelemetryRing<espnow_protocol::TelemetryPayload> telemetry_ring_buffer;
uint32_t last_ring_push_ms = 0;
i2c_bus::scan_t bus_scan = {};
uint32_t last_status_ms = 0;
const uint32_t STATUS_INTERVAL_MS = 1000;  // sensor panel refresh on the diagnostics page
bool last_espnow_active = false;           // for the diagnostics page's ESP-NOW panel
crash_report::report_t boot_report = {};   // why this boot happened, for the page

const int NUM_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int rainbow_tick = 0, led_intensity = 20;
uint32_t last_led_ms = 0;

bool is_loading_firmware = false;
bool prev_button_state = false;
uint32_t prev_loop_us = 0;

// Steering in the flip-switch-DOWN mode: lib/yaw_control's cascaded heading and yaw-rate loop
// on the BNO055, run once per IMU sample. It replaces the heading-hold PID, which was off while
// the stick turned and so could not stop the tail-first spin. Without a usable IMU (no sample
// for HEADING_STALE_US, or the gyro failed its sign check) and while inverted, the turn stick
// passes straight through, as in MIDDLE.
yaw_control::YawController *yaw_controller;
float steer_output = 0.0f;  // differential percent: added to the left wheel, taken off the right
bool yaw_loop_active = false;
bool was_upside_down = false;
const float TURN_DEADBAND_PERCENT = 1.0f;  // passthrough steering, same as the yaw loop's
const uint32_t HEADING_STALE_US = 100000;
uint32_t last_heading_sample_us = 0;

// ESP-NOW link to the Jetson's dongle (docs/plans/espnow_link_plan.md).
espnow_link::EspNowLink espnow;

// Off means no ESP-NOW init, telemetry, or OTA; the SoftAP dashboard and scripts/ota_wifi are the
// paths in. It was off until the Mk2 PCBA because the old board's radio heard the dongle at
// -85 dBm from 30 cm. See "Robot radio" in the plan.
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

void set_led_intensity(float percent) {
    led_intensity = (int)(2.35 * min(100.0f, max(-100.0f, percent))) + 20;
}

void stop_escs() {
    esc::Esc::stop_pair(*left_esc, *right_esc);
    last_left_duty = 0.0f;
    last_right_duty = 0.0f;
    set_led_intensity(0);
}

// Publishes one tick of diagnostics to the WiFi dashboard and, at 100 Hz, to the telemetry ring
// and over ESP-NOW.
void publish_diag(diag_data_t &diag, bool autonomy_enabled, bool espnow_active, uint32_t now_ms) {
    diag.motor_rise_left = guard.rise_c(0);
    diag.motor_rise_right = guard.rise_c(1);
    diag.motor_speed_left = guard.speed_rad_s(0);
    diag.motor_speed_right = guard.speed_rad_s(1);
    diag.motor_amps_left = guard.amps_estimate(0);
    diag.motor_amps_right = guard.amps_estimate(1);
    diag.guard_scale = guard.scale();
    diag.guard_gain = guard.gain();
    diag.guard_state = (uint8_t)guard.state();
    diag.guard_sensors = (uint8_t)guard.sensors();
    diag.stall_flags = (guard.stalled(0) ? 1 : 0) | (guard.stalled(1) ? 2 : 0);
    diag.erpm_crc_errors =
        (uint16_t)min<uint32_t>(left_esc->crc_errors() + right_esc->crc_errors(), UINT16_MAX);
    diag_server.update(&diag);
    last_espnow_active = espnow_active;
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
    telemetry.vbat = diag.vbat;
    telemetry.ibat = diag.ibat;
    telemetry.yaw_rate = diag.yaw_rate;
    telemetry.yaw_rate_cmd = diag.yaw_rate_cmd;
    updown_sensor::vector3_t *linear_accel = accel->get_linear_accel();
    telemetry.linear_accel_x = linear_accel->x;
    telemetry.linear_accel_y = linear_accel->y;
    telemetry.linear_accel_z = linear_accel->z;
    updown_sensor::vector3_t *gyro = accel->get_gyro();
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
    telemetry.motor_rise_left = diag.motor_rise_left;
    telemetry.motor_rise_right = diag.motor_rise_right;
    telemetry.motor_speed_left = diag.motor_speed_left;
    telemetry.motor_speed_right = diag.motor_speed_right;
    telemetry.motor_amps_left = diag.motor_amps_left;
    telemetry.motor_amps_right = diag.motor_amps_right;
    telemetry.guard_scale = diag.guard_scale;
    telemetry.guard_gain = diag.guard_gain;
    telemetry.guard_state = diag.guard_state;
    telemetry.guard_sensors = diag.guard_sensors;
    telemetry.stall_flags = diag.stall_flags;
    telemetry.erpm_crc_errors = diag.erpm_crc_errors;
    if (ring_due) {
        // Step the schedule by exactly one period so the ring averages 100 Hz however the loop
        // ticks land. After a stall longer than a period, restart from now instead of bursting.
        last_ring_push_ms += espnow_link::TELEMETRY_PERIOD_MS;
        if (now_ms - last_ring_push_ms >= espnow_link::TELEMETRY_PERIOD_MS) {
            last_ring_push_ms = now_ms;
        }
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

// Feeds the diagnostics page's sensor and I2C panel. Does nothing with no browser open. A bus
// scan blocks for ~15 ms, so a requested scan waits until the robot is disarmed.
void publish_sensor_status(bool armed) {
    if (!diag_server.has_clients()) return;
    if (diag_server.scan_requested() && !armed) {
        i2c_bus::scan(&Wire1, &bus_scan);
        diag_server.clear_scan_request();
        last_status_ms = 0;
    }
    uint32_t now = millis();
    if (last_status_ms != 0 && now - last_status_ms < STATUS_INTERVAL_MS) return;
    last_status_ms = now;

    accel->refresh_details();
    sensor_status_t status = {};
    status.imu = accel->get_status();
    status.ina = vbat_sensor_ina238.get_status();
    status.scan = bus_scan;
    status.lines = i2c_bus::read_lines(SDA1, SCL1);
    status.sda_pin = SDA1;
    status.scl_pin = SCL1;
    status.snapshot_ms = millis();
    status.yaw_rate = accel->get_yaw_rate();
    status.heading_rate = accel->get_heading_rate();
    status.gyro_sign_suspect = accel->gyro_sign_suspect();
    if (ESPNOW_ENABLED) espnow.fill_status(status.espnow, status.snapshot_ms);
    status.espnow.enabled = ESPNOW_ENABLED;
    status.espnow.substitution = ESPNOW_COMMAND_SUBSTITUTION;
    status.espnow.driving = last_espnow_active;
    status.boot = boot_report;
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

// Feeds the guard one INA238 sample, or a NaN one after GUARD_PERIOD_US without a sample.
void update_motor_guard(uint32_t now_us) {
    const uint32_t reads = vbat_sensor_ina238.get_status().reads;
    const bool fresh = reads != last_guard_reads;
    if (!fresh && now_us - last_guard_us < GUARD_PERIOD_US) return;
    const float dt_s = (now_us - last_guard_us) / 1000000.0f;
    last_guard_us = now_us;
    last_guard_reads = reads;
    const float volts = fresh ? vbat_sensor_ina238.get_volts() : NAN;
    const float amps = fresh ? vbat_sensor_ina238.get_amps() : NAN;
    const float left_erpm = left_esc->erpm_valid() ? (float)left_esc->erpm() : -1.0f;
    const float right_erpm = right_esc->erpm_valid() ? (float)right_esc->erpm() : -1.0f;
    guard.update(dt_s, volts, amps, last_left_duty, last_right_duty, left_erpm, right_erpm);
    guard_rtc.rise_c[0] = guard.rise_c(0);
    guard_rtc.rise_c[1] = guard.rise_c(1);
}

void reset_steering(float sensed_angle_z) {
    yaw_controller->reset(sensed_angle_z);
    steer_output = 0.0f;
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
    // Capped so the first sample after a gap does not integrate the whole gap.
    float heading_dt = min(sample_us - last_heading_sample_us, HEADING_STALE_US) / 1000000.0f;
    if (fresh_heading) last_heading_sample_us = sample_us;
    bool heading_stale = sample_us == 0 || now_us - sample_us > HEADING_STALE_US;

    // The differential turns the robot the same way inverted (both the wheel sides and their
    // ground direction swap), and the yaw rate is projected on the measured up direction, so
    // the loop runs upside down too. A flip restarts it: the Euler heading swings while the
    // robot goes over.
    bool yaw_usable = auto_steer_enabled && !heading_stale && !accel->gyro_sign_suspect();
    if (!yaw_usable) {
        // The stick's resting trim (about 1% on this transmitter) would otherwise creep a wheel.
        steer_output = fabs(b_percent) > TURN_DEADBAND_PERCENT ? b_percent : 0.0f;
        yaw_loop_active = false;
    } else {
        if (!yaw_loop_active || is_upside_down != was_upside_down) {
            // Engaging (switch moved to DOWN, IMU back) or just flipped: hold this heading.
            yaw_controller->reset(sensed_angle_z);
            yaw_loop_active = true;
        }
        if (fresh_heading) {
            steer_output = yaw_controller->update(b_percent, tail_first_throttle, sensed_angle_z,
                                                  accel->get_yaw_rate(), heading_dt);
        }
    }
    was_upside_down = is_upside_down;

    left_command = -1 * a_percent + steer_output;
    right_command = -1 * a_percent - steer_output;
    float max_command = max(abs(left_command), abs(right_command));
    if (max_command > 100.0) {
        left_command = left_command / max_command * 100.0;
        right_command = right_command / max_command * 100.0;
    }
}

// Sends DShot stop frames every 5 ms from a timer while setup() runs, so the ESCs see an
// unbroken signal from the moment their pins are driven until loop() takes over. The IMU, WiFi,
// and ESP-NOW setup between those points takes about 2.5 s.
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
    Serial.begin(115200);
    crash_report::read(boot_report);
    crash_report::print(boot_report);

#if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

    // The RX channels read each ESC's eRPM reply back on the same pin (bidirectional DShot).
    left_esc = new esc::Esc(LEFT_ESC_PIN, RMT_CHANNEL_3, RMT_CHANNEL_4);
    right_esc = new esc::Esc(RIGHT_ESC_PIN, RMT_CHANNEL_2, RMT_CHANNEL_5);
    left_esc->stop_threshold = 1.0f;
    right_esc->stop_threshold = 1.0f;
    left_esc->begin();
    right_esc->begin();
    start_setup_frames();
    const uint32_t esc_start_ms = millis();

    pixels.begin();
    pixels.setBrightness(20);

    for (int count = 0; count < 2; count++) pulse_led();

    Wire1.begin();  // BNO055 IMU lives on the Wire1 I2C bus
    // Stays at the default 100 kHz. The BNO055 stretches the clock, which can fail at 400 kHz,
    // and every failed read blocks the loop for the 50 ms Wire timeout, starving the ESCs of
    // DShot frames.
    // Recorded for the diagnostics page: shows which devices answered at boot.
    i2c_bus::scan(&Wire1, &bus_scan);
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

    yaw_controller = new yaw_control::YawController(yaw_control::Config());

    if (esp_reset_reason() != ESP_RST_POWERON && guard_rtc.magic == GUARD_RTC_MAGIC) {
        guard.restore(guard_rtc.rise_c[0], guard_rtc.rise_c[1]);
        Serial.printf("motor guard: resumed at %.1f / %.1f C\n", guard.rise_c(0), guard.rise_c(1));
    }
    guard_rtc.magic = GUARD_RTC_MAGIC;
    guard_rtc.rise_c[0] = guard.rise_c(0);
    guard_rtc.rise_c[1] = guard.rise_c(1);

    // Access point or router, as saved from the diagnostics page. With ESP-NOW on, the access
    // point stays up on the pinned channel ESP-NOW shares, whatever is saved.
    wifi.begin(HOSTNAME, WIFI_SSID, WIFI_PASSWORD, espnow_protocol::CHANNEL_OVERRIDE,
               ESPNOW_ENABLED);
    setup_ota();
    tunable_ptrs_t tunables;
    tunables.left_esc_deadzone = &left_esc->stop_threshold;
    tunables.right_esc_deadzone = &right_esc->stop_threshold;
    setup_telemetry_ring();
    diag_server.begin(&wifi, &telemetry_ring_buffer, esp_random(), tunables);
    if (ESPNOW_ENABLED) espnow.begin(WIFI_IF_AP, espnow_protocol::CHANNEL_OVERRIDE, tunables);

    // ESCs need continuous DShot zero-throttle frames to initialize (~2 seconds). The timer has
    // sent them since the ESC pins came up; top that up to 2 s, then hand over to loop(). Before
    // the timer, the 2.4 s of IMU and WiFi setup left the line idle, and a reset of the ESP32 alone
    // made the running ESCs play their wake-up jingle twice. A pack power-on still plays it twice:
    // the ESCs sample the line before any firmware can drive it (measured 212 ms after reset).
    while (millis() - esc_start_ms < 2000) delay(5);
    stop_setup_frames();

    // A loop() pass that takes over CONFIG_ESP_TASK_WDT_TIMEOUT_S (5 s) panics: the core dump goes
    // to flash and the robot reboots, instead of hanging with the ESCs on their last command. The
    // core already watches core 0's idle task. Core 1's idle task is not watched: loop() spins on
    // core 1 without yielding, so it never runs. Started last: setup() blocks longer than that.
    enableLoopWDT();

    prev_loop_us = micros();
    last_guard_us = prev_loop_us;
}

void loop() {
    uint32_t now_us = micros();
    uint32_t loop_us = now_us - prev_loop_us;
    prev_loop_us = now_us;

    ArduinoOTA.handle();
    if (is_loading_firmware) {
        return;
    }

    bool radio_ok = crsf->update(radio_data);
    uint32_t now_ms = millis();

    // Before the ESP-NOW OTA return, so the guard's model keeps cooling during a transfer.
    vbat_sensor_ina238.update();
    float vbat = vbat_sensor_ina238.get_volts();
    float ibat = vbat_sensor_ina238.get_amps();
    update_motor_guard(micros());

    // An ESP-NOW firmware transfer owns the loop: motors stopped, no control, same as ArduinoOTA.
    // It only starts disarmed, and arming aborts it inside update().
    if (ESPNOW_ENABLED) {
        espnow.update(now_ms, radio_ok && radio_data->armed);
        if (espnow.ota_in_progress()) {
            stop_escs();
            return;
        }
    }
    // A dongle pair or unpair from the diagnostics page. Refused while armed inside pair().
    pair_request_t pair_request;
    // Erasing the core dump writes flash, so it waits for the loop and refuses while armed.
    if (diag_server.take_erase_dump_request() && !(radio_ok && radio_data->armed)) {
        if (crash_report::erase_dump()) {
            boot_report.has_dump = false;
            last_status_ms = 0;
        }
    }
    if (ESPNOW_ENABLED && diag_server.take_pair_request(pair_request)) {
        const bool armed = radio_ok && radio_data->armed;
        if (pair_request.op == 1) {
            espnow.pair(pair_request.mac, armed);
        } else {
            espnow.unpair(pair_request.mac, armed);
        }
        last_status_ms = 0;  // show the result on the next status poll
    }

    // Read the IMU every loop whatever the radio is doing, so heading stays fresh for the PID and
    // the diagnostics page and telemetry track the robot with no handset. Rate-limited inside the
    // sensor. Reconnect stays off: it blocks the loop for over a second.
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

    // Combat mode
    if (now_ms - last_led_ms >= 20) {
        last_led_ms = now_ms;
        cycle_rainbow_led(rainbow_tick, led_intensity);
        rainbow_tick = (rainbow_tick + 1) % 255;
    }

    // The only link failsafe: isLinkUp() drops 300 ms after the last channel packet. That catches
    // a lost handset only while the receiver's failsafe is set to send nothing; a receiver set to
    // hold its last frame keeps the robot armed on stale values.
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
            .pid_output = steer_output,
            .vbat = vbat,
            .ibat = ibat,
            .yaw_rate = accel->get_yaw_rate(),
            .yaw_rate_cmd = yaw_loop_active ? yaw_controller->rate_command() : NAN,
        };
        publish_diag(diag, radio_data->autonomy_enabled, false, now_ms);
        return;
    }

    if (!radio_data->armed) {
        stop_escs();
        reset_steering(accel->get_orientation()->x);

        updown_sensor::vector3_t *av = accel->get();
        updown_sensor::vector3_t *ori = accel->get_orientation();
        diag_data_t diag = {
            .timestamp_ms = millis(),
            .radio_connected = true,
            .armed = false,
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
            .pid_output = steer_output,
            .vbat = vbat,
            .ibat = ibat,
            .yaw_rate = accel->get_yaw_rate(),
            .yaw_rate_cmd = yaw_loop_active ? yaw_controller->rate_command() : NAN,
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
        // Same place the trainer path lands: the Jetson's two channel values go into ch0/ch1 at
        // full scale, and crsf_bridge negates ch0 into a_percent.
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
        case crsf_bridge::DOWN:  // this is the default position when the transmitter powers on
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
    last_left_duty = left_esc->duty(left_command);
    last_right_duty = right_esc->duty(right_command);

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
        .pid_output = steer_output,
        .vbat = vbat,
        .ibat = ibat,
        .yaw_rate = accel->get_yaw_rate(),
        .yaw_rate_cmd = yaw_loop_active ? yaw_controller->rate_command() : NAN,
    };
    publish_diag(diag, radio_data->autonomy_enabled, espnow_active, now_ms);
}
