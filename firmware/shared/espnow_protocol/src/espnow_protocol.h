#pragma once

#include <stddef.h>
#include <stdint.h>

namespace espnow_protocol {

const uint16_t SERIAL_MAGIC = 0xB07B;
const uint8_t AIR_MAGIC[4] = {'B', 'W', 'B', 0x01};
const size_t AIR_HEADER_SIZE = sizeof(AIR_MAGIC) + 1;
// ESP_NOW_MAX_DATA_LEN on IDF 4.4. ESP-NOW v2 needs IDF 5.4.
const size_t AIR_MAX_FRAME = 250;
const size_t AIR_MAX_PAYLOAD = AIR_MAX_FRAME - AIR_HEADER_SIZE;
const size_t SERIAL_OVERHEAD = 6;  // magic, type, length, crc
const size_t SERIAL_MAX_FRAME = SERIAL_OVERHEAD + 255;

const uint8_t CHANNEL_OVERRIDE = 1;

enum MessageType : uint8_t {
    MSG_COMMAND = 0x01,
    MSG_TELEMETRY = 0x02,
    MSG_TUNABLE_SET = 0x03,
    MSG_LINK_STATS = 0x04,
    MSG_PING = 0x05,
    MSG_OTA_BEGIN = 0x10,
    MSG_OTA_CHUNK = 0x11,
    MSG_OTA_END = 0x12,
    MSG_OTA_ABORT = 0x13,
    MSG_OTA_STATUS = 0x14,
};

#pragma pack(push, 1)
struct CommandPayload {
    uint32_t seq;
    uint32_t jetson_tx_us;
    float channel_a;  // ch0
    float channel_b;  // ch1
};

struct TelemetryPayload {
    uint32_t timestamp_ms;
    uint32_t loop_us;
    float a_percent;
    float b_percent;
    float left_cmd;
    float right_cmd;
    float accel_x, accel_y, accel_z;
    float orientation_x, orientation_y, orientation_z;
    float pid_setpoint;
    float pid_output;
    float vbat;          // pack volts, NaN when the INA228 is absent
    float ibat;          // pack amps, positive discharging, NaN when absent
    float yaw_rate;      // deg/s clockwise
    float yaw_rate_cmd;  // deg/s clockwise, NaN when the yaw loop is not running
    // BNO055 fused linear acceleration, gravity removed, m/s^2 in the chip frame. accel_x..z
    // above are the fused gravity vector.
    float linear_accel_x, linear_accel_y, linear_accel_z;
    float gyro_x, gyro_y;     // deg/s about the chip's x and y axes, right-handed
    uint8_t imu_calibration;  // BNO055 CALIB_STAT: sys, gyro, accel, mag, two bits each, 0-3
    uint8_t radio_connected;
    uint8_t armed;
    uint8_t autonomy_enabled;  // ch5 (SA) as the robot saw it; gates espnow_active
    uint8_t button_state;
    uint8_t flip_switch;
    uint8_t is_upside_down;
    uint8_t espnow_active;   // ESP-NOW supplied this tick's a/b percent
    uint8_t espnow_channel;  // channel the robot is listening on, 0 while sweeping
    uint32_t espnow_age_ms;  // since the last accepted ESP-NOW command frame
    uint32_t espnow_seq;     // last accepted sequence number
    uint32_t jetson_tx_us;   // echoed from that command, for round-trip measurement
    uint8_t espnow_lq;       // telemetry send ACK success percent over a sliding window
    uint32_t telemetry_seq;  // +1 per telemetry frame sent, so the host can count drops
    // Motor stall guard (lib/motor_guard), docs/plans/motor_stall_protection_plan.md.
    float motor_rise_left, motor_rise_right;    // C above ambient, thermal model
    float motor_speed_left, motor_speed_right;  // rad/s at the motor shaft, signed
    float motor_amps_left, motor_amps_right;    // estimated winding current I_hat
    float guard_scale;                          // 1 passes commands through, below 1 limiting
    float guard_gain;                           // shunt correction g on the model current
    uint8_t guard_state;                        // motor_guard::State
    uint8_t guard_sensors;                      // motor_guard::Sensors
    uint8_t stall_flags;                        // bit 0 left, bit 1 right
    uint16_t erpm_crc_errors;                   // both ESCs summed, from boot
};

struct TunableSetPayload {
    uint8_t id;
    float value;
};

enum TunableId : uint8_t {
    TUNABLE_LEFT_ESC_DEADZONE = 0,
    TUNABLE_RIGHT_ESC_DEADZONE = 1,
};

struct LinkStatsPayload {
    uint32_t sent;
    uint32_t acked;
    uint32_t failed;
    uint32_t last_rtt_us;  // esp_now_send to send callback for the newest unicast frame
    uint8_t channel;
    uint8_t robot_known;
};

struct OtaBeginPayload {
    uint32_t size;
    uint8_t md5[16];
};

struct OtaChunkHeader {
    uint32_t offset;
};

struct OtaStatusPayload {
    uint8_t state;  // OtaState
    uint32_t next_offset;
    uint8_t error;  // OtaError
    uint8_t running_md5[16];
};
struct TelemetryBatchHeader {
    uint32_t magic;        // TELEMETRY_BATCH_MAGIC
    uint16_t version;      // TELEMETRY_BATCH_VERSION
    uint16_t record_size;  // sizeof(TelemetryPayload); a mismatch means mismatched builds
    uint32_t boot_id;      // random per boot: a change means seqs restarted at 0
    uint32_t capacity;     // records the ring holds
    uint32_t head;         // seq the next record gets; head - 1 is the newest
    uint32_t oldest;       // oldest seq still in the ring
    uint32_t first_seq;    // seq of the first record in this body
    uint32_t count;
};

#pragma pack(pop)

const uint32_t TELEMETRY_BATCH_MAGIC = 0x4D525442;
const uint16_t TELEMETRY_BATCH_VERSION = 2;
// Records per response at most. The client asks again at once when a response comes back full.
const uint32_t TELEMETRY_BATCH_MAX = 256;

const size_t OTA_CHUNK_DATA = 240;

const uint32_t PING_PERIOD_MS = 10;

enum OtaState : uint8_t {
    OTA_IDLE = 0,
    OTA_RECEIVING = 1,
    OTA_DONE = 2,
    OTA_VALIDATED = 3,
    OTA_ERROR = 4,
};

enum OtaError : uint8_t {
    OTA_OK = 0,
    OTA_ERR_ARMED = 1,
    OTA_ERR_BEGIN = 2,
    OTA_ERR_WRITE = 3,
    OTA_ERR_VERIFY = 4,
    OTA_ERR_ABORTED = 5,
    OTA_ERR_NOT_ACTIVE = 6,
};

static_assert(sizeof(CommandPayload) == 16, "CommandPayload layout");
static_assert(sizeof(LinkStatsPayload) == 18, "LinkStatsPayload layout");
static_assert(sizeof(OtaStatusPayload) == 22, "OtaStatusPayload layout");
static_assert(sizeof(TelemetryPayload) == 155, "TelemetryPayload layout");
static_assert(sizeof(TelemetryBatchHeader) == 32, "TelemetryBatchHeader layout");
static_assert(sizeof(TelemetryPayload) <= AIR_MAX_PAYLOAD, "telemetry must fit one air frame");
static_assert(sizeof(OtaChunkHeader) + OTA_CHUNK_DATA <= AIR_MAX_PAYLOAD,
              "OTA chunk must fit one air frame");

uint16_t crc16_ccitt(const uint8_t *data, size_t length);

/** Writes a serial frame into out, which must hold SERIAL_OVERHEAD + length bytes. Returns the
 *  frame size. */
size_t encode_serial_frame(uint8_t type, const uint8_t *payload, uint8_t length, uint8_t *out);

/** Writes an air frame into out, which must hold AIR_HEADER_SIZE + length bytes. Returns the
 *  frame size, or 0 when the payload does not fit AIR_MAX_PAYLOAD. */
size_t encode_air_frame(uint8_t type, const uint8_t *payload, size_t length, uint8_t *out);

bool is_air_frame(const uint8_t *frame, size_t length);

class SerialFrameParser {
   public:
    using Handler = void (*)(void *context, uint8_t type, const uint8_t *payload, uint8_t length);

    void feed(const uint8_t *data, size_t count, Handler handler, void *context);
    uint32_t crc_errors() const { return crc_errors_; }

   private:
    void drop(size_t count);

    uint8_t buffer_[SERIAL_MAX_FRAME];
    size_t size_ = 0;
    uint32_t crc_errors_ = 0;
};

}  // namespace espnow_protocol
