#pragma once

#include <stddef.h>
#include <stdint.h>

namespace espnow_protocol {

const uint16_t SERIAL_MAGIC = 0xB07B;
const uint8_t AIR_MAGIC[4] = {'B', 'W', 'B', 0x01};
const size_t AIR_HEADER_SIZE = sizeof(AIR_MAGIC) + 1;
const size_t AIR_MAX_FRAME = 250;
const size_t AIR_MAX_PAYLOAD = AIR_MAX_FRAME - AIR_HEADER_SIZE;
const size_t SERIAL_OVERHEAD = 6;
static_assert(SERIAL_OVERHEAD == sizeof(SERIAL_MAGIC) + 1 + 1 + 2,
              "serial frame is magic, type, length, crc");
const size_t SERIAL_MAX_FRAME = SERIAL_OVERHEAD + 255;

const uint8_t CHANNEL_OVERRIDE = 1;

const uint16_t PROTOCOL_VERSION = 1;
const size_t FIRMWARE_BUILD_LENGTH = 16;

enum MessageType : uint8_t {
    MSG_COMMAND = 0x01,
    MSG_TELEMETRY = 0x02,
    MSG_TUNABLE_SET = 0x03,
    MSG_LINK_STATS = 0x04,
    MSG_PING = 0x05,
    MSG_FIRMWARE_INFO = 0x06,
    MSG_OTA_BEGIN = 0x10,
    MSG_OTA_CHUNK = 0x11,
    MSG_OTA_END = 0x12,
    MSG_OTA_ABORT = 0x13,
    MSG_OTA_STATUS = 0x14,
};

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

#pragma pack(push, 1)
struct CommandPayload {
    uint32_t seq;
    uint32_t jetson_tx_us;
    float ch0_value;
    float ch1_value;
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
    float pack_volts;
    float pack_amps_discharge;
    float yaw_rate_cw_dps;
    float yaw_rate_cmd_cw_dps;
    float linear_accel_x_mps2, linear_accel_y_mps2, linear_accel_z_mps2;
    float gyro_x_dps, gyro_y_dps;
    uint8_t imu_calib_stat;
    uint8_t radio_connected;
    uint8_t armed;
    uint8_t autonomy_switch_sa;
    uint8_t button_state;
    uint8_t flip_switch;
    uint8_t is_upside_down;
    uint8_t espnow_driving;
    uint8_t espnow_channel;
    uint32_t espnow_age_ms;
    uint32_t espnow_seq;
    uint32_t jetson_tx_us;
    uint8_t espnow_lq;
    uint32_t telemetry_seq;
    float motor_rise_left_c, motor_rise_right_c;
    float motor_speed_left_rad_s, motor_speed_right_rad_s;
    float motor_amps_est_left, motor_amps_est_right;
    float guard_scale;
    float guard_shunt_gain;
    uint8_t guard_state;
    uint8_t guard_sensors;
    uint8_t stall_flags;
    uint16_t erpm_crc_errors;
};

struct FirmwareInfoPayload {
    uint16_t protocol_version;
    char build[FIRMWARE_BUILD_LENGTH];
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
    uint32_t last_unicast_rtt_us;
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
    OtaState state;
    uint32_t next_offset;
    OtaError error;
    uint8_t running_md5[16];
};

#pragma pack(pop)

const size_t OTA_CHUNK_DATA = 240;

const uint32_t PING_PERIOD_MS = 10;
const uint32_t FIRMWARE_INFO_PERIOD_MS = 1000;

static_assert(sizeof(CommandPayload) == 16, "CommandPayload layout");
static_assert(sizeof(LinkStatsPayload) == 18, "LinkStatsPayload layout");
static_assert(sizeof(OtaStatusPayload) == 22, "OtaStatusPayload layout");
static_assert(sizeof(TelemetryPayload) == 155, "TelemetryPayload layout");
static_assert(sizeof(FirmwareInfoPayload) == 18, "FirmwareInfoPayload layout");
static_assert(sizeof(TelemetryPayload) <= AIR_MAX_PAYLOAD, "telemetry must fit one air frame");
static_assert(sizeof(OtaChunkHeader) + OTA_CHUNK_DATA <= AIR_MAX_PAYLOAD,
              "OTA chunk must fit one air frame");

uint16_t crc16_ccitt(const uint8_t *data, size_t length);

size_t encode_serial_frame(uint8_t type, const uint8_t *payload, uint8_t length,
                           uint8_t (&out)[SERIAL_MAX_FRAME]);

size_t encode_air_frame(uint8_t type, const uint8_t *payload, size_t length,
                        uint8_t (&out)[AIR_MAX_FRAME]);

bool is_air_frame(const uint8_t *frame, size_t length);

FirmwareInfoPayload make_firmware_info(const char *build);

class SerialFrameParser {
   public:
    using Handler = void (*)(void *context, uint8_t type, const uint8_t *payload, uint8_t length);

    void feed(const uint8_t *data, size_t count, Handler handler, void *context);
    uint32_t crc_errors() const { return crc_errors_; }
    size_t buffered() const { return size_; }

   private:
    void drop(size_t count);

    uint8_t buffer_[SERIAL_MAX_FRAME];
    size_t size_ = 0;
    uint32_t crc_errors_ = 0;
};

inline bool ota_should_abort(bool active, bool armed, uint32_t now_ms, uint32_t last_frame_ms,
                             uint32_t silence_ms) {
    return active && (armed || now_ms - last_frame_ms > silence_ms);
}

}  // namespace espnow_protocol
