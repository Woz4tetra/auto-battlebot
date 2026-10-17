#pragma once

// Wire format for the ESP-NOW link between the Jetson, the USB dongle, and mr_stabs_mk2.
// See docs/plans/espnow_link_plan.md. Plain C++17 with no Arduino dependency: the robot firmware,
// the dongle firmware, the native unit tests, and the Jetson app all compile this one header, so
// the three ends cannot disagree about a field.
//
// Two hops:
//   Jetson <--serial frame--> dongle <--air frame--> robot
// Serial frame: magic:u16 (0xB07B, little endian), type:u8, length:u8, payload, crc16:u16 over
//               type, length, and payload.
// Air frame:    session magic:u8[4], type:u8, payload. Length is the ESP-NOW frame length.

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

// Nonzero pins both ends to this channel: the dongle skips its boot scan and the robot skips its
// sweep. Phase 1 of the plan runs pinned, since the robot's SoftAP still shares the radio and a
// sweep would drag it across channels. 0 selects the scan and the sweep.
const uint8_t CHANNEL_OVERRIDE = 1;

enum MessageType : uint8_t {
    MSG_COMMAND = 0x01,      // Jetson to robot, CommandPayload
    MSG_TELEMETRY = 0x02,    // robot to Jetson, TelemetryPayload
    MSG_TUNABLE_SET = 0x03,  // Jetson to robot, TunableSetPayload
    MSG_LINK_STATS = 0x04,   // dongle to Jetson, LinkStatsPayload
    MSG_PING = 0x05,         // dongle to robot, no payload: keepalive, see PING_PERIOD_MS
    MSG_OTA_BEGIN = 0x10,    // host to robot, OtaBeginPayload
    MSG_OTA_CHUNK = 0x11,    // host to robot, OtaChunkHeader then data
    MSG_OTA_END = 0x12,      // host to robot, no payload
    MSG_OTA_ABORT = 0x13,    // host to robot, no payload
    MSG_OTA_STATUS = 0x14,   // robot to host, OtaStatusPayload
};

#pragma pack(push, 1)
// The two values the Jetson's drive processor would write to the trainer channels, normalized to
// [-1, 1]: linear and angular under DifferentialDriveProcessor, left and right wheel under
// TankDriveProcessor. The firmware puts them where ch0 and ch1 land, so the mixing is whatever the
// trainer path would have done.
struct CommandPayload {
    uint32_t seq;
    uint32_t jetson_tx_us;
    float channel_a;  // ch0
    float channel_b;  // ch1
};

struct TelemetryPayload {
    uint32_t timestamp_ms;
    uint32_t loop_us;
    float a_percent;  // arbitrated drive command, after the flip
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
    uint8_t robot_known;  // 0 while the dongle broadcasts, 1 once it has learned the robot MAC
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
// Body of the robot's `GET /telemetry?from=<seq>&max=<n>` over WiFi: this header, then `count`
// TelemetryPayload records, oldest first. The firmware fills a ring of them at the ESP-NOW
// telemetry rate whether or not anyone reads it, so a client that loses the link catches up on
// everything still in the ring once it is back. telemetry_seq in each record is the ring's seq.
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

const uint32_t TELEMETRY_BATCH_MAGIC = 0x4D525442;  // "BTRM" little-endian
const uint16_t TELEMETRY_BATCH_VERSION = 1;
// Records per response at most. The client asks again at once when a response comes back full.
const uint32_t TELEMETRY_BATCH_MAX = 256;

const size_t OTA_CHUNK_DATA = 240;

// The dongle sends MSG_PING whenever the radio has been idle this long. Without it the dongle is
// silent unless the Jetson is commanding, and then the robot cannot sweep onto its channel, has
// nowhere to send telemetry, and cannot validate a freshly flashed image (rollback needs to hear a
// dongle). 10 ms puts two pings inside every 25 ms sweep dwell, at ~1% of the channel.
const uint32_t PING_PERIOD_MS = 10;

enum OtaState : uint8_t {
    OTA_IDLE = 0,
    OTA_RECEIVING = 1,
    OTA_DONE = 2,       // image accepted, rebooting
    OTA_VALIDATED = 3,  // sent after reboot once the new image marked itself valid
    OTA_ERROR = 4,
};

enum OtaError : uint8_t {
    OTA_OK = 0,
    OTA_ERR_ARMED = 1,
    OTA_ERR_BEGIN = 2,  // Update.begin refused: size does not fit the slot
    OTA_ERR_WRITE = 3,
    OTA_ERR_VERIFY = 4,   // MD5 or image check failed in Update.end
    OTA_ERR_ABORTED = 5,  // host abort, arming, or silence
    OTA_ERR_NOT_ACTIVE = 6,
};

static_assert(sizeof(CommandPayload) == 16, "CommandPayload layout");
static_assert(sizeof(LinkStatsPayload) == 18, "LinkStatsPayload layout");
static_assert(sizeof(OtaStatusPayload) == 22, "OtaStatusPayload layout");
static_assert(sizeof(TelemetryPayload) == 118, "TelemetryPayload layout");
static_assert(sizeof(TelemetryBatchHeader) == 32, "TelemetryBatchHeader layout");
static_assert(sizeof(TelemetryPayload) <= AIR_MAX_PAYLOAD, "telemetry must fit one air frame");
static_assert(sizeof(OtaChunkHeader) + OTA_CHUNK_DATA <= AIR_MAX_PAYLOAD,
              "OTA chunk must fit one air frame");

/** CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF. */
uint16_t crc16_ccitt(const uint8_t *data, size_t length);

/** Writes a serial frame into out, which must hold SERIAL_OVERHEAD + length bytes. Returns the
 *  frame size. */
size_t encode_serial_frame(uint8_t type, const uint8_t *payload, uint8_t length, uint8_t *out);

/** Writes an air frame into out, which must hold AIR_HEADER_SIZE + length bytes. Returns the
 *  frame size, or 0 when the payload does not fit AIR_MAX_PAYLOAD. */
size_t encode_air_frame(uint8_t type, const uint8_t *payload, size_t length, uint8_t *out);

/** True when frame starts with the session magic and has a type byte. */
bool is_air_frame(const uint8_t *frame, size_t length);

/**
 * Streaming serial frame parser. Feed bytes as they arrive, in any split. Every complete frame with
 * a valid CRC is handed to the handler once. Garbage and CRC failures are skipped by dropping one
 * byte and rescanning for the magic, so a corrupted frame cannot hide a good one behind it.
 */
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
