#pragma once
#include <cstddef>
#include <cstdint>

// BLHeli 4-way interface server and BLHeli_32 bootloader client. Tuning mode relays 4-way
// request frames from HTTP (the diagnostics page, or firmware/blheli/blheli32.py --url) to the
// ESC signal wires. Ported from firmware/BlHeli-Passthrough/ESP32/4Way.cpp and ESC_Serial.cpp,
// minus MSP, with reads that end on the expected length instead of fixed delays, and with an
// address guard. Protocol and settings layout: firmware/blheli/README.md.
namespace blheli_boot {

// 4-way framing: request 2F cmd addr_hi addr_lo n params[n] crc_hi crc_lo, response
// 2E cmd addr_hi addr_lo n params[n] ack crc_hi crc_lo. n = 0 means 256. CRC16-XMODEM.
const uint8_t REQUEST_START = 0x2F;
const uint8_t RESPONSE_START = 0x2E;
const size_t MAX_REQUEST = 5 + 256 + 2;
const size_t MAX_RESPONSE = 5 + 256 + 3;

enum Command : uint8_t {
    TEST_ALIVE = 0x30,
    PROTOCOL_GET_VERSION = 0x31,
    INTERFACE_GET_NAME = 0x32,
    INTERFACE_GET_VERSION = 0x33,
    INTERFACE_EXIT = 0x34,
    DEVICE_RESET = 0x35,
    DEVICE_INIT_FLASH = 0x37,
    DEVICE_ERASE_ALL = 0x38,
    DEVICE_PAGE_ERASE = 0x39,
    DEVICE_READ = 0x3A,
    DEVICE_WRITE = 0x3B,
    INTERFACE_SET_MODE = 0x3F,
};

enum Ack : uint8_t {
    ACK_OK = 0x00,
    ACK_INVALID_CMD = 0x02,
    ACK_INVALID_CRC = 0x03,
    ACK_INVALID_CHANNEL = 0x08,
    ACK_INVALID_PARAM = 0x09,  // also what the address guard answers
    ACK_GENERAL_ERROR = 0x0F,
};

const uint8_t MODE_ARM_BLB = 0x04;
const uint8_t PROTOCOL_VERSION = 108;
const char INTERFACE_NAME[] = "MrStabsMk2";

// The settings page is the only flash a write may touch. BLHeliSuite's Write Setup is one
// 256-byte DeviceWrite here and no page erase, so anything else is refused: a bad page can
// corrupt settings (restored from a backup) but never the ESC's application or bootloader.
const uint16_t SETTINGS_ADDRESS = 0xF800;
const uint16_t SETTINGS_SIZE = 256;
bool write_allowed(uint16_t address, size_t length);

// Bootloader link: 19200 8N1 half duplex on the signal wire, CRC16 poly 0xA001 (reflected,
// init 0), sent low byte first. Replies end in a result byte.
const uint8_t BOOT_SUCCESS = 0x30;
const uint8_t BOOT_RESTART = 0x00;
const uint8_t BOOT_PROG_FLASH = 0x01;
const uint8_t BOOT_READ_FLASH = 0x03;
const uint8_t BOOT_KEEP_ALIVE = 0xFD;
const uint8_t BOOT_SET_BUFFER = 0xFE;
const uint8_t BOOT_SET_ADDRESS = 0xFF;

uint16_t crc_xmodem(const uint8_t *data, size_t size, uint16_t crc = 0);
uint16_t crc_boot(const uint8_t *data, size_t size, uint16_t crc = 0);

struct Request {
    uint8_t cmd;
    uint16_t address;
    const uint8_t *params;  // points into the frame
    size_t length;          // 1..256
};

enum class ParseResult : uint8_t { OK, SHORT, BAD_START, BAD_LENGTH, BAD_CRC };
ParseResult parse_request(const uint8_t *frame, size_t size, Request &out);
const char *parse_result_name(ParseResult result);

// Writes a response frame into out (MAX_RESPONSE bytes) and returns its size. length is 1..256.
size_t build_response(uint8_t cmd, uint16_t address, const uint8_t *params, size_t length,
                      uint8_t ack, uint8_t *out);

// The one-wire serial link to the ESCs, by passthrough index (0 = A2, 1 = A3).
class EscLink {
   public:
    virtual ~EscLink() = default;
    virtual uint8_t count() const = 0;
    virtual void select(uint8_t esc) = 0;
    virtual void send(const uint8_t *data, size_t size) = 0;
    // Reads until expect bytes have arrived (0: no length known), a 10 ms gap after the first
    // byte, max bytes, or nothing at all for first_ms. Returns the count read.
    virtual size_t receive(uint8_t *buf, size_t max, size_t expect, uint32_t first_ms) = 0;
    // Drives the selected line low for ms, then high.
    virtual void hold_low(uint32_t ms) = 0;
    virtual void delay_ms(uint32_t ms) = 0;
};

class Server {
   public:
    explicit Server(EscLink &link) : _link(link) {}
    // One request frame in, one response frame out. Returns the response size, 0 when the frame
    // is too short to answer.
    size_t handle(const uint8_t *frame, size_t size, uint8_t *out);
    // DeviceReset outside a frame: restarts the ESC, which leaves its bootloader.
    void reset_esc(uint8_t esc);
    // Writes and erases the guard refused, from boot.
    uint32_t guard_refusals() const { return _guard_refusals; }

   private:
    EscLink &_link;
    uint32_t _guard_refusals = 0;
    uint8_t _rx[MAX_RESPONSE];

    void send_boot(const uint8_t *data, size_t size);
    bool boot_ack(uint32_t first_ms);
    bool set_address(uint16_t address, uint8_t tries);
    uint8_t init_flash(uint8_t esc, uint8_t *signature);
    uint8_t read(uint16_t address, size_t length, uint8_t *data);
    uint8_t write(uint16_t address, const uint8_t *data, size_t length);
};

}  // namespace blheli_boot
