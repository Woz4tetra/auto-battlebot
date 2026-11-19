#pragma once
#include <cstddef>
#include <cstdint>

namespace blheli_boot {

const uint8_t REQUEST_START = 0x2F;
const uint8_t RESPONSE_START = 0x2E;
const uint8_t WRITE_TRIES = 3;
const uint8_t INIT_TRIES = 3;
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
    ACK_INVALID_PARAM = 0x09,
    ACK_GENERAL_ERROR = 0x0F,
};

const uint8_t MODE_ARM_BLB = 0x04;
const uint8_t PROTOCOL_VERSION = 108;
const char INTERFACE_NAME[] = "MrStabsMk2";

const uint16_t SETTINGS_ADDRESS = 0xF800;
const uint16_t SETTINGS_SIZE = 256;
bool write_allowed(uint16_t address, size_t length);

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
    const uint8_t *params_in_frame;
    size_t length;
};

enum class ParseResult : uint8_t { OK, SHORT, BAD_START, BAD_LENGTH, BAD_CRC };
ParseResult parse_request(const uint8_t *frame, size_t size, Request &out);
const char *parse_result_name(ParseResult result);

size_t build_response(uint8_t cmd, uint16_t address, const uint8_t *params, size_t length,
                      uint8_t ack, uint8_t *out);

class EscLink {
   public:
    virtual ~EscLink() = default;
    virtual uint8_t count() const = 0;
    virtual void select(uint8_t esc) = 0;
    virtual void send(const uint8_t *data, size_t size) = 0;
    virtual size_t receive(uint8_t *buf, size_t max, size_t expect, uint32_t first_ms) = 0;
    virtual void hold_low(uint32_t ms) = 0;
    virtual void delay_ms(uint32_t ms) = 0;
};

class Server {
   public:
    explicit Server(EscLink &link) : _link(link) {}
    size_t handle(const uint8_t *frame, size_t size, uint8_t *out);
    void reset_esc(uint8_t esc);
    uint32_t guard_refusals_since_boot() const { return _guard_refusals; }
    const char *failure() const { return _failure; }
    const char *init_reply() const { return _init_reply; }
    uint32_t handshake_resyncs() const { return _resyncs; }
    const char *first_read_failure() const { return _first_read_failure; }
    uint32_t handshake_restarts() const { return _reboots; }

   private:
    EscLink &_link;
    uint32_t _guard_refusals = 0;
    uint8_t _rx[MAX_RESPONSE];
    char _failure[96] = "";
    char _init_reply[96] = "";
    char _first_read_failure[96] = "";
    int8_t _handshake_esc = -1;
    uint32_t _resyncs = 0;
    uint32_t _reboots = 0;

    void describe(char *out, size_t size, const char *step, size_t received, size_t expected) const;
    uint8_t fail(const char *step, size_t received, size_t expected);

    void send_boot(const uint8_t *data, size_t size);
    bool boot_ack(uint32_t first_ms, const char *step);
    bool set_address(uint16_t address, uint8_t tries);
    bool handshake(uint8_t *signature);
    uint8_t init_flash(uint8_t esc, uint8_t *signature);
    uint8_t read(uint16_t address, size_t length, uint8_t *data);
    uint8_t read_once(uint16_t address, size_t length, uint8_t *data);
    uint8_t write(uint16_t address, const uint8_t *data, size_t length);
};

}  // namespace blheli_boot
