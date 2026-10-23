#include "blheli_boot.h"

#include <cstring>

namespace blheli_boot {

// The passthrough's DeviceInitFlash handshake. It carries its own CRC, so it goes out raw.
static const uint8_t BOOT_INIT[] = {0,   0,   0,   0,   0,   0,   0,    0,   0x0D,
                                    'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
// The passthrough's InterfaceGetVersion, 20.0.05.
static const uint8_t INTERFACE_VERSION[] = {200, 5};
// The passthrough's retry count for the steps of a write.
static const uint8_t WRITE_TRIES = 3;

bool write_allowed(uint16_t address, size_t length) {
    return address == SETTINGS_ADDRESS && length == SETTINGS_SIZE;
}

uint16_t crc_xmodem(const uint8_t *data, size_t size, uint16_t crc) {
    for (size_t i = 0; i < size; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

uint16_t crc_boot(const uint8_t *data, size_t size, uint16_t crc) {
    for (size_t i = 0; i < size; i++) {
        uint8_t byte = data[i];
        for (int bit = 0; bit < 8; bit++) {
            const bool flip = ((byte ^ crc) & 1) != 0;
            crc >>= 1;
            if (flip) crc ^= 0xA001;
            byte >>= 1;
        }
    }
    return crc;
}

ParseResult parse_request(const uint8_t *frame, size_t size, Request &out) {
    if (size < 8) return ParseResult::SHORT;
    if (frame[0] != REQUEST_START) return ParseResult::BAD_START;
    const size_t length = frame[4] == 0 ? 256 : frame[4];
    if (size != 5 + length + 2) return ParseResult::BAD_LENGTH;
    const uint16_t crc = static_cast<uint16_t>((frame[size - 2] << 8) | frame[size - 1]);
    if (crc_xmodem(frame, size - 2) != crc) return ParseResult::BAD_CRC;
    out.cmd = frame[1];
    out.address = static_cast<uint16_t>((frame[2] << 8) | frame[3]);
    out.params = frame + 5;
    out.length = length;
    return ParseResult::OK;
}

const char *parse_result_name(ParseResult result) {
    switch (result) {
        case ParseResult::OK:
            return "ok";
        case ParseResult::SHORT:
            return "frame shorter than 8 bytes";
        case ParseResult::BAD_START:
            return "frame does not start with 0x2F";
        case ParseResult::BAD_LENGTH:
            return "frame length does not match its n byte";
        case ParseResult::BAD_CRC:
            return "frame CRC mismatch";
    }
    return "unknown";
}

size_t build_response(uint8_t cmd, uint16_t address, const uint8_t *params, size_t length,
                      uint8_t ack, uint8_t *out) {
    out[0] = RESPONSE_START;
    out[1] = cmd;
    out[2] = static_cast<uint8_t>(address >> 8);
    out[3] = static_cast<uint8_t>(address & 0xFF);
    out[4] = static_cast<uint8_t>(length & 0xFF);
    memmove(out + 5, params, length);
    out[5 + length] = ack;
    const uint16_t crc = crc_xmodem(out, 6 + length);
    out[6 + length] = static_cast<uint8_t>(crc >> 8);
    out[7 + length] = static_cast<uint8_t>(crc & 0xFF);
    return 8 + length;
}

void Server::send_boot(const uint8_t *data, size_t size) {
    uint8_t buf[256 + 2];
    memcpy(buf, data, size);
    const uint16_t crc = crc_boot(data, size);
    buf[size] = static_cast<uint8_t>(crc & 0xFF);
    buf[size + 1] = static_cast<uint8_t>(crc >> 8);
    _link.send(buf, size + 2);
}

bool Server::boot_ack(uint32_t first_ms) {
    return _link.receive(_rx, sizeof(_rx), 1, first_ms) >= 1 && _rx[0] == BOOT_SUCCESS;
}

bool Server::set_address(uint16_t address, uint8_t tries) {
    const uint8_t cmd[] = {BOOT_SET_ADDRESS, 0x00, static_cast<uint8_t>(address >> 8),
                           static_cast<uint8_t>(address & 0xFF)};
    for (uint8_t attempt = 0; attempt < tries; attempt++) {
        if (attempt > 0) _link.delay_ms(20);
        send_boot(cmd, sizeof(cmd));
        if (boot_ack(250)) return true;
    }
    return false;
}

uint8_t Server::init_flash(uint8_t esc, uint8_t *signature) {
    _link.select(esc);
    _link.send(BOOT_INIT, sizeof(BOOT_INIT));
    // The reply's length is not fixed: read until the line goes quiet. The passthrough maps
    // bytes 5, 4, 3 to the signature BLHeliSuite reads (06 47 6d 04 on these ESCs).
    const size_t n = _link.receive(_rx, sizeof(_rx), 0, 250);
    if (n < 6 || _rx[n - 1] != BOOT_SUCCESS) return ACK_GENERAL_ERROR;
    signature[0] = _rx[5];
    signature[1] = _rx[4];
    signature[2] = _rx[3];
    signature[3] = MODE_ARM_BLB;
    return ACK_OK;
}

uint8_t Server::read(uint16_t address, size_t length, uint8_t *data) {
    if (!set_address(address, 1)) return ACK_GENERAL_ERROR;
    const uint8_t cmd[] = {BOOT_READ_FLASH, static_cast<uint8_t>(length & 0xFF)};
    send_boot(cmd, sizeof(cmd));
    // Data, CRC low and high, result. 256 bytes take 135 ms on the wire at 19200.
    const size_t expect = length + 3;
    const size_t n = _link.receive(_rx, sizeof(_rx), expect, 500);
    if (n != expect || _rx[length + 2] != BOOT_SUCCESS) return ACK_GENERAL_ERROR;
    if (crc_boot(_rx, length + 2) != 0) return ACK_GENERAL_ERROR;
    memcpy(data, _rx, length);
    return ACK_OK;
}

// The passthrough's write, retries included, which wrote and verified a page on these ESCs on
// 2026-10-23. SetBuffer has no reply; the ESC answers after the data.
uint8_t Server::write(uint16_t address, const uint8_t *data, size_t length) {
    uint8_t ack = ACK_OK;
    if (!set_address(address, WRITE_TRIES)) ack = ACK_GENERAL_ERROR;

    const uint8_t buffer_cmd[] = {BOOT_SET_BUFFER, 0x00, static_cast<uint8_t>(length == 256),
                                  static_cast<uint8_t>(length & 0xFF)};
    send_boot(buffer_cmd, sizeof(buffer_cmd));
    _link.delay_ms(10);

    bool ok = false;
    for (uint8_t attempt = 0; attempt < WRITE_TRIES && !ok; attempt++) {
        if (attempt > 0) _link.delay_ms(20);
        send_boot(data, length);
        ok = boot_ack(350);
    }
    if (!ok) ack = ACK_GENERAL_ERROR;

    const uint8_t prog_cmd[] = {BOOT_PROG_FLASH, 0x01};
    ok = false;
    for (uint8_t attempt = 0; attempt < WRITE_TRIES && !ok; attempt++) {
        if (attempt > 0) _link.delay_ms(30);
        send_boot(prog_cmd, sizeof(prog_cmd));
        ok = boot_ack(400);  // the result waits for the flash commit
    }
    if (!ok) ack = ACK_GENERAL_ERROR;
    return ack;
}

void Server::reset_esc(uint8_t esc) {
    _link.select(esc);
    const uint8_t cmd[] = {BOOT_RESTART, 0x00};
    send_boot(cmd, sizeof(cmd));
    // Betaflight's sequence: line low 300 ms, then high. No reply; drain whatever comes.
    _link.hold_low(300);
    _link.receive(_rx, sizeof(_rx), 0, 50);
}

size_t Server::handle(const uint8_t *frame, size_t size, uint8_t *out) {
    Request req;
    const ParseResult parsed = parse_request(frame, size, req);
    const uint8_t zero = 0;
    if (parsed != ParseResult::OK) {
        if (size < 5) return 0;
        const uint16_t address = static_cast<uint16_t>((frame[2] << 8) | frame[3]);
        return build_response(frame[1], address, &zero, 1, ACK_INVALID_CRC, out);
    }

    const uint8_t param = req.params[0];
    switch (req.cmd) {
        case PROTOCOL_GET_VERSION:
            return build_response(req.cmd, req.address, &PROTOCOL_VERSION, 1, ACK_OK, out);
        case INTERFACE_GET_NAME:
            return build_response(req.cmd, req.address,
                                  reinterpret_cast<const uint8_t *>(INTERFACE_NAME),
                                  sizeof(INTERFACE_NAME) - 1, ACK_OK, out);
        case INTERFACE_GET_VERSION:
            return build_response(req.cmd, req.address, INTERFACE_VERSION,
                                  sizeof(INTERFACE_VERSION), ACK_OK, out);
        case INTERFACE_SET_MODE:
            return build_response(req.cmd, req.address, &zero, 1,
                                  param == MODE_ARM_BLB ? ACK_OK : ACK_INVALID_PARAM, out);
        case INTERFACE_EXIT:
            // Tuning mode owns the pins; leaving it is POST /esc/tuning.
            return build_response(req.cmd, req.address, &zero, 1, ACK_OK, out);
        case TEST_ALIVE: {
            // Answered OK whatever the ESC says, as the passthrough does.
            const uint8_t cmd[] = {BOOT_KEEP_ALIVE, 0x00};
            send_boot(cmd, sizeof(cmd));
            _link.receive(_rx, sizeof(_rx), 1, 200);
            return build_response(req.cmd, req.address, &zero, 1, ACK_OK, out);
        }
        case DEVICE_RESET:
            if (param >= _link.count()) {
                return build_response(req.cmd, req.address, &zero, 1, ACK_INVALID_CHANNEL, out);
            }
            reset_esc(param);
            return build_response(req.cmd, req.address, &zero, 1, ACK_OK, out);
        case DEVICE_INIT_FLASH: {
            if (param >= _link.count()) {
                return build_response(req.cmd, req.address, &zero, 1, ACK_INVALID_CHANNEL, out);
            }
            uint8_t signature[4] = {0, 0, 0, MODE_ARM_BLB};
            const uint8_t ack = init_flash(param, signature);
            return build_response(req.cmd, req.address, signature, 4, ack, out);
        }
        case DEVICE_READ: {
            // Read straight into the response's parameter bytes.
            const size_t length = param == 0 ? 256 : param;
            const uint8_t ack = read(req.address, length, out + 5);
            if (ack != ACK_OK) return build_response(req.cmd, req.address, &zero, 1, ack, out);
            return build_response(req.cmd, req.address, out + 5, length, ACK_OK, out);
        }
        case DEVICE_WRITE: {
            if (!write_allowed(req.address, req.length)) {
                _guard_refusals++;
                return build_response(req.cmd, req.address, &zero, 1, ACK_INVALID_PARAM, out);
            }
            const uint8_t ack = write(req.address, req.params, req.length);
            return build_response(req.cmd, req.address, &zero, 1, ack, out);
        }
        case DEVICE_PAGE_ERASE:
        case DEVICE_ERASE_ALL:
            _guard_refusals++;
            return build_response(req.cmd, req.address, &zero, 1, ACK_INVALID_PARAM, out);
        default:
            return build_response(req.cmd, req.address, &zero, 1, ACK_INVALID_CMD, out);
    }
}

}  // namespace blheli_boot
