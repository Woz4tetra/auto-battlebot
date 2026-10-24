#include "blheli_boot.h"

#include <cstdio>
#include <cstring>

namespace blheli_boot {

// The passthrough's DeviceInitFlash handshake. It carries its own CRC, so it goes out raw.
static const uint8_t BOOT_INIT[] = {0,   0,   0,   0,   0,   0,   0,    0,   0x0D,
                                    'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
// The passthrough's InterfaceGetVersion, 20.0.05.
static const uint8_t INTERFACE_VERSION[] = {200, 5};
// The passthrough's retry count for the steps of a write.
static const uint8_t WRITE_TRIES = 3;
static const uint8_t READ_TRIES = 3;  // a read changes nothing, so it can repeat
static const uint8_t INIT_TRIES = 3;
static const uint32_t REBOOT_SETTLE_MS = 500;
static const size_t INIT_REPLY_SIZE = 9;

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

void Server::describe(char *out, size_t size, const char *step, size_t received,
                      size_t expected) const {
    int at = expected > 0
                 ? snprintf(out, size, "%s: got %u of %u bytes", step,
                            static_cast<unsigned>(received), static_cast<unsigned>(expected))
                 : snprintf(out, size, "%s: %u bytes", step, static_cast<unsigned>(received));
    const size_t shown = received < 12 ? received : 12;
    for (size_t i = 0; i < shown && at > 0 && static_cast<size_t>(at) < size; i++) {
        at += snprintf(out + at, size - at, "%s%02x", i == 0 ? " [" : " ", _rx[i]);
    }
    if (shown > 0 && at > 0 && static_cast<size_t>(at) < size) {
        snprintf(out + at, size - at, "%s]", received > shown ? " .." : "");
    }
}

uint8_t Server::fail(const char *step, size_t received, size_t expected) {
    describe(_failure, sizeof(_failure), step, received, expected);
    return ACK_GENERAL_ERROR;
}

void Server::send_boot(const uint8_t *data, size_t size) {
    uint8_t buf[256 + 2];
    memcpy(buf, data, size);
    const uint16_t crc = crc_boot(data, size);
    buf[size] = static_cast<uint8_t>(crc & 0xFF);
    buf[size + 1] = static_cast<uint8_t>(crc >> 8);
    _link.send(buf, size + 2);
}

// Replies are read until the line goes quiet and checked from their end. Switching the pin from
// transmit to receive sometimes decodes as a stray byte (0xfe) ahead of the reply, seen on the
// robot on 2026-10-23; the passthrough's first-byte checks would fail on it the same way.
bool Server::boot_ack(uint32_t first_ms, const char *step) {
    const size_t n = _link.receive(_rx, sizeof(_rx), 0, first_ms);
    if (n >= 1 && _rx[n - 1] == BOOT_SUCCESS) return true;
    fail(step, n, 1);
    return false;
}

bool Server::set_address(uint16_t address, uint8_t tries) {
    const uint8_t cmd[] = {BOOT_SET_ADDRESS, 0x00, static_cast<uint8_t>(address >> 8),
                           static_cast<uint8_t>(address & 0xFF)};
    for (uint8_t attempt = 0; attempt < tries; attempt++) {
        if (attempt > 0) _link.delay_ms(20);
        send_boot(cmd, sizeof(cmd));
        if (boot_ack(250, "set address")) return true;
    }
    return false;
}

// The handshake reply is "471", the signature, boot version and pages, then 0x30:
// 34 37 31 6d 47 06 07 04 30 on these ESCs.
bool Server::handshake(uint8_t *signature) {
    static const uint16_t RETRY_DELAY_MS[INIT_TRIES] = {0, 100, 250};
    for (uint8_t attempt = 0; attempt < INIT_TRIES; attempt++) {
        if (RETRY_DELAY_MS[attempt] > 0) _link.delay_ms(RETRY_DELAY_MS[attempt]);
        _link.send(BOOT_INIT, sizeof(BOOT_INIT));
        const size_t n = _link.receive(_rx, sizeof(_rx), 0, 250);
        describe(_init_reply, sizeof(_init_reply), "init reply", n, 0);
        if (n < INIT_REPLY_SIZE) {
            fail("init flash", n, INIT_REPLY_SIZE);
            continue;
        }
        const uint8_t *reply = _rx + n - INIT_REPLY_SIZE;
        if (reply[0] != '4' || reply[1] != '7' || reply[2] != '1' ||
            reply[INIT_REPLY_SIZE - 1] != BOOT_SUCCESS) {
            fail("init flash", n, INIT_REPLY_SIZE);
            continue;
        }
        _failure[0] = '\0';
        // The passthrough's order, which is what BLHeliSuite reads: 06 47 6d 04 here.
        signature[0] = reply[5];
        signature[1] = reply[4];
        signature[2] = reply[3];
        signature[3] = MODE_ARM_BLB;
        return true;
    }
    return false;
}

// Tuning mode puts the ESCs in their bootloader by stopping DShot, so the handshake normally
// answers at once. If it does not, the ESC is restarted the passthrough's way and given
// REBOOT_SETTLE_MS of quiet high line before the handshake is tried again.
uint8_t Server::init_flash(uint8_t esc, uint8_t *signature) {
    _link.select(esc);
    _esc = esc;
    if (handshake(signature)) return ACK_OK;
    _reboots++;
    reset_esc(esc);
    _link.delay_ms(REBOOT_SETTLE_MS);
    _link.select(esc);
    return handshake(signature) ? ACK_OK : ACK_GENERAL_ERROR;
}

uint8_t Server::read_once(uint16_t address, size_t length, uint8_t *data) {
    if (!set_address(address, 1)) return ACK_GENERAL_ERROR;
    const uint8_t cmd[] = {BOOT_READ_FLASH, static_cast<uint8_t>(length & 0xFF)};
    send_boot(cmd, sizeof(cmd));
    // Data, CRC low and high, result. 256 bytes take 135 ms on the wire at 19200.
    const size_t expect = length + 3;
    const size_t n = _link.receive(_rx, sizeof(_rx), 0, 500);
    if (n < expect) return fail("read flash", n, expect);
    const uint8_t *reply = _rx + n - expect;
    if (reply[length + 2] != BOOT_SUCCESS) return fail("read flash", n, expect);
    if (crc_boot(reply, length + 2) != 0) return fail("read flash CRC", n, expect);
    _failure[0] = '\0';
    memcpy(data, reply, length);
    return ACK_OK;
}

// A read changes nothing, so it repeats. On the robot a session's first 256-byte read has failed
// its CRC three times running and taken the session down (2 CLI runs in 9 on 2026-10-23), so the
// last resort redoes the handshake before trying again.
uint8_t Server::read(uint16_t address, size_t length, uint8_t *data) {
    _first_read_failure[0] = '\0';
    for (uint8_t attempt = 0; attempt < READ_TRIES; attempt++) {
        if (attempt > 0) _link.delay_ms(20);
        if (read_once(address, length, data) == ACK_OK) return ACK_OK;
        if (attempt == 0) memcpy(_first_read_failure, _failure, sizeof(_failure));
    }
    if (_esc < 0) return ACK_GENERAL_ERROR;
    _resyncs++;
    uint8_t signature[4];
    _link.select(static_cast<uint8_t>(_esc));
    if (!handshake(signature)) return ACK_GENERAL_ERROR;
    for (uint8_t attempt = 0; attempt < READ_TRIES; attempt++) {
        if (read_once(address, length, data) == ACK_OK) return ACK_OK;
        _link.delay_ms(20);
    }
    return ACK_GENERAL_ERROR;
}

// The passthrough's write and retries, which wrote and verified a page on these ESCs on
// 2026-10-23, except that it stops when the address or the data is not acknowledged: the
// passthrough sends ProgFlash anyway, which could commit a buffer that never arrived intact.
// SetBuffer has no reply; the ESC answers after the data.
uint8_t Server::write(uint16_t address, const uint8_t *data, size_t length) {
    if (!set_address(address, WRITE_TRIES)) return ACK_GENERAL_ERROR;

    const uint8_t buffer_cmd[] = {BOOT_SET_BUFFER, 0x00, static_cast<uint8_t>(length == 256),
                                  static_cast<uint8_t>(length & 0xFF)};
    send_boot(buffer_cmd, sizeof(buffer_cmd));
    _link.delay_ms(10);

    bool ok = false;
    for (uint8_t attempt = 0; attempt < WRITE_TRIES && !ok; attempt++) {
        if (attempt > 0) _link.delay_ms(20);
        send_boot(data, length);
        ok = boot_ack(350, "write buffer");
    }
    if (!ok) return ACK_GENERAL_ERROR;

    const uint8_t prog_cmd[] = {BOOT_PROG_FLASH, 0x01};
    ok = false;
    for (uint8_t attempt = 0; attempt < WRITE_TRIES && !ok; attempt++) {
        if (attempt > 0) _link.delay_ms(30);
        send_boot(prog_cmd, sizeof(prog_cmd));
        ok = boot_ack(400, "program flash");  // the result waits for the flash commit
    }
    if (!ok) return ACK_GENERAL_ERROR;
    _failure[0] = '\0';
    return ACK_OK;
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
            // BLHeliSuite and the clients send this before each DeviceInitFlash and after each
            // session. A real restart holds the line low, so the ESC boots its application and
            // has to fall back into the bootloader through its lost-signal check; a handshake
            // landing on that check left the ESC unreachable in 1 session in 10 on 2026-10-23.
            // The ESC is already in its bootloader, so only select it. Leaving tuning mode, the
            // idle timeout, and a failed handshake still restart it with reset_esc().
            if (param >= _link.count()) {
                return build_response(req.cmd, req.address, &zero, 1, ACK_INVALID_CHANNEL, out);
            }
            _link.select(param);
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
