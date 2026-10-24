#include <Arduino.h>
#include <WiFi.h>
#include <esp32s3/rom/crc.h>
#include <esp32s3/rom/ets_sys.h>
#include <lwip/sockets.h>
#include <mbedtls/base64.h>
#include <usb_bridge.h>

namespace usb_bridge {

static const char FRAME_START = 0x1e;
static const size_t MAX_CONNECTIONS = 6;
static const size_t MAX_LINE = 4096;  // a base64 request: the largest is a 4-way frame POST
static const size_t CHUNK = 384;      // response bytes per frame

struct Connection {
    uint32_t id = 0;
    bool open = false;
    WiFiClient client;
};

static Connection connections[MAX_CONNECTIONS];

// Writes a frame in one call once the USB ring buffer has room for all of it, so log lines and
// frames never split each other. False when the laptop stopped reading.
static bool write_whole(const char *data, size_t size) {
    const uint32_t start = millis();
    while (static_cast<size_t>(Serial.availableForWrite()) < size) {
        if (millis() - start > 1000) return false;
        vTaskDelay(1);
    }
    Serial.write(reinterpret_cast<const uint8_t *>(data), size);
    return true;
}

// Arduino's log_e() and friends write one character at a time through the ROM putc hook, from any
// task or core, with no lock, so they would land in the middle of a frame. Collect them here and
// let the bridge task write whole lines between frames.
static const size_t LOG_RING = 2048;
static char log_ring[LOG_RING];
static size_t log_head = 0;  // next write
static size_t log_tail = 0;  // next read
static portMUX_TYPE log_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR log_putc(char c) {
    portENTER_CRITICAL_SAFE(&log_mux);
    const size_t next = (log_head + 1) % LOG_RING;
    if (next != log_tail) {  // full: drop the character
        log_ring[log_head] = c;
        log_head = next;
    }
    portEXIT_CRITICAL_SAFE(&log_mux);
}

// Writes out the buffered log up to its last complete line.
static void flush_log() {
    static char out[LOG_RING];
    size_t n = 0;
    portENTER_CRITICAL(&log_mux);
    size_t last_line = 0;
    for (size_t i = log_tail; i != log_head; i = (i + 1) % LOG_RING) {
        out[n++] = log_ring[i];
        if (log_ring[i] == '\n') last_line = n;
    }
    // A full ring with no newline would never drain: write it all.
    if (last_line == 0 && n >= LOG_RING - 1) last_line = n;
    log_tail = (log_tail + last_line) % LOG_RING;
    portEXIT_CRITICAL(&log_mux);
    if (last_line > 0) write_whole(out, last_line);
}

static char line[MAX_LINE];
static size_t line_size = 0;
static bool line_overflow = false;

static void send_end(uint32_t id) {
    char out[24];
    const int n = snprintf(out, sizeof(out), "%cE %lu\n", FRAME_START, (unsigned long)id);
    write_whole(out, n);
}

static void close(Connection &c) {
    c.client.stop();
    c.open = false;
}

// WiFiClient::connected() probes with a zero-length recv, which never reports an orderly close,
// so a finished response would hold its slot forever. Peek one byte instead: 0 means EOF.
static bool peer_closed(WiFiClient &client) {
    const int fd = client.fd();
    if (fd < 0) return true;
    uint8_t byte;
    const int n = recv(fd, &byte, 1, MSG_PEEK | MSG_DONTWAIT);
    if (n == 0) return true;
    if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
        Serial.printf("usb_bridge: fd %d closed with errno %d\n", fd, errno);
        return true;
    }
    return false;
}

static void open_connection(uint32_t id, const char *encoded) {
    static uint8_t request[MAX_LINE];
    size_t size = 0;
    if (mbedtls_base64_decode(request, sizeof(request), &size,
                              reinterpret_cast<const uint8_t *>(encoded), strlen(encoded)) != 0) {
        Serial.printf("usb_bridge: %lu: bad base64\n", (unsigned long)id);
        send_end(id);
        return;
    }
    Connection *slot = nullptr;
    for (Connection &c : connections) {
        if (!c.open) {
            slot = &c;
            break;
        }
    }
    if (slot == nullptr) {
        Serial.printf("usb_bridge: %lu: all %u connections busy\n", (unsigned long)id,
                      (unsigned)MAX_CONNECTIONS);
        send_end(id);
        return;
    }
    const uint32_t connect_start = millis();
    const bool connected = slot->client.connect(IPAddress(127, 0, 0, 1), 80, 1000);
    const uint32_t connect_ms = millis() - connect_start;
    if (connect_ms > 200) {
        Serial.printf("usb_bridge: %lu: connect took %lu ms\n", (unsigned long)id,
                      (unsigned long)connect_ms);
    }
    if (!connected) {
        Serial.printf("usb_bridge: %lu: connect to 127.0.0.1:80 failed\n", (unsigned long)id);
        send_end(id);
        return;
    }
    slot->id = id;
    slot->open = true;
    slot->client.write(request, size);
}

static void handle_line() {
    if (line_size < 4 || line[0] != FRAME_START || line[2] != ' ') return;
    char *end = nullptr;
    const uint32_t id = strtoul(line + 3, &end, 10);
    if (line[1] == 'Q' && *end == ' ') {
        open_connection(id, end + 1);
    } else if (line[1] == 'C') {
        for (Connection &c : connections) {
            if (c.open && c.id == id) close(c);
        }
    }
}

static void read_serial() {
    while (Serial.available()) {
        const int b = Serial.read();
        if (b < 0) return;
        if (b == '\n') {
            line[line_size] = '\0';
            if (!line_overflow) handle_line();
            line_size = 0;
            line_overflow = false;
        } else if (line_size < MAX_LINE - 1) {
            line[line_size++] = static_cast<char>(b);
        } else {
            line_overflow = true;
        }
    }
}

// Returns true when any connection moved bytes.
static bool pump_connections() {
    static uint8_t data[CHUNK];
    static char out[16 + 4 * ((CHUNK + 2) / 3) + 12];
    bool moved = false;
    for (Connection &c : connections) {
        if (!c.open) continue;
        const int available = c.client.available();
        if (available > 0) {
            const int n = c.client.read(data, available < (int)CHUNK ? available : CHUNK);
            if (n > 0) {
                int at = snprintf(out, sizeof(out), "%cD %lu ", FRAME_START, (unsigned long)c.id);
                size_t written = 0;
                mbedtls_base64_encode(reinterpret_cast<uint8_t *>(out + at), sizeof(out) - at - 1,
                                      &written, data, n);
                at += written;
                at += snprintf(out + at, sizeof(out) - at, " %08lx\n",
                               (unsigned long)crc32_le(0, data, n));
                if (!write_whole(out, at)) close(c);  // the laptop is not reading
                moved = true;
            }
        } else if (peer_closed(c.client)) {
            send_end(c.id);
            close(c);
            moved = true;
        }
    }
    return moved;
}

static void task(void *) {
    for (;;) {
        flush_log();
        read_serial();
        if (!pump_connections()) vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void begin() {
    ets_install_putc2(log_putc);
    // Core 1, where Serial.begin() put the USB-Serial/JTAG interrupt. Writing from core 0 raced it
    // and lost whole 64-byte packets: 2 to 5 damaged frames per 10 kB file, none on core 1 (236
    // frames checked on 2026-10-23).
    xTaskCreatePinnedToCore(task, "usb_bridge", 6144, nullptr, 1, nullptr, 1);
}

}  // namespace usb_bridge
