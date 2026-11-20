#include <Arduino.h>
#include <WiFi.h>
#include <esp32s3/rom/crc.h>
#include <esp32s3/rom/ets_sys.h>
#include <lwip/sockets.h>
#include <mbedtls/base64.h>
#include <peer_closed.h>
#include <usb_bridge.h>
#include <usb_framing.h>

namespace usb_bridge {

static const char FRAME_START = 0x1e;
static const size_t MAX_CONNECTIONS = 6;
static const size_t MAX_REQUEST_LINE = 4096;
static const size_t RESPONSE_CHUNK_BYTES = 384;
static const uint32_t WRITE_TIMEOUT_MS = 1000;

struct Connection {
    uint32_t id = 0;
    bool open = false;
    WiFiClient client;
};

static Connection connections[MAX_CONNECTIONS];

static bool write_whole(const char *data, size_t size) {
    return usb_framing::write_whole(
        Serial, reinterpret_cast<const uint8_t *>(data), size, WRITE_TIMEOUT_MS,
        [] { return millis(); }, [] { vTaskDelay(1); });
}

static const size_t LOG_RING = 2048;
static char log_ring[LOG_RING];
static size_t log_write_index = 0;
static size_t log_read_index = 0;
static portMUX_TYPE log_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR log_putc(char c) {
    portENTER_CRITICAL_SAFE(&log_mux);
    usb_framing::push_log_char(log_ring, LOG_RING, log_write_index, log_read_index, c);
    portEXIT_CRITICAL_SAFE(&log_mux);
}

static void flush_log() {
    static char out[LOG_RING];
    size_t n = 0;
    portENTER_CRITICAL(&log_mux);
    const size_t last_line = usb_framing::copy_drainable_log(log_ring, LOG_RING, log_read_index,
                                                             log_write_index, out, n);
    log_read_index = (log_read_index + last_line) % LOG_RING;
    portEXIT_CRITICAL(&log_mux);
    if (last_line > 0) write_whole(out, last_line);
}

static char line[MAX_REQUEST_LINE];
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

static bool peer_closed(WiFiClient &client) {
    int error = 0;
    const peer_closed::State state = peer_closed::probe(client.fd(), &error);
    if (state == peer_closed::State::ERROR) {
        Serial.printf("usb_bridge: fd %d closed with errno %d\n", client.fd(), error);
    }
    return state != peer_closed::State::OPEN;
}

static void open_connection(uint32_t id, const char *encoded) {
    static uint8_t request[MAX_REQUEST_LINE];
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
        } else if (line_size < MAX_REQUEST_LINE - 1) {
            line[line_size++] = static_cast<char>(b);
        } else {
            line_overflow = true;
        }
    }
}

static bool pump_connections() {
    static uint8_t data[RESPONSE_CHUNK_BYTES];
    static char out[16 + 4 * ((RESPONSE_CHUNK_BYTES + 2) / 3) + 12];
    bool moved = false;
    for (Connection &c : connections) {
        if (!c.open) continue;
        const int available = c.client.available();
        if (available > 0) {
            const int n = c.client.read(
                data, available < (int)RESPONSE_CHUNK_BYTES ? available : RESPONSE_CHUNK_BYTES);
            if (n > 0) {
                int at = snprintf(out, sizeof(out), "%cD %lu ", FRAME_START, (unsigned long)c.id);
                size_t written = 0;
                mbedtls_base64_encode(reinterpret_cast<uint8_t *>(out + at), sizeof(out) - at - 1,
                                      &written, data, n);
                at += written;
                at += snprintf(out + at, sizeof(out) - at, " %08lx\n",
                               (unsigned long)crc32_le(0, data, n));
                if (!write_whole(out, at)) close(c);
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
    xTaskCreatePinnedToCore(task, "usb_bridge", 6144, nullptr, 1, nullptr, 1);
}

}  // namespace usb_bridge
