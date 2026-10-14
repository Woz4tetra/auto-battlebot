// ESP-NOW dongle for mr_stabs_mk2. Bridges framed USB serial from the Jetson to ESP-NOW and back.
// Holds no authority: it forwards what the Jetson sends, repeats the latest command so one lost
// packet is covered in 4 ms, and never invents a command. See docs/plans/espnow_link_plan.md.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <espnow_protocol.h>
#include <hal/usb_serial_jtag_ll.h>

using namespace espnow_protocol;

namespace {

// Re-send the latest command at 250 Hz so a lost packet is covered quickly.
const uint32_t REPEAT_PERIOD_US = 4000;
// Stop repeating once the Jetson has been quiet this long. Without it the dongle would re-send a
// dead process's last command forever, and the robot would treat it as live.
const uint32_t COMMAND_SILENCE_MS = 100;
const uint32_t LINK_STATS_PERIOD_MS = 100;
// A send whose callback never arrives would wedge the queue. ESP-NOW reports every send within
// a few ms; this only covers a driver hiccup.
const uint32_t SEND_CALLBACK_TIMEOUT_MS = 50;
const uint32_t SCAN_DWELL_MS = 150;

const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct AirFrame {
    uint8_t mac[6];
    uint8_t length;
    uint8_t data[AIR_MAX_FRAME];
};

// Received air frames, handed from the WiFi task to loop().
QueueHandle_t rx_queue;

// Frames waiting for the radio. Commands do not queue here: only the newest command matters, so
// it lives in its own slot and jumps the queue.
const size_t TX_QUEUE_LEN = 24;
AirFrame tx_queue[TX_QUEUE_LEN];
size_t tx_head = 0;
size_t tx_count = 0;
uint32_t tx_dropped = 0;

uint8_t command_frame[AIR_MAX_FRAME];
size_t command_frame_len = 0;
bool command_pending = false;
uint32_t last_command_serial_ms = 0;
uint32_t last_repeat_us = 0;

uint8_t robot_mac[6];
bool robot_known = false;
uint8_t channel = 0;

volatile bool send_in_flight = false;
volatile uint32_t sent_count = 0;
volatile uint32_t acked_count = 0;
volatile uint32_t failed_count = 0;
volatile uint32_t last_rtt_us = 0;
uint32_t send_started_us = 0;
uint32_t send_started_ms = 0;
uint32_t last_send_ms = 0;
uint32_t last_stats_ms = 0;

SerialFrameParser parser;

// Promiscuous scan accumulators, indexed by channel.
volatile uint32_t scan_bytes[14];
volatile uint8_t scan_channel = 0;

void on_sent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    last_rtt_us = micros() - send_started_us;
    if (status == ESP_NOW_SEND_SUCCESS) {
        acked_count = acked_count + 1;
    } else {
        failed_count = failed_count + 1;
    }
    send_in_flight = false;
}

void on_recv(const uint8_t *mac, const uint8_t *data, int length) {
    // WiFi task context: copy and return. Everything else happens in loop().
    if (!is_air_frame(data, static_cast<size_t>(length))) return;
    AirFrame frame;
    memcpy(frame.mac, mac, 6);
    frame.length = static_cast<uint8_t>(length);
    memcpy(frame.data, data, length);
    xQueueSend(rx_queue, &frame, 0);
}

void on_promiscuous(void *buffer, wifi_promiscuous_pkt_type_t type) {
    (void)type;
    const auto *packet = static_cast<const wifi_promiscuous_pkt_t *>(buffer);
    if (scan_channel < 14) scan_bytes[scan_channel] += packet->rx_ctrl.sig_len;
}

// Scores every channel by bytes heard and picks the quietest of 1, 6, and 11, counting energy
// from overlapping neighbors. Bytes heard is a proxy for airtime: it misses non-WiFi interferers
// (other teams' control links, video TX), which nothing in a WiFi scan can see.
uint8_t pick_channel() {
    if (CHANNEL_OVERRIDE != 0) return CHANNEL_OVERRIDE;

    esp_wifi_set_promiscuous_rx_cb(on_promiscuous);
    esp_wifi_set_promiscuous(true);
    for (uint8_t ch = 1; ch <= 13; ch++) {
        scan_channel = ch;
        scan_bytes[ch] = 0;
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        delay(SCAN_DWELL_MS);
    }
    scan_channel = 0;
    esp_wifi_set_promiscuous(false);

    const uint8_t candidates[] = {1, 6, 11};
    const float weights[] = {1.0f, 0.7f, 0.3f};  // by channel distance 0, 1, 2
    uint8_t best = 1;
    float best_score = 0.0f;
    for (size_t i = 0; i < sizeof(candidates); i++) {
        const int c = candidates[i];
        float score = 0.0f;
        for (int d = -2; d <= 2; d++) {
            const int ch = c + d;
            if (ch >= 1 && ch <= 13) score += weights[abs(d)] * scan_bytes[ch];
        }
        if (i == 0 || score < best_score) {
            best = static_cast<uint8_t>(c);
            best_score = score;
        }
    }
    return best;
}

void add_peer(const uint8_t *mac) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = channel;
    peer.encrypt = false;
    if (esp_now_is_peer_exist(mac)) {
        esp_now_mod_peer(&peer);
    } else {
        esp_now_add_peer(&peer);
    }
}

void write_serial_frame(uint8_t type, const uint8_t *payload, uint8_t length) {
    uint8_t out[SERIAL_MAX_FRAME];
    const size_t size = encode_serial_frame(type, payload, length, out);
    // Never block on the host. With no reader the frame is dropped, which is what we want.
    if (Serial.availableForWrite() >= static_cast<int>(size)) {
        Serial.write(out, size);
        return;
    }
    // HWCDC's ISR disables the IN_EMPTY interrupt each time it fires and only Serial.write()
    // turns it back on. Skipping write() while the buffer is full leaves it off for good once
    // the host stops reading, and after a USB reset nothing drains the buffer again: the dongle
    // goes silent until it is power cycled. Re-arm it the way write() does.
    usb_serial_jtag_ll_txfifo_flush();
    usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
}

void queue_frame(uint8_t type, const uint8_t *payload, uint8_t length) {
    if (tx_count == TX_QUEUE_LEN) {
        tx_dropped++;
        return;
    }
    AirFrame &frame = tx_queue[(tx_head + tx_count) % TX_QUEUE_LEN];
    frame.length = static_cast<uint8_t>(encode_air_frame(type, payload, length, frame.data));
    if (frame.length == 0) return;
    tx_count++;
}

void on_serial_frame(void *context, uint8_t type, const uint8_t *payload, uint8_t length) {
    (void)context;
    if (type == MSG_COMMAND) {
        if (length != sizeof(CommandPayload)) return;
        // Forward on arrival: the new command takes the slot and goes out at the next free radio
        // slot, never waiting for the repeat timer.
        command_frame_len = encode_air_frame(type, payload, length, command_frame);
        command_pending = true;
        last_command_serial_ms = millis();
        last_repeat_us = micros();
        return;
    }
    // Everything else goes out once. OTA retries are the host's job; a repeated chunk would only
    // be dropped by the robot as out of order.
    queue_frame(type, payload, length);
}

void service_radio() {
    if (send_in_flight) {
        if (millis() - send_started_ms < SEND_CALLBACK_TIMEOUT_MS) return;
        send_in_flight = false;
        failed_count = failed_count + 1;
    }

    const uint8_t *data;
    size_t length;
    if (command_pending) {
        data = command_frame;
        length = command_frame_len;
        command_pending = false;
    } else if (tx_count > 0) {
        data = tx_queue[tx_head].data;
        length = tx_queue[tx_head].length;
        tx_head = (tx_head + 1) % TX_QUEUE_LEN;
        tx_count--;
    } else if (millis() - last_send_ms >= PING_PERIOD_MS) {
        static uint8_t ping[AIR_HEADER_SIZE];
        length = encode_air_frame(MSG_PING, nullptr, 0, ping);
        data = ping;
    } else {
        return;
    }
    last_send_ms = millis();

    // Broadcast until the robot is heard from. Broadcast gets no MAC-layer ACK or retry, so the
    // link stats only mean something once robot_known is set.
    const uint8_t *dest = robot_known ? robot_mac : BROADCAST_MAC;
    send_started_us = micros();
    send_started_ms = millis();
    send_in_flight = true;
    if (esp_now_send(dest, data, length) == ESP_OK) {
        sent_count = sent_count + 1;
    } else {
        send_in_flight = false;
        failed_count = failed_count + 1;
    }
}

void service_repeat() {
    if (command_frame_len == 0) return;
    if (millis() - last_command_serial_ms >= COMMAND_SILENCE_MS) return;
    const uint32_t now_us = micros();
    if (now_us - last_repeat_us < REPEAT_PERIOD_US) return;
    last_repeat_us = now_us;
    // Byte-identical copy, seq included. The robot treats an equal seq as a repeat.
    command_pending = true;
}

void service_receive() {
    AirFrame frame;
    while (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) {
        const uint8_t type = frame.data[sizeof(AIR_MAGIC)];
        // Only the robot sends these types, so the first one names the robot.
        if ((type == MSG_TELEMETRY || type == MSG_OTA_STATUS) &&
            (!robot_known || memcmp(frame.mac, robot_mac, 6) != 0)) {
            memcpy(robot_mac, frame.mac, 6);
            add_peer(robot_mac);
            esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M);
            robot_known = true;
        }
        write_serial_frame(type, frame.data + AIR_HEADER_SIZE,
                           static_cast<uint8_t>(frame.length - AIR_HEADER_SIZE));
    }
}

void service_link_stats() {
    const uint32_t now = millis();
    if (now - last_stats_ms < LINK_STATS_PERIOD_MS) return;
    last_stats_ms = now;
    const LinkStatsPayload stats = {sent_count,  acked_count, failed_count,
                                    last_rtt_us, channel,     static_cast<uint8_t>(robot_known)};
    write_serial_frame(MSG_LINK_STATS, reinterpret_cast<const uint8_t *>(&stats), sizeof(stats));
}

}  // namespace

void setup() {
    // The default 256-byte RX buffer overflows on the OTA host's 16-chunk (~4 KB) window and the
    // HWCDC ISR drops the excess, so every chunk after the first fails its CRC. Two windows.
    Serial.setRxBufferSize(8192);
    Serial.begin(921600);
    Serial.setTxTimeoutMs(0);

    rx_queue = xQueueCreate(32, sizeof(AirFrame));

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);

    channel = pick_channel();
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    esp_now_init();
    esp_now_register_send_cb(on_sent);
    esp_now_register_recv_cb(on_recv);
    add_peer(BROADCAST_MAC);
    // After esp_wifi_start(), which WiFi.mode() ran. The 1 Mbps default costs ~1.2 ms a frame.
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M);

    // Plain text, once. The Jetson's parser skips anything that is not a framed message.
    Serial.printf("\nespnow_dongle mac=%s channel=%u\n", WiFi.macAddress().c_str(), channel);
}

void loop() {
    uint8_t buffer[256];
    const int available = Serial.available();
    if (available > 0) {
        const size_t count = Serial.read(buffer, min(available, static_cast<int>(sizeof(buffer))));
        parser.feed(buffer, count, on_serial_frame, nullptr);
    }
    service_repeat();
    service_radio();
    service_receive();
    service_link_stats();
}
