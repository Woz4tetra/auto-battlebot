#include <Preferences.h>
#include <Update.h>
#include <esp_now.h>
#include <espnow_link.h>
#include <pair_decision.h>
#include <tuning_store.h>

#include <atomic>

using namespace espnow_protocol;

static_assert(AIR_MAX_FRAME == ESP_NOW_MAX_DATA_LEN,
              "air frame size must track the IDF ESP-NOW v1 limit");

namespace espnow_link {
namespace {

struct RxFrame {
    uint8_t mac[6];
    uint8_t length;
    uint8_t data[AIR_MAX_FRAME];
};

const UBaseType_t RX_QUEUE_DEPTH = 40;
QueueHandle_t rx_queue = nullptr;

volatile uint32_t send_ack_history_newest_lsb = 0;
volatile uint8_t send_history_count = 0;

std::atomic<uint32_t> tx_ok{0};
std::atomic<uint32_t> tx_fail{0};
std::atomic<uint32_t> tx_errors{0};
std::atomic<uint32_t> rx_rejected{0};
std::atomic<uint32_t> rx_queue_full{0};
uint8_t rejected_mac[6] = {0};

const size_t MAX_ALLOWED = MAX_PAIRED_DONGLES;
static_assert(MAX_ALLOWED <= 12, "espnow_status_t::allowed holds 12");
uint8_t allowed_macs[MAX_ALLOWED][6];
std::atomic<uint8_t> allowed_count{0};

bool is_dongle(const uint8_t *mac) {
    const uint8_t count = allowed_count.load(std::memory_order_acquire);
    for (uint8_t i = 0; i < count; i++) {
        if (memcmp(mac, allowed_macs[i], 6) == 0) return true;
    }
    return false;
}

void hex_to_md5(const String &hex, uint8_t *out) {
    for (int i = 0; i < 16; i++) {
        out[i] =
            static_cast<uint8_t>(strtoul(hex.substring(i * 2, i * 2 + 2).c_str(), nullptr, 16));
    }
}

void add_or_update_peer(const uint8_t *mac, wifi_interface_t ifidx, uint8_t channel) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.ifidx = ifidx;
    peer.channel = channel;
    peer.encrypt = false;
    if (esp_now_is_peer_exist(mac)) {
        esp_now_mod_peer(&peer);
    } else {
        esp_now_add_peer(&peer);
    }
}

}  // namespace

bool EspNowLink::begin(wifi_interface_t ifidx, uint8_t pinned_channel, TuningStore *tuning) {
    ifidx_ = ifidx;
    tuning_ = tuning;
    rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(RxFrame));

    hex_to_md5(ESP.getSketchMD5(), running_md5_);

    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(on_recv);
    esp_now_register_send_cb(on_sent);
    begin_ok_ = true;

    Preferences prefs;
    prefs.begin("espnow", true);
    last_good_channel_ = prefs.getUChar("channel", 0);
    prefs.end();
    load_paired();

    if (pinned_channel != SWEEP_CHANNEL) {
        set_channel(pinned_channel);
    } else {
        sweeping_ = true;
        sweep_index_ = 0;
        set_channel(last_good_channel_ != 0 ? last_good_channel_ : CHANNEL_CANDIDATES[0]);
        sweep_dwell_start_ms_ = millis();
    }
    return true;
}

void EspNowLink::set_channel(uint8_t channel) {
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    channel_ = channel;
    const uint8_t count = allowed_count.load(std::memory_order_acquire);
    for (uint8_t i = 0; i < count; i++) add_or_update_peer(allowed_macs[i], ifidx_, channel);
    esp_wifi_config_espnow_rate(ifidx_, WIFI_PHY_RATE_6M);
}

void EspNowLink::on_recv(const uint8_t *mac, const uint8_t *data, int length) {
    if (rx_queue == nullptr || !is_air_frame(data, static_cast<size_t>(length))) return;
    if (!is_dongle(mac)) {
        memcpy(rejected_mac, mac, 6);
        rx_rejected.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    RxFrame frame;
    memcpy(frame.mac, mac, 6);
    frame.length = static_cast<uint8_t>(length);
    memcpy(frame.data, data, length);
    if (xQueueSend(rx_queue, &frame, 0) != pdTRUE)
        rx_queue_full.fetch_add(1, std::memory_order_relaxed);
}

void EspNowLink::on_sent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    send_ack_history_newest_lsb =
        (send_ack_history_newest_lsb << 1) | (status == ESP_NOW_SEND_SUCCESS ? 1u : 0u);
    if (send_history_count < 32) send_history_count = send_history_count + 1;
    if (status == ESP_NOW_SEND_SUCCESS) {
        tx_ok.fetch_add(1, std::memory_order_relaxed);
    } else {
        tx_fail.fetch_add(1, std::memory_order_relaxed);
    }
}

void EspNowLink::update(uint32_t now_ms, bool armed) {
    RxFrame frame;
    while (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) {
        handle_frame(frame.mac, frame.data[sizeof(AIR_MAGIC)], frame.data + AIR_HEADER_SIZE,
                     frame.length - AIR_HEADER_SIZE, now_ms, armed);
    }

    if (ota_should_abort(ota_.active(), armed, now_ms, last_ota_frame_ms_, OTA_SILENCE_MS)) {
        ota_abort(OTA_ERR_ABORTED);
    }

    service_sweep(now_ms);
    service_validation(now_ms);
}

void EspNowLink::handle_frame(const uint8_t *mac, uint8_t type, const uint8_t *payload,
                              size_t length, uint32_t now_ms, bool armed) {
    memcpy(reply_mac_, mac, 6);
    heard_dongle_ = true;
    last_valid_frame_ms_ = now_ms;
    rx_frames_++;

    switch (type) {
        case MSG_COMMAND: {
            if (length != sizeof(CommandPayload)) {
                rx_size_errors_++;
                return;
            }
            CommandPayload command;
            memcpy(&command, payload, sizeof(command));
            switch (tracker_.on_command(command, now_ms)) {
                case CommandTracker::ACCEPTED_NEW:
                    rx_commands_new_++;
                    break;
                case CommandTracker::ACCEPTED_REPEAT:
                    rx_commands_repeat_++;
                    break;
                case CommandTracker::DROPPED_OLD:
                    rx_commands_old_++;
                    break;
            }
            break;
        }
        case MSG_TUNABLE_SET: {
            rx_tunables_++;
            if (length != sizeof(TunableSetPayload)) {
                rx_size_errors_++;
                return;
            }
            TunableSetPayload tunable;
            memcpy(&tunable, payload, sizeof(tunable));
            if (!tuning_) break;
            if (tunable.id == TUNABLE_LEFT_ESC_DEADZONE) {
                tuning_->set("left_deadzone", tunable.value);
            } else if (tunable.id == TUNABLE_RIGHT_ESC_DEADZONE) {
                tuning_->set("right_deadzone", tunable.value);
            }
            break;
        }
        case MSG_OTA_BEGIN:
            rx_ota_++;
            ota_begin(mac, payload, length, now_ms, armed);
            break;
        case MSG_OTA_CHUNK:
            rx_ota_++;
            ota_chunk(payload, length, now_ms);
            break;
        case MSG_OTA_END:
            rx_ota_++;
            ota_end();
            break;
        case MSG_OTA_ABORT:
            rx_ota_++;
            if (ota_.active()) ota_abort(OTA_ERR_ABORTED);
            break;
        case MSG_PING:
            rx_pings_++;
            break;
        default:
            break;
    }
}

void EspNowLink::ota_begin(const uint8_t *mac, const uint8_t *payload, size_t length,
                           uint32_t now_ms, bool armed) {
    if (length != sizeof(OtaBeginPayload)) return;
    memcpy(ota_mac_, mac, 6);
    if (armed) {
        send_ota_status(OTA_ERROR, OTA_ERR_ARMED);
        return;
    }
    if (ota_.active()) {
        Update.abort();
        ota_.reset();
    }

    OtaBeginPayload begin;
    memcpy(&begin, payload, sizeof(begin));
    char md5_hex[33];
    for (int i = 0; i < 16; i++) snprintf(md5_hex + i * 2, 3, "%02x", begin.md5[i]);

    if (!Update.begin(begin.size)) {
        send_ota_status(OTA_ERROR, OTA_ERR_BEGIN);
        return;
    }
    Update.setMD5(md5_hex);
    ota_.begin(begin.size);
    last_ota_frame_ms_ = now_ms;
    send_ota_status(OTA_RECEIVING, OTA_OK);
}

void EspNowLink::ota_chunk(const uint8_t *payload, size_t length, uint32_t now_ms) {
    if (!ota_.active() || length <= sizeof(OtaChunkHeader)) return;
    last_ota_frame_ms_ = now_ms;

    OtaChunkHeader header;
    memcpy(&header, payload, sizeof(header));
    uint8_t *data = const_cast<uint8_t *>(payload) + sizeof(header);
    const uint32_t data_length = static_cast<uint32_t>(length - sizeof(header));

    if (ota_.on_chunk(header.offset, data_length) == OtaSequencer::DROP) {
        if (now_ms - last_drop_status_ms_ >= 20) {
            last_drop_status_ms_ = now_ms;
            send_ota_status(OTA_RECEIVING, OTA_OK);
        }
        return;
    }
    if (Update.write(data, data_length) != data_length) {
        ota_abort(OTA_ERR_WRITE);
        return;
    }
    if (ota_.take_ack_due()) send_ota_status(OTA_RECEIVING, OTA_OK);
}

void EspNowLink::ota_end() {
    if (!ota_.active()) {
        send_ota_status(OTA_ERROR, OTA_ERR_NOT_ACTIVE);
        return;
    }
    if (!ota_.complete()) {
        send_ota_status(OTA_RECEIVING, OTA_OK);
        return;
    }
    if (!Update.end()) {
        ota_.reset();
        send_ota_status(OTA_ERROR, OTA_ERR_VERIFY);
        return;
    }
    ota_.reset();
    for (int i = 0; i < 3; i++) {
        send_ota_status(OTA_DONE, OTA_OK);
        delay(20);
    }
    delay(200);
    ESP.restart();
}

void EspNowLink::ota_abort(OtaError error) {
    Update.abort();
    ota_.reset();
    send_ota_status(OTA_ERROR, error);
}

void EspNowLink::send_ota_status(OtaState state, OtaError error) {
    ota_state_ = state;
    OtaStatusPayload status;
    status.state = state;
    status.next_offset = ota_.next_offset();
    status.error = error;
    memcpy(status.running_md5, running_md5_, sizeof(running_md5_));
    send_frame(ota_mac_, MSG_OTA_STATUS, &status, sizeof(status));
}

void EspNowLink::service_validation(uint32_t now_ms) {
    if (!validated_) {
        if (now_ms < VALIDATE_AFTER_MS || !heard_dongle_) return;
        validated_ = true;
        validated_reports_left_ = 10;
        memcpy(ota_mac_, reply_mac_, 6);
    }
    if (validated_reports_left_ > 0 && now_ms - last_validated_report_ms_ >= 500) {
        last_validated_report_ms_ = now_ms;
        validated_reports_left_--;
        send_ota_status(OTA_VALIDATED, OTA_OK);
    }
}

void EspNowLink::service_sweep(uint32_t now_ms) {
    if (CHANNEL_OVERRIDE != SWEEP_CHANNEL) return;
    if (!sweeping_) {
        if (now_ms - last_valid_frame_ms_ > REACQUIRE_SILENCE_MS) {
            sweeping_ = true;
            sweep_index_ = 0;
            sweep_dwell_start_ms_ = now_ms;
        }
        return;
    }
    if (heard_dongle_ && now_ms - last_valid_frame_ms_ <= CHANNEL_DWELL_MS) {
        sweeping_ = false;
        if (channel_ != last_good_channel_) {
            last_good_channel_ = channel_;
            Preferences prefs;
            prefs.begin("espnow", false);
            prefs.putUChar("channel", channel_);
            prefs.end();
        }
        return;
    }
    if (now_ms - sweep_dwell_start_ms_ < CHANNEL_DWELL_MS) return;
    sweep_dwell_start_ms_ = now_ms;
    sweep_index_ = (sweep_index_ + 1) % (sizeof(CHANNEL_CANDIDATES) + 1);
    set_channel(next_sweep_channel());
}

uint8_t EspNowLink::next_sweep_channel() const {
    if (sweep_index_ == 0 && last_good_channel_ != 0) return last_good_channel_;
    return CHANNEL_CANDIDATES[(sweep_index_ + sizeof(CHANNEL_CANDIDATES) - 1) %
                              sizeof(CHANNEL_CANDIDATES)];
}

void EspNowLink::send_telemetry(TelemetryPayload &telemetry, uint32_t now_ms) {
    if (!heard_dongle_ || now_ms - last_telemetry_ms_ < TELEMETRY_PERIOD_MS) return;
    last_telemetry_ms_ = now_ms;

    const uint32_t age = tracker_.age_ms(now_ms);
    telemetry.espnow_age_ms = age;
    telemetry.espnow_seq = tracker_.latest().seq;
    telemetry.jetson_tx_us = tracker_.latest().jetson_tx_us;
    telemetry.espnow_channel = sweeping_ ? 0 : channel_;
    const uint8_t count = send_history_count;
    const uint32_t mask = count >= 32 ? 0xFFFFFFFFu : ((1u << count) - 1u);
    telemetry.espnow_lq =
        count == 0 ? 0
                   : static_cast<uint8_t>(
                         100 * __builtin_popcount(send_ack_history_newest_lsb & mask) / count);
    telemetry.telemetry_seq = telemetry_seq_++;
    tx_telemetry_++;
    send_frame(reply_mac_, MSG_TELEMETRY, &telemetry, sizeof(telemetry));
}

void EspNowLink::load_paired() {
    Preferences prefs;
    if (prefs.begin("espnow_pair", true)) {
        const uint8_t count = prefs.getUChar("count", 0);
        if (count <= MAX_PAIRED_DONGLES &&
            prefs.getBytes("macs", paired_, sizeof(paired_)) == sizeof(paired_)) {
            paired_count_ = count;
        }
        prefs.end();
    }
    publish_allowed();
}

void EspNowLink::store_paired() {
    Preferences prefs;
    prefs.begin("espnow_pair", false);
    prefs.putUChar("count", paired_count_);
    prefs.putBytes("macs", paired_, sizeof(paired_));
    prefs.end();
}

void EspNowLink::publish_allowed() {
    allowed_count.store(0, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_seq_cst);
    size_t n = 0;
    for (uint8_t i = 0; i < paired_count_; i++) memcpy(allowed_macs[n++], paired_[i], 6);
    allowed_count.store(static_cast<uint8_t>(n), std::memory_order_release);
}

uint8_t EspNowLink::pair(const uint8_t *mac, bool armed) {
    uint8_t result = PAIR_OK;
    switch (
        pair_decision::decide(armed, mac[0], is_dongle(mac), paired_count_, MAX_PAIRED_DONGLES)) {
        case pair_decision::Decision::REFUSE_ARMED:
            result = PAIR_ARMED;
            break;
        case pair_decision::Decision::REFUSE_BAD_MAC:
            result = PAIR_BAD_MAC;
            break;
        case pair_decision::Decision::REFUSE_EXISTS:
            result = PAIR_EXISTS;
            break;
        case pair_decision::Decision::REFUSE_FULL:
            result = PAIR_FULL;
            break;
        case pair_decision::Decision::ADD:
            memcpy(paired_[paired_count_++], mac, 6);
            store_paired();
            publish_allowed();
            if (begin_ok_) add_or_update_peer(mac, ifidx_, channel_);
            break;
    }
    last_pair_result_ = result;
    return result;
}

uint8_t EspNowLink::unpair(const uint8_t *mac, bool armed) {
    uint8_t result = PAIR_NOT_FOUND;
    if (armed) {
        result = PAIR_ARMED;
    } else {
        for (uint8_t i = 0; i < paired_count_; i++) {
            if (memcmp(paired_[i], mac, 6) != 0) continue;
            memmove(paired_[i], paired_[i + 1], (paired_count_ - i - 1) * 6);
            paired_count_--;
            memset(paired_[paired_count_], 0, 6);
            store_paired();
            publish_allowed();
            if (begin_ok_ && esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
            result = PAIR_OK;
            break;
        }
    }
    last_pair_result_ = result;
    return result;
}

void EspNowLink::fill_status(espnow_status_t &out, uint32_t now_ms) const {
    out.esp_now_init_ok = begin_ok_;
    out.channel = channel_;
    out.sweeping = sweeping_;
    out.last_good_channel = last_good_channel_;
    out.heard_dongle = heard_dongle_;
    memcpy(out.dongle_mac, reply_mac_, 6);
    out.last_allowed_frame_ms = last_valid_frame_ms_;
    out.rx_allowed_frames = rx_frames_;
    out.rx_commands_new = rx_commands_new_;
    out.rx_commands_repeat = rx_commands_repeat_;
    out.rx_commands_late_dropped = rx_commands_old_;
    out.rx_pings = rx_pings_;
    out.rx_tunables = rx_tunables_;
    out.rx_ota = rx_ota_;
    out.rx_wrong_length = rx_size_errors_;
    out.rx_unpaired_mac = rx_rejected.load(std::memory_order_relaxed);
    memcpy(out.rejected_mac, rejected_mac, 6);
    out.rx_queue_full_drops = rx_queue_full.load(std::memory_order_relaxed);
    out.tx_telemetry = tx_telemetry_;
    out.tx_acked = tx_ok.load(std::memory_order_relaxed);
    out.tx_no_ack = tx_fail.load(std::memory_order_relaxed);
    out.tx_send_refused = tx_errors.load(std::memory_order_relaxed);
    const uint8_t count = send_history_count;
    const uint32_t mask = count >= 32 ? 0xFFFFFFFFu : ((1u << count) - 1u);
    out.lq = count == 0 ? 0
                        : static_cast<uint8_t>(
                              100 * __builtin_popcount(send_ack_history_newest_lsb & mask) / count);
    out.command_fresh = tracker_.fresh(now_ms);
    out.command_seq = tracker_.latest().seq;
    out.command_age_ms = tracker_.age_ms(now_ms);
    out.command_a = tracker_.latest().ch0_value;
    out.command_b = tracker_.latest().ch1_value;
    out.ota_state = ota_state_;
    out.validated = validated_;
    out.allowed_count = allowed_count.load(std::memory_order_acquire);
    for (uint8_t i = 0; i < out.allowed_count; i++) memcpy(out.allowed[i], allowed_macs[i], 6);
    out.pair_result = static_cast<pair_result_t>(last_pair_result_);
}

void EspNowLink::send_frame(const uint8_t *mac, uint8_t type, const void *payload, size_t length) {
    uint8_t frame[AIR_MAX_FRAME];
    const size_t size =
        encode_air_frame(type, static_cast<const uint8_t *>(payload), length, frame);
    if (size == 0) return;
    if (esp_now_send(mac, frame, size) != ESP_OK) tx_errors.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace espnow_link
