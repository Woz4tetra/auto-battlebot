#pragma once
#include <Arduino.h>
#include <blheli_boot.h>
#include <esc.h>

namespace esc_config {

enum class State : uint8_t { COMBAT, TUNING, EXITING };

const uint32_t IDLE_BOOTLOADER_RELEASE_MS = 60000;
const uint32_t ESC_WAKE_MS = 2000;
const uint32_t SETTLE_MS = 1000;
const uint32_t REENTRY_MS = 3000;

struct status_t {
    State state;
    const char *last_error;
    uint32_t frames_relayed;
    uint32_t guard_refusals;
    uint32_t idle_ms_in_tuning;
    bool idle_released;
    bool frame_pending;
    bool armed_last_update;
    bool espnow_driving;
    char last_failure[96];
};

enum class Poll : uint8_t { NONE, PENDING, DONE };

class EscConfig {
   public:
    EscConfig();
    void begin(esc::Esc *esc0_right, esc::Esc *esc1_left, uint8_t esc0_pin, uint8_t esc1_pin);
    void update(uint32_t now_ms, bool armed, bool driving, bool button_pressed);
    bool owns_escs() const { return _state != State::COMBAT; }
    State state() const { return _state; }

    void request(bool enter);
    bool submit(const uint8_t *frame, size_t size, uint32_t &id, const char *&reason);
    Poll poll(uint32_t &id, uint8_t (&out)[blheli_boot::MAX_RESPONSE], size_t &size);
    status_t status();

   private:
    class SerialLink;
    SerialLink *_link = nullptr;
    blheli_boot::Server *_server = nullptr;
    esc::Esc *_escs[2] = {nullptr, nullptr};
    volatile State _state = State::COMBAT;
    uint32_t _exit_started_ms = 0;
    uint32_t _entered_ms = 0;
    uint32_t _combat_since_ms = 0;
    uint32_t _logged_resyncs = 0;
    uint32_t _logged_reboots = 0;
    uint32_t _last_activity_ms = 0;
    bool _released = false;
    bool _prev_button = false;

    enum class Request : uint8_t { NONE, EXIT, ENTER };

    struct Guarded {
        Request requested = Request::NONE;
        const char *error = "";
        char failure[96] = "";
        uint32_t frames = 0;
        bool armed = false;
        bool driving = false;
        uint32_t next_id = 1;
        uint32_t id = 0;
        Poll slot_state = Poll::NONE;
        uint8_t request[blheli_boot::MAX_REQUEST];
        size_t request_size = 0;
        uint8_t response[blheli_boot::MAX_RESPONSE];
        size_t response_size = 0;
    };

    portMUX_TYPE _mux;
    Guarded _guarded;

    void enter(uint32_t now_ms);
    void start_exit(uint32_t now_ms);
    void set_error(const char *error);
};

const char *state_name(State state);

}  // namespace esc_config
