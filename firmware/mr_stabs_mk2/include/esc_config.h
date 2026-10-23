#pragma once
#include <Arduino.h>
#include <blheli_boot.h>
#include <esc.h>

// ESC tuning mode (docs/plans/esc_settings_dashboard_plan.md). Stops DShot, hands both ESC
// signal wires to the BLHeli bootloader link, and relays 4-way frames posted over HTTP. The robot
// never decodes settings: the diagnostics page and firmware/blheli/blheli32.py --url do.
//
// ESC indices follow the passthrough: 0 = A2, the right ESC; 1 = A3, the left ESC.
namespace esc_config {

enum class State : uint8_t { COMBAT, TUNING, EXITING };

// After this long with no frame, both ESCs are reset out of their bootloader. Tuning mode stays.
const uint32_t IDLE_RELEASE_MS = 60000;
// EXITING sends zero-throttle DShot this long before control comes back, as setup() does.
const uint32_t WAKE_MS = 2000;

struct status_t {
    State state;
    const char *error;  // last refusal or failure, "" when none
    uint32_t frames;    // relayed since boot
    uint32_t guard_refusals;
    uint32_t idle_ms;  // since the last frame or entry, in TUNING
    bool released;     // the idle timeout reset both ESCs
    bool pending;      // a frame is queued
    bool armed;        // as of the latest update()
    bool driving;      // ESP-NOW drove the latest tick
};

enum class Poll : uint8_t { NONE, PENDING, DONE };

class EscConfig {
   public:
    EscConfig();
    // right and left in passthrough order: right is ESC 0 (A2), left is ESC 1 (A3).
    void begin(esc::Esc *right, esc::Esc *left, uint8_t right_pin, uint8_t left_pin);
    // Called by loop() every pass. armed and driving refuse entry; a radio button press exits.
    // Runs at most one queued frame, which blocks for up to about 3 s (a write).
    void update(uint32_t now_ms, bool armed, bool driving, bool button_pressed);
    // Not COMBAT: control and the motor outputs are off.
    bool active() const { return _state != State::COMBAT; }
    State state() const { return _state; }

    // HTTP side, safe from the network task.
    void request(bool enter);
    // Queues one 4-way request frame. False with a reason when not in TUNING or one is pending.
    bool submit(const uint8_t *frame, size_t size, uint32_t &id, const char *&reason);
    // The latest frame's state; out gets the response frame when DONE (MAX_RESPONSE bytes).
    Poll poll(uint32_t &id, uint8_t *out, size_t &size);
    status_t status();

   private:
    class SerialLink;
    SerialLink *_link = nullptr;
    blheli_boot::Server *_server = nullptr;
    esc::Esc *_escs[2] = {nullptr, nullptr};
    volatile State _state = State::COMBAT;
    uint32_t _exit_started_ms = 0;
    uint32_t _last_activity_ms = 0;
    bool _released = false;
    bool _prev_button = false;

    portMUX_TYPE _mux;
    // Guarded by _mux.
    int8_t _requested = -1;  // -1 none, 0 exit, 1 enter
    const char *_error = "";
    uint32_t _frames = 0;
    bool _armed = false;
    bool _driving = false;
    uint32_t _next_id = 1;
    uint32_t _id = 0;
    Poll _slot_state = Poll::NONE;
    uint8_t _request[blheli_boot::MAX_REQUEST];
    size_t _request_size = 0;
    uint8_t _response[blheli_boot::MAX_RESPONSE];
    size_t _response_size = 0;

    void enter(uint32_t now_ms);
    void start_exit(uint32_t now_ms);
    void set_error(const char *error);
};

const char *state_name(State state);

}  // namespace esc_config
