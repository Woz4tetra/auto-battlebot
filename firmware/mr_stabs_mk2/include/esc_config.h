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

const uint32_t IDLE_BOOTLOADER_RELEASE_MS = 60000;
const uint32_t ESC_WAKE_MS = 2000;
const uint32_t SETTLE_MS = 1000;
// Entry waits until COMBAT has lasted this long. Exiting restarts the ESCs, and entering again
// right away left them out of the bootloader 3 times in 8; after 1 s of DShot, 0 in 14. Measured
// on 2026-10-23 with the earlier software serial link.
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
    // Called by loop() every pass. armed and driving refuse entry; a radio button press exits.
    // Runs at most one queued frame, which blocks for up to about 3 s (a write).
    void update(uint32_t now_ms, bool armed, bool driving, bool button_pressed);
    bool owns_escs() const { return _state != State::COMBAT; }
    State state() const { return _state; }

    // HTTP side, safe from the network task.
    void request(bool enter);
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
    uint32_t _entered_ms = 0;
    uint32_t _combat_since_ms = 0;  // boot counts as the start of COMBAT
    uint32_t _logged_resyncs = 0;
    uint32_t _logged_reboots = 0;
    uint32_t _last_activity_ms = 0;
    bool _released = false;
    bool _prev_button = false;

    portMUX_TYPE _mux;
    // Guarded by _mux.
    int8_t _requested = -1;  // -1 none, 0 exit, 1 enter
    const char *_error = "";
    char _failure[96] = "";
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
