#pragma once
#include <Arduino.h>
#include <tuning.h>

// The runtime settings (lib/tuning) shared by the tasks that change them and the loop that
// uses them. The diagnostics page writes from the web server task and ESP-NOW from loop(), so
// every access copies under a spinlock. loop() applies a change on its next pass and saves it to
// NVS only while disarmed, since a flash write stalls both cores for a few ms; a change made
// while armed is saved at the next disarm, and lost if the robot powers off first.
class TuningStore {
   public:
    // Loads saved values over the defaults. A saved value outside its field's range keeps the
    // default.
    void begin();

    tuning::Settings get();
    // True when `key` names a field and `value` is in its range.
    bool set(const char *key, float value);
    // True once per change since the last call, with the settings to apply.
    bool take_change(tuning::Settings &out);
    // Writes changed values to NVS. Call from loop() while disarmed.
    void save();
    // False while a change is waiting for save().
    bool saved();

   private:
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    tuning::Settings current_;
    tuning::Settings stored_;
    bool changed_ = false;
};
