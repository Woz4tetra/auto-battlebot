#pragma once
#include <Arduino.h>
#include <tuning.h>

class TuningStore {
   public:
    void begin();

    tuning::Settings get();
    bool set(const char *key, float value);
    bool take_change(tuning::Settings &out);
    bool save(bool armed);
    bool saved();

   private:
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    tuning::Settings current_;
    tuning::Settings stored_;
    bool changed_ = false;
};
