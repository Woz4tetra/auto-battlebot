#include <Preferences.h>
#include <tuning_store.h>

static const char *PREFS_NAMESPACE = "tuning";

void TuningStore::begin() {
    tuning::Settings loaded;
    Preferences prefs;
    if (prefs.begin(PREFS_NAMESPACE, true)) {
        for (size_t i = 0; i < tuning::FIELD_COUNT; i++) {
            const tuning::Field &field = tuning::FIELDS[i];
            if (!prefs.isKey(field.key)) continue;
            tuning::set(loaded, field.key, prefs.getFloat(field.key, tuning::get(loaded, field)));
        }
        prefs.end();
    }
    portENTER_CRITICAL(&mux_);
    current_ = stored_ = loaded;
    changed_ = true;
    portEXIT_CRITICAL(&mux_);
}

tuning::Settings TuningStore::get() {
    portENTER_CRITICAL(&mux_);
    tuning::Settings out = current_;
    portEXIT_CRITICAL(&mux_);
    return out;
}

bool TuningStore::set(const char *key, float value) {
    portENTER_CRITICAL(&mux_);
    bool ok = tuning::set(current_, key, value);
    if (ok) changed_ = true;
    portEXIT_CRITICAL(&mux_);
    return ok;
}

bool TuningStore::take_change(tuning::Settings &out) {
    portENTER_CRITICAL(&mux_);
    bool changed = changed_;
    changed_ = false;
    out = current_;
    portEXIT_CRITICAL(&mux_);
    return changed;
}

void TuningStore::save() {
    tuning::Settings now = get();
    Preferences prefs;
    bool open = false;
    for (size_t i = 0; i < tuning::FIELD_COUNT; i++) {
        const tuning::Field &field = tuning::FIELDS[i];
        float value = tuning::get(now, field);
        if (value == tuning::get(stored_, field)) continue;
        if (!open) open = prefs.begin(PREFS_NAMESPACE, false);
        if (!open) return;
        prefs.putFloat(field.key, value);
        tuning::set(stored_, field.key, value);
        Serial.printf("tuning: saved %s = %g\n", field.key, value);
    }
    if (open) prefs.end();
}

bool TuningStore::saved() {
    tuning::Settings now = get();
    for (size_t i = 0; i < tuning::FIELD_COUNT; i++) {
        if (tuning::get(now, tuning::FIELDS[i]) != tuning::get(stored_, tuning::FIELDS[i])) {
            return false;
        }
    }
    return true;
}
