#pragma once

#include <math.h>
#include <stdint.h>

namespace ina238_id {
struct SamplerConfig {
    uint32_t sample_interval_us = 10000;
    uint32_t retry_interval_us = 1000000;
    float volts_per_lsb = 3.125e-3f;
    float amps_per_lsb = 5e-6f / 0.0005f;
};

enum class SampleResult : uint8_t { SKIPPED, READ_OK, READ_FAILED };

class Sampler {
   public:
    explicit Sampler(const SamplerConfig &config = SamplerConfig()) : _config(config) {}

    void begin(bool present, uint32_t now_us) {
        _present = present;
        _volts = NAN;
        _amps = NAN;
        _next_interval_us = _config.sample_interval_us;
        _timer_us = now_us - _config.sample_interval_us;
    }

    template <class ReadRegister>
    SampleResult update(uint32_t now_us, ReadRegister read_register) {
        if (!_present) return SampleResult::SKIPPED;
        if (now_us - _timer_us < _next_interval_us) return SampleResult::SKIPPED;
        _timer_us = now_us;
        uint16_t bus_raw = 0;
        uint16_t shunt_raw = 0;
        if (!read_register(BUS_REGISTER, &bus_raw)) {
            _volts = NAN;
            _amps = NAN;
            _next_interval_us = _config.retry_interval_us;
            return SampleResult::READ_FAILED;
        }
        _volts = static_cast<float>(static_cast<int16_t>(bus_raw)) * _config.volts_per_lsb;
        _amps = read_register(SHUNT_REGISTER, &shunt_raw)
                    ? static_cast<float>(static_cast<int16_t>(shunt_raw)) * _config.amps_per_lsb
                    : NAN;
        _next_interval_us = _config.sample_interval_us;
        return SampleResult::READ_OK;
    }

    float volts() const { return _volts; }
    float amps() const { return _amps; }

    static constexpr uint8_t SHUNT_REGISTER = 0x04;
    static constexpr uint8_t BUS_REGISTER = 0x05;

   private:
    SamplerConfig _config;
    bool _present = false;
    float _volts = NAN;
    float _amps = NAN;
    uint32_t _timer_us = 0;
    uint32_t _next_interval_us = 0;
};
}  // namespace ina238_id
