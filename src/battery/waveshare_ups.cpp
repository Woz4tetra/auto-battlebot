#include "battery/waveshare_ups.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <string>

namespace auto_battlebot {
namespace {

// INA219 register addresses from datasheet.
constexpr uint8_t INA219_REG_CONFIG = 0x00;
constexpr uint8_t INA219_REG_BUSVOLTAGE = 0x02;
constexpr uint8_t INA219_REG_POWER = 0x03;
constexpr uint8_t INA219_REG_CURRENT = 0x04;
constexpr uint8_t INA219_REG_CALIBRATION = 0x05;
// Calibration register value for the selected profile (16V bus range, up to ~5A expected current).
// Derived with Cal = trunc(0.04096 / (Current_LSB * Rshunt)).
constexpr uint16_t INA219_CALIBRATION_16V_5A = 26868;
// Config register bits matching the Waveshare reference setup (16V range, gain/ADC, continuous
// mode).
constexpr uint16_t INA219_CONFIG_16V_5A_CONTINUOUS = 0x0EEF;
// Datasheet constant used in the INA219 calibration equation.
constexpr double INA219_CALIBRATION_NUMERATOR = 0.04096;
// Shunt resistor value on the UPS board; used with calibration to convert raw current counts.
constexpr double INA219_SHUNT_RESISTANCE_OHM = 0.01;
// Current LSB derived from calibration: Current_LSB = 0.04096 / (Cal * Rshunt).
constexpr double INA219_CURRENT_LSB_A =
    INA219_CALIBRATION_NUMERATOR / (INA219_CALIBRATION_16V_5A * INA219_SHUNT_RESISTANCE_OHM);
// INA219 power register uses Power_LSB = 20 * Current_LSB.
constexpr double INA219_POWER_CURRENT_LSB_MULTIPLIER = 20.0;
constexpr double INA219_POWER_LSB_W = INA219_POWER_CURRENT_LSB_MULTIPLIER * INA219_CURRENT_LSB_A;
constexpr double kBatteryReadWarnMs = 30.0;

bool write_i2c_register_u16(int fd, uint8_t reg, uint16_t value) {
    uint8_t payload[3] = {reg, static_cast<uint8_t>((value >> 8) & 0xFF),
                          static_cast<uint8_t>(value & 0xFF)};
    return write(fd, payload, sizeof(payload)) == static_cast<ssize_t>(sizeof(payload));
}

bool read_i2c_register_u16(int fd, uint8_t reg, uint16_t &value) {
    uint8_t reg_addr = reg;
    if (write(fd, &reg_addr, sizeof(reg_addr)) != static_cast<ssize_t>(sizeof(reg_addr))) {
        return false;
    }
    uint8_t data[2] = {0, 0};
    if (read(fd, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data))) {
        return false;
    }
    value = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    return true;
}

}  // namespace

bool read_waveshare_ups_sample(const BatteryOptions &options, BatterySample &sample_out) {
    const auto read_start = std::chrono::steady_clock::now();
    auto finish = [&](bool ok) {
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - read_start)
                .count();
        if (elapsed_ms > kBatteryReadWarnMs) {
            spdlog::warn("Waveshare UPS read slow: elapsed_ms={:.2f} ok={}", elapsed_ms, ok);
        }
        return ok;
    };

    const int battery_i2c_bus = options.i2c_bus;
    const int battery_i2c_address = options.i2c_address;
    const std::string dev_path = "/dev/i2c-" + std::to_string(battery_i2c_bus);
    int fd = open(dev_path.c_str(), O_RDWR);
    if (fd < 0) return finish(false);
    if (ioctl(fd, I2C_SLAVE, battery_i2c_address) < 0) {
        close(fd);
        return finish(false);
    }

    bool ok = write_i2c_register_u16(fd, INA219_REG_CALIBRATION, INA219_CALIBRATION_16V_5A);
    if (ok) {
        ok = write_i2c_register_u16(fd, INA219_REG_CONFIG, INA219_CONFIG_16V_5A_CONTINUOUS);
    }
    uint16_t raw_bus_voltage = 0;
    uint16_t raw_current = 0;
    uint16_t raw_power = 0;
    if (ok) ok = read_i2c_register_u16(fd, INA219_REG_BUSVOLTAGE, raw_bus_voltage);
    if (ok) ok = read_i2c_register_u16(fd, INA219_REG_CURRENT, raw_current);
    if (ok) ok = read_i2c_register_u16(fd, INA219_REG_POWER, raw_power);
    close(fd);
    if (!ok) return finish(false);

    const int16_t current_signed = static_cast<int16_t>(raw_current);
    const int16_t power_signed = static_cast<int16_t>(raw_power);
    sample_out.timestamp = std::chrono::steady_clock::now();
    sample_out.raw_bus_voltage = raw_bus_voltage;
    sample_out.raw_current = current_signed;
    sample_out.raw_power = power_signed;
    sample_out.bus_voltage_v = static_cast<double>(raw_bus_voltage >> 3U) * 0.004;
    sample_out.current_a = static_cast<double>(current_signed) * INA219_CURRENT_LSB_A;
    sample_out.power_w = static_cast<double>(power_signed) * INA219_POWER_LSB_W;
    sample_out.valid = true;
    return finish(true);
}

}  // namespace auto_battlebot
