#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace auto_battlebot {

/** Minimal POSIX serial port wrapper (115200 8N1, non-blocking reads). */
class SerialPort {
   public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& path, int baud_rate = 115200);
    void close();
    bool is_open() const { return fd_ >= 0; }

    bool write(const std::string& data);
    bool write(const uint8_t* data, size_t length);

    /** Read all currently available bytes (non-blocking). Returns empty on error/no data. */
    std::vector<uint8_t> read_available();

   private:
    int fd_ = -1;
};

/**
 * Scan sysfs to find a USB serial device with VID=0x0483, PID=0x5740 (OpenTX/STM32 CDC).
 * Returns the /dev path (e.g. "/dev/ttyACM0") if found.
 */
std::optional<std::string> find_opentx_device();

}  // namespace auto_battlebot
