#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace auto_battlebot {

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

    std::vector<uint8_t> read_available();

   private:
    int fd_ = -1;
};

std::optional<std::string> find_opentx_stm32_cdc_device();

}  // namespace auto_battlebot
