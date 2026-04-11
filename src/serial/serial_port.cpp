#include "serial/serial_port.hpp"

#include <fcntl.h>
#include <limits.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <string>

namespace auto_battlebot {

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& path, int baud_rate) {
    close();

    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    speed_t speed;
    switch (baud_rate) {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        default:
            speed = B115200;
            break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, no flow control, raw mode
    cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;

    // Non-blocking: return immediately with whatever data is available
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::write(const std::string& data) {
    return write(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

bool SerialPort::write(const uint8_t* data, size_t length) {
    if (fd_ < 0 || length == 0) return false;
    ssize_t written = ::write(fd_, data, length);
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        close();
    }
    return written == static_cast<ssize_t>(length);
}

std::vector<uint8_t> SerialPort::read_available() {
    if (fd_ < 0) return {};

    std::vector<uint8_t> result;
    uint8_t buf[256];
    ssize_t n;
    while ((n = ::read(fd_, buf, sizeof(buf))) > 0) {
        result.insert(result.end(), buf, buf + n);
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        close();
        return {};
    }
    return result;
}

// ---------------------------------------------------------------------------

static std::string read_sysfs_string(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::string s;
    std::getline(f, s);
    return s;
}

std::optional<std::string> find_opentx_device() {
    static constexpr uint16_t kTargetVid = 0x0483;
    static constexpr uint16_t kTargetPid = 0x5740;

    for (const char* prefix : {"ttyACM", "ttyUSB"}) {
        for (int i = 0; i < 16; ++i) {
            std::string name = std::string(prefix) + std::to_string(i);
            std::string dev_path = "/dev/" + name;

            if (access(dev_path.c_str(), F_OK) != 0) continue;

            // Resolve sysfs symlink: /sys/class/tty/<name>/device -> USB interface
            std::string sysfs_link = "/sys/class/tty/" + name + "/device";
            char real_path[PATH_MAX];
            if (realpath(sysfs_link.c_str(), real_path) == nullptr) continue;

            // Walk up the device hierarchy looking for idVendor/idProduct
            std::string current(real_path);
            for (int depth = 0; depth < 5; ++depth) {
                std::string vendor_str = read_sysfs_string(current + "/idVendor");
                std::string product_str = read_sysfs_string(current + "/idProduct");

                if (!vendor_str.empty() && !product_str.empty()) {
                    try {
                        auto vid = static_cast<uint16_t>(std::stoul(vendor_str, nullptr, 16));
                        auto pid = static_cast<uint16_t>(std::stoul(product_str, nullptr, 16));
                        if (vid == kTargetVid && pid == kTargetPid) {
                            return dev_path;
                        }
                    } catch (...) {
                    }
                    break;
                }

                auto slash = current.rfind('/');
                if (slash == std::string::npos) break;
                current = current.substr(0, slash);
            }
        }
    }
    return std::nullopt;
}

}  // namespace auto_battlebot
