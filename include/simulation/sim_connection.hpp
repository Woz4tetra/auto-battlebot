#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "data_structures.hpp"

namespace auto_battlebot {

class SimConnection {
   public:
    static void configure(const std::string &host, int port);
    static std::shared_ptr<SimConnection> instance();

    ~SimConnection();

    SimConnection(const SimConnection &) = delete;
    SimConnection &operator=(const SimConnection &) = delete;

    bool connect();
    void disconnect();
    bool is_connected() const;

    void set_command(VelocityCommand cmd);
    bool step_and_receive(CameraData &data);

   private:
    SimConnection(const std::string &host, int port);

    bool send_all(const void *buf, size_t len);
    bool recv_all(void *buf, size_t len);

    std::string host_;
    int port_;
    int sock_fd_ = -1;
    VelocityCommand pending_command_{0.0, 0.0, 0.0};

    static std::shared_ptr<SimConnection> instance_;
};

}  // namespace auto_battlebot
