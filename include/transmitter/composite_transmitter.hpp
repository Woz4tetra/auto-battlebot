#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

class CompositeTransmitter : public TransmitterInterface {
   public:
    using TimeSource = std::function<std::chrono::steady_clock::time_point()>;

    explicit CompositeTransmitter(std::vector<std::shared_ptr<TransmitterInterface>> children,
                                  std::vector<double> transmit_rate_hz = {},
                                  TimeSource now = std::chrono::steady_clock::now);

    bool initialize() override;
    CommandFeedback update() override;
    void send(VelocityCommand command) override;
    bool did_init_button_press() override;
    TransmitterStatus get_status() const override { return children_.front()->get_status(); }
    void enable() override;
    void disable() override;

   private:
    std::vector<std::shared_ptr<TransmitterInterface>> children_;
    std::vector<std::optional<std::chrono::steady_clock::duration>> transmit_period_;
    std::vector<std::optional<std::chrono::steady_clock::time_point>> last_send_;
    std::vector<std::optional<VelocityCommand>> owed_command_;
    TimeSource now_;

    void flush_child_if_due(size_t i, std::chrono::steady_clock::time_point now, bool force);
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
};

}  // namespace auto_battlebot
