#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

/**
 * Drives several transmitters from one command stream.
 *
 * send(), enable(), and disable() fan out to every child, and update() polls every child so their
 * receive paths drain. The first child is primary and owns everything read-side: status, behavior
 * mode, and command feedback on a key collision. Non-primary children are accelerators, so their
 * initialize() failures are logged and ignored: a dead ESP-NOW dongle costs latency, not control.
 *
 * No command preprocessing here. Each child does its own scaling, and the rate limit that makes two
 * transports agree lives in the firmware, after the arbitration that picks between them.
 *
 * Each child can carry a transmit rate cap. The control loop sends at 250 Hz; the OpenTX trainer
 * path feeds a 150 Hz Crossfire link through serial CLI writes and gains nothing past that, while
 * ESP-NOW takes every cycle. A capped child gets the newest command once its period has passed.
 * A command it skips is held and delivered when the period runs out, from update() if no newer
 * send() comes, and before disable(): the final stop command always lands.
 */
class CompositeTransmitter : public TransmitterInterface {
   public:
    using TimeSource = std::function<std::chrono::steady_clock::time_point()>;

    /** transmit_rate_hz is per child, in the same order; empty or 0 means every send(). The
     *  time source is for tests: pacing is a hardware rate, so it runs on the steady clock. */
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
    /** Zero for an uncapped child. */
    std::vector<std::chrono::steady_clock::duration> transmit_period_;
    /** Empty until a child's first send, which always goes through. */
    std::vector<std::optional<std::chrono::steady_clock::time_point>> last_send_;
    /** A command a capped child skipped, still owed to it. */
    std::vector<std::optional<VelocityCommand>> pending_;
    TimeSource now_;

    /** Sends child i its pending command if its period has passed, or unconditionally. */
    void deliver_pending(size_t i, std::chrono::steady_clock::time_point now, bool force);
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
};

}  // namespace auto_battlebot
