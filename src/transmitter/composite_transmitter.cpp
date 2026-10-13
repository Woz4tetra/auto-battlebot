#include "transmitter/composite_transmitter.hpp"

#include <stdexcept>

#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {

CompositeTransmitter::CompositeTransmitter(
    std::vector<std::shared_ptr<TransmitterInterface>> children,
    std::vector<double> transmit_rate_hz, TimeSource now)
    : children_(std::move(children)),
      transmit_period_(children_.size(), std::chrono::steady_clock::duration::zero()),
      last_send_(children_.size()),
      pending_(children_.size()),
      now_(std::move(now)),
      logger_(DiagnosticsLogger::get_logger("composite_transmitter")) {
    if (children_.empty()) {
        throw std::invalid_argument("CompositeTransmitter requires at least one child");
    }
    if (!transmit_rate_hz.empty() && transmit_rate_hz.size() != children_.size()) {
        throw std::invalid_argument("CompositeTransmitter needs one transmit rate per child");
    }
    for (size_t i = 0; i < transmit_rate_hz.size(); i++) {
        if (transmit_rate_hz[i] > 0.0) {
            transmit_period_[i] = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / transmit_rate_hz[i]));
        }
    }
}

bool CompositeTransmitter::initialize() {
    bool primary_ok = false;
    for (size_t i = 0; i < children_.size(); i++) {
        const bool ok = children_[i]->initialize();
        if (i == 0) {
            primary_ok = ok;
        } else if (!ok) {
            logger_->warning("child_initialize_failed", {{"index", static_cast<int>(i)}});
        }
    }
    return primary_ok;
}

CommandFeedback CompositeTransmitter::update() {
    const auto now = now_();
    for (size_t i = 0; i < children_.size(); i++) deliver_pending(i, now, false);

    CommandFeedback merged;
    // Reverse order so the primary is applied last and wins on a key collision.
    for (auto it = children_.rbegin(); it != children_.rend(); ++it) {
        for (const auto &[frame_id, command] : (*it)->update().stick_commands) {
            merged.stick_commands[frame_id] = command;
        }
    }
    return merged;
}

void CompositeTransmitter::send(VelocityCommand command) {
    const auto now = now_();
    for (size_t i = 0; i < children_.size(); i++) {
        pending_[i] = command;
        deliver_pending(i, now, false);
    }
}

void CompositeTransmitter::deliver_pending(size_t i, std::chrono::steady_clock::time_point now,
                                           bool force) {
    if (!pending_[i]) return;
    if (!force && last_send_[i] && now - *last_send_[i] < transmit_period_[i]) return;
    last_send_[i] = now;
    children_[i]->send(*pending_[i]);
    pending_[i].reset();
}

bool CompositeTransmitter::did_init_button_press() {
    // No short-circuit: every child is polled so its edge latch clears.
    bool pressed = false;
    for (const auto &child : children_) {
        if (child->did_init_button_press()) pressed = true;
    }
    return pressed;
}

void CompositeTransmitter::enable() {
    for (const auto &child : children_) child->enable();
}

void CompositeTransmitter::disable() {
    const auto now = now_();
    for (size_t i = 0; i < children_.size(); i++) {
        deliver_pending(i, now, true);
        children_[i]->disable();
    }
}

}  // namespace auto_battlebot
