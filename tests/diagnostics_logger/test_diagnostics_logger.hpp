#pragma once

#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {
/**
 * @brief Backend that records every snapshot batch it receives, for verifying the logger fans
 * out without a relay or recorder.
 */
class RecordingBackend : public DiagnosticsBackend {
   public:
    void receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) override {
        last_snapshots_ = snapshots;
        receive_count_++;
    }

    const std::vector<DiagnosticStatusSnapshot> &last_snapshots() const { return last_snapshots_; }
    int receive_count() const { return receive_count_; }

   private:
    std::vector<DiagnosticStatusSnapshot> last_snapshots_;
    int receive_count_ = 0;
};

/**
 * @brief Test-only subclass of DiagnosticsLogger with reset capability
 *
 * This class should ONLY be used in unit tests. It provides a reset()
 * method to clear the singleton state between tests.
 *
 * WARNING: Do not use this class in production code!
 */
class TestDiagnosticsLogger : public DiagnosticsLogger {
   public:
    /**
     * @brief Reset the singleton state for testing
     *
     * Clears all loggers and resets the initialization state.
     * This allows tests to start with a clean slate.
     */
    static void reset() {
        loggers_.clear();
        backends_.clear();
        initialized_ = false;
        test_mode_ = false;
    }

    /**
     * @brief Enable test mode to skip calling backends
     */
    static void enable_test_mode() { test_mode_ = true; }
};

}  // namespace auto_battlebot
