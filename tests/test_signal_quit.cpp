#include <gtest/gtest.h>

#include <atomic>
#include <csignal>

#include "signal_quit.hpp"

namespace auto_battlebot {
namespace {

class CountingQuittable : public Quittable {
   public:
    void quit_from_signal_handler() override { quits.fetch_add(1); }
    std::atomic<int> quits{0};
};

bool has_default_disposition(int signal_number) {
    struct sigaction current {};
    sigaction(signal_number, nullptr, &current);
    return current.sa_handler == SIG_DFL;
}

TEST(SignalQuitTest, SigintAndSigtermReachTheQuittable) {
    CountingQuittable target;
    ScopedSignalQuit scope(target);

    std::raise(SIGINT);
    EXPECT_EQ(target.quits.load(), 1);
    std::raise(SIGTERM);
    EXPECT_EQ(target.quits.load(), 2);
}

TEST(SignalQuitTest, LeavingScopeRestoresTheDefaultDisposition) {
    CountingQuittable target;
    {
        ScopedSignalQuit scope(target);
        EXPECT_FALSE(has_default_disposition(SIGINT));
        EXPECT_FALSE(has_default_disposition(SIGTERM));
    }
    EXPECT_TRUE(has_default_disposition(SIGINT));
    EXPECT_TRUE(has_default_disposition(SIGTERM));
}

}  // namespace
}  // namespace auto_battlebot
