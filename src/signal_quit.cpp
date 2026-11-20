#include "signal_quit.hpp"

#include <atomic>
#include <csignal>

namespace auto_battlebot {
namespace {

std::atomic<Quittable *> g_target{nullptr};

static_assert(std::atomic<Quittable *>::is_always_lock_free);

void forward_signal(int) {
    Quittable *target = g_target.load();
    if (target) target->quit_from_signal_handler();
}

}  // namespace

ScopedSignalQuit::ScopedSignalQuit(Quittable &target) {
    g_target.store(&target);
    std::signal(SIGINT, forward_signal);
    std::signal(SIGTERM, forward_signal);
}

ScopedSignalQuit::~ScopedSignalQuit() {
    std::signal(SIGINT, SIG_DFL);
    std::signal(SIGTERM, SIG_DFL);
    g_target.store(nullptr);
}

}  // namespace auto_battlebot
