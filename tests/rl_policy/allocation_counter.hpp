#pragma once

namespace auto_battlebot::rl_test {

extern thread_local bool counting_allocations;
extern thread_local long allocation_count;

template <typename F>
long allocations_during(F &&work) {
    allocation_count = 0;
    counting_allocations = true;
    work();
    counting_allocations = false;
    return allocation_count;
}

}  // namespace auto_battlebot::rl_test
