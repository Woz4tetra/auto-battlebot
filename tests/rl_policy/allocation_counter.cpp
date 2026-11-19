#include "rl_policy/allocation_counter.hpp"

#include <cstddef>

extern "C" {
void *__libc_malloc(size_t size) noexcept;
void *__libc_calloc(size_t count, size_t size) noexcept;
void *__libc_realloc(void *pointer, size_t size) noexcept;
}

namespace auto_battlebot::rl_test {
thread_local bool counting_allocations = false;
thread_local long allocation_count = 0;
}  // namespace auto_battlebot::rl_test

extern "C" void *malloc(size_t size) noexcept {
    if (auto_battlebot::rl_test::counting_allocations) ++auto_battlebot::rl_test::allocation_count;
    return __libc_malloc(size);
}

extern "C" void *calloc(size_t count, size_t size) noexcept {
    if (auto_battlebot::rl_test::counting_allocations) ++auto_battlebot::rl_test::allocation_count;
    return __libc_calloc(count, size);
}

extern "C" void *realloc(void *pointer, size_t size) noexcept {
    if (auto_battlebot::rl_test::counting_allocations) ++auto_battlebot::rl_test::allocation_count;
    return __libc_realloc(pointer, size);
}
