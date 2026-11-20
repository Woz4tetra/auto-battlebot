#pragma once

#include <errno.h>
#include <sys/socket.h>

namespace peer_closed {

enum class State { OPEN, CLOSED, ERROR };

inline State probe(int fd, int *error_out) {
    if (fd < 0) return State::CLOSED;
    char byte;
    const ssize_t n = recv(fd, &byte, 1, MSG_PEEK | MSG_DONTWAIT);
    if (n == 0) return State::CLOSED;
    if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
        if (error_out != nullptr) *error_out = errno;
        return State::ERROR;
    }
    return State::OPEN;
}

}  // namespace peer_closed
