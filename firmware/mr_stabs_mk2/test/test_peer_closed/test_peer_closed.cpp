#include <gtest/gtest.h>
#include <peer_closed.h>
#include <unistd.h>

using peer_closed::State;

namespace {

struct Pair {
    int fds[2];
    Pair() { EXPECT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0); }
    ~Pair() {
        close(fds[0]);
        close(fds[1]);
    }
};

}  // namespace

TEST(PeerClosed, IdleOpenPeerIsOpen) {
    Pair p;
    EXPECT_EQ(peer_closed::probe(p.fds[0], nullptr), State::OPEN);
}

TEST(PeerClosed, PendingDataIsOpenAndNotConsumed) {
    Pair p;
    ASSERT_EQ(write(p.fds[1], "x", 1), 1);
    EXPECT_EQ(peer_closed::probe(p.fds[0], nullptr), State::OPEN);
    char byte = 0;
    EXPECT_EQ(read(p.fds[0], &byte, 1), 1);
    EXPECT_EQ(byte, 'x');
}

TEST(PeerClosed, OrderlyCloseIsDetected) {
    Pair p;
    shutdown(p.fds[1], SHUT_WR);
    EXPECT_EQ(peer_closed::probe(p.fds[0], nullptr), State::CLOSED);
}

TEST(PeerClosed, InvalidFdIsClosed) { EXPECT_EQ(peer_closed::probe(-1, nullptr), State::CLOSED); }

TEST(PeerClosed, BadDescriptorReportsErrno) {
    int error = 0;
    EXPECT_EQ(peer_closed::probe(1000, &error), State::ERROR);
    EXPECT_EQ(error, EBADF);
}

TEST(PeerClosed, ZeroLengthRecvCannotSeeOrderlyClose) {
    Pair p;
    shutdown(p.fds[1], SHUT_WR);
    char byte;
    errno = 0;
    EXPECT_EQ(recv(p.fds[0], &byte, 0, MSG_DONTWAIT), 0);
    EXPECT_EQ(errno, 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
