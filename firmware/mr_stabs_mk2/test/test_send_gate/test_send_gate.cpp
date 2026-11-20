#include <gtest/gtest.h>
#include <send_gate.h>

using namespace espnow_protocol;

TEST(SendGate, AMissingSendCallbackClearsAfterTheTimeoutSoLaterFramesGoOut) {
    SendGate gate(50);
    EXPECT_EQ(gate.poll(1000), SendPoll::READY);
    gate.start(1000);
    EXPECT_EQ(gate.poll(1049), SendPoll::BUSY);
    EXPECT_EQ(gate.poll(1050), SendPoll::CALLBACK_LOST);
    EXPECT_EQ(gate.poll(1051), SendPoll::READY);
}

TEST(SendGate, CallbackReleasesTheGateBeforeTheTimeout) {
    SendGate gate(50);
    gate.start(1000);
    gate.complete();
    EXPECT_EQ(gate.poll(1010), SendPoll::READY);
}

TEST(SendGate, TimeoutSurvivesMillisWraparound) {
    SendGate gate(50);
    gate.start(UINT32_MAX - 10);
    EXPECT_EQ(gate.poll(20), SendPoll::BUSY);
    EXPECT_EQ(gate.poll(40), SendPoll::CALLBACK_LOST);
}

TEST(UsbTxAction, ResetsTheFifoOnlyWhenTheFrameDoesNotFit) {
    EXPECT_EQ(usb_tx_action(64, 64), UsbTxAction::WRITE);
    EXPECT_EQ(usb_tx_action(63, 64), UsbTxAction::RESET_STUCK_FIFO);
    EXPECT_EQ(usb_tx_action(0, 5), UsbTxAction::RESET_STUCK_FIFO);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
