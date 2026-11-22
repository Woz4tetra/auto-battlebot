#include <espnow_protocol.h>
#include <gtest/gtest.h>

#include <string>

#include "health/command_chain.hpp"

namespace auto_battlebot {
namespace {

TEST(CommandChainTest, EspNowChainCarriesTheSlewMixerAndProtocol) {
    EspNowTransmitterConfiguration transmitter;
    transmitter.max_motor_rpm_per_s = 2000.0;
    const auto chain = resolve_command_chain(transmitter);
    ASSERT_TRUE(chain);
    EXPECT_EQ(chain->at("transmitter"), "EspNowTransmitter");
    EXPECT_EQ(chain->at("drive_processor"), "DifferentialDriveProcessor");
    EXPECT_EQ(chain->at("max_motor_rpm_per_s"), "2000");
    EXPECT_EQ(chain->at("protocol_version"), std::to_string(espnow_protocol::PROTOCOL_VERSION));
}

TEST(CommandChainTest, AbsentAndZeroSlewAreTheSameChain) {
    EspNowTransmitterConfiguration absent;
    EspNowTransmitterConfiguration zero;
    zero.max_motor_rpm_per_s = 0.0;
    EXPECT_EQ(command_chain_hash(*resolve_command_chain(absent)),
              command_chain_hash(*resolve_command_chain(zero)));
}

TEST(CommandChainTest, OpenTxHasNoEspNowProtocol) {
    const auto chain = resolve_command_chain(OpenTxTransmitterConfiguration{});
    ASSERT_TRUE(chain);
    EXPECT_EQ(chain->at("transmitter"), "OpenTxTransmitter");
    EXPECT_EQ(chain->at("protocol_version"), "none");
}

TEST(CommandChainTest, TransmittersThatDriveNoRobotHaveNoChain) {
    EXPECT_FALSE(resolve_command_chain(PlaybackTransmitterConfiguration{}));
    EXPECT_FALSE(resolve_command_chain(SimTransmitterConfiguration{}));
    EXPECT_FALSE(resolve_command_chain(NoopTransmitterConfiguration{}));
}

TEST(CommandChainTest, HashIsStableAndChangesWithAnyValue) {
    const CommandChain chain{{"a", "1"}, {"b", "2"}};
    EXPECT_EQ(format_command_chain(chain), "a=1\nb=2\n");
    EXPECT_EQ(command_chain_hash(chain), command_chain_hash(CommandChain{{"b", "2"}, {"a", "1"}}));
    EXPECT_EQ(command_chain_hash(chain).size(), 16u);
    EXPECT_NE(command_chain_hash(chain), command_chain_hash(CommandChain{{"a", "1"}, {"b", "3"}}));
}

TEST(CommandChainTest, DifferencesListEveryKeyThatDiffersOrIsMissing) {
    const CommandChain bundle{{"max_motor_rpm_per_s", "0"},
                              {"transmitter", "EspNowTransmitter"},
                              {"drive_mode", "WHEELS"}};
    const CommandChain profile{{"max_motor_rpm_per_s", "2000"},
                               {"transmitter", "EspNowTransmitter"}};
    const auto differences = command_chain_differences(bundle, profile);
    ASSERT_EQ(differences.size(), 2u);
    EXPECT_EQ(differences[0], "drive_mode: bundle WHEELS, profile (missing)");
    EXPECT_EQ(differences[1], "max_motor_rpm_per_s: bundle 0, profile 2000");
    EXPECT_TRUE(command_chain_differences(profile, profile).empty());
}

}  // namespace
}  // namespace auto_battlebot
