#include <gtest/gtest.h>
#include "transmitter/sim_transmitter.hpp"
#include "shared_memory/shared_memory_reader.hpp"
#include "shared_memory/simulation/simulation_velocity_command.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "../diagnostics_logger/test_diagnostics_logger.hpp"

namespace auto_battlebot
{
    class SimTransmitterTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Enable test mode to skip actual ROS publishing
            TestDiagnosticsLogger::enable_test_mode();

            config_.enable_double_buffering = false;
        }

        void TearDown() override
        {
            // Clean up shared memory
            shm_unlink("/auto_battlebot_command");
            TestDiagnosticsLogger::reset();
        }

        SimTransmitterConfiguration config_;
    };

    // Test basic initialization
    TEST_F(SimTransmitterTest, InitializeCreatesSharedMemory)
    {
        SimTransmitter transmitter(config_);

        EXPECT_TRUE(transmitter.initialize());
    }

    // Test send writes correct command structure
    TEST_F(SimTransmitterTest, SendWritesVelocityCommand)
    {
        SimTransmitter transmitter(config_);
        ASSERT_TRUE(transmitter.initialize());

        VelocityCommand cmd;
        cmd.linear_x = 1.5;
        cmd.linear_y = -0.5;
        cmd.angular_z = 2.0;

        transmitter.send(cmd);

        // Read back the command using SharedMemoryReader
        SharedMemoryReader reader("auto_battlebot_command", SimulationVelocityCommand::SIZE);
        ASSERT_TRUE(reader.open());

        const auto *written_cmd = reader.read_at<SimulationVelocityCommand>(0);
        EXPECT_EQ(written_cmd->command_id, 0);
        EXPECT_DOUBLE_EQ(written_cmd->linear_x, 1.5);
        EXPECT_DOUBLE_EQ(written_cmd->linear_y, -0.5);
        EXPECT_DOUBLE_EQ(written_cmd->angular_z, 2.0);
    }

    // Test command_id increments with each send
    TEST_F(SimTransmitterTest, SendIncrementsCommandId)
    {
        SimTransmitter transmitter(config_);
        ASSERT_TRUE(transmitter.initialize());

        VelocityCommand cmd{1.0, 0.0, 0.0};

        transmitter.send(cmd);
        transmitter.send(cmd);
        transmitter.send(cmd);

        // Without double buffering, all commands go to offset 0
        SharedMemoryReader reader("auto_battlebot_command", SimulationVelocityCommand::SIZE);
        ASSERT_TRUE(reader.open());

        const auto *written_cmd = reader.read_at<SimulationVelocityCommand>(0);
        // Last command should have command_id = 2 (0, 1, 2)
        EXPECT_EQ(written_cmd->command_id, 2);
    }

    // Test update returns empty feedback
    TEST_F(SimTransmitterTest, UpdateReturnsEmptyFeedback)
    {
        SimTransmitter transmitter(config_);
        ASSERT_TRUE(transmitter.initialize());

        CommandFeedback feedback = transmitter.update();

        EXPECT_TRUE(feedback.commands.empty());
    }

    // Test did_init_button_press returns true once then false
    TEST_F(SimTransmitterTest, DidInitButtonPressReturnsTrueOnce)
    {
        SimTransmitter transmitter(config_);
        ASSERT_TRUE(transmitter.initialize());

        // First call should return true
        EXPECT_TRUE(transmitter.did_init_button_press());

        // Subsequent calls should return false
        EXPECT_FALSE(transmitter.did_init_button_press());
        EXPECT_FALSE(transmitter.did_init_button_press());
        EXPECT_FALSE(transmitter.did_init_button_press());
    }

    // Test SimulationVelocityCommand struct size
    TEST_F(SimTransmitterTest, SimulationVelocityCommandSize)
    {
        // Verify the struct is packed correctly
        // command_id (8) + linear_x (8) + linear_y (8) + angular_z (8) = 32
        EXPECT_EQ(SimulationVelocityCommand::SIZE, 32);
        EXPECT_EQ(sizeof(SimulationVelocityCommand), 32);
    }

    // Test double buffering configuration
    class SimTransmitterDoubleBufferTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            TestDiagnosticsLogger::enable_test_mode();
            config_.enable_double_buffering = true;
        }

        void TearDown() override
        {
            shm_unlink("/auto_battlebot_command");
            TestDiagnosticsLogger::reset();
        }

        SimTransmitterConfiguration config_;
    };

    TEST_F(SimTransmitterDoubleBufferTest, DoubleBufferingAlternatesBuffers)
    {
        SimTransmitter transmitter(config_);
        ASSERT_TRUE(transmitter.initialize());

        // With double buffering enabled, buffer size is doubled
        SharedMemoryReader reader("auto_battlebot_command", SimulationVelocityCommand::SIZE * 2);
        ASSERT_TRUE(reader.open());

        // First command goes to buffer 0
        VelocityCommand cmd1{1.0, 0.0, 0.0};
        transmitter.send(cmd1);

        const auto *buf0_cmd = reader.read_at<SimulationVelocityCommand>(0);
        EXPECT_EQ(buf0_cmd->command_id, 0);
        EXPECT_DOUBLE_EQ(buf0_cmd->linear_x, 1.0);

        // Second command goes to buffer 1 (offset = buffer_size_)
        VelocityCommand cmd2{2.0, 0.0, 0.0};
        transmitter.send(cmd2);

        const auto *buf1_cmd = reader.read_at<SimulationVelocityCommand>(SimulationVelocityCommand::SIZE);
        EXPECT_EQ(buf1_cmd->command_id, 1);
        EXPECT_DOUBLE_EQ(buf1_cmd->linear_x, 2.0);

        // Third command goes back to buffer 0
        VelocityCommand cmd3{3.0, 0.0, 0.0};
        transmitter.send(cmd3);

        buf0_cmd = reader.read_at<SimulationVelocityCommand>(0);
        EXPECT_EQ(buf0_cmd->command_id, 2);
        EXPECT_DOUBLE_EQ(buf0_cmd->linear_x, 3.0);

        // Buffer 1 should still have command 1
        buf1_cmd = reader.read_at<SimulationVelocityCommand>(SimulationVelocityCommand::SIZE);
        EXPECT_EQ(buf1_cmd->command_id, 1);
    }

    // Test multiple velocity values
    TEST_F(SimTransmitterTest, SendVariousVelocities)
    {
        SimTransmitter transmitter(config_);
        ASSERT_TRUE(transmitter.initialize());

        SharedMemoryReader reader("auto_battlebot_command", SimulationVelocityCommand::SIZE);
        ASSERT_TRUE(reader.open());

        // Test zero velocities
        VelocityCommand zero_cmd{0.0, 0.0, 0.0};
        transmitter.send(zero_cmd);

        const auto *cmd = reader.read_at<SimulationVelocityCommand>(0);
        EXPECT_DOUBLE_EQ(cmd->linear_x, 0.0);
        EXPECT_DOUBLE_EQ(cmd->linear_y, 0.0);
        EXPECT_DOUBLE_EQ(cmd->angular_z, 0.0);

        // Test negative velocities
        VelocityCommand neg_cmd{-1.0, -2.0, -3.0};
        transmitter.send(neg_cmd);

        cmd = reader.read_at<SimulationVelocityCommand>(0);
        EXPECT_DOUBLE_EQ(cmd->linear_x, -1.0);
        EXPECT_DOUBLE_EQ(cmd->linear_y, -2.0);
        EXPECT_DOUBLE_EQ(cmd->angular_z, -3.0);

        // Test large velocities
        VelocityCommand large_cmd{100.0, 50.0, 10.0};
        transmitter.send(large_cmd);

        cmd = reader.read_at<SimulationVelocityCommand>(0);
        EXPECT_DOUBLE_EQ(cmd->linear_x, 100.0);
        EXPECT_DOUBLE_EQ(cmd->linear_y, 50.0);
        EXPECT_DOUBLE_EQ(cmd->angular_z, 10.0);
    }

    // Test that send can be called multiple times rapidly
    TEST_F(SimTransmitterTest, RapidSendCalls)
    {
        SimTransmitter transmitter(config_);
        ASSERT_TRUE(transmitter.initialize());

        // Send many commands rapidly
        for (int i = 0; i < 1000; i++)
        {
            VelocityCommand cmd{static_cast<double>(i), 0.0, 0.0};
            transmitter.send(cmd);
        }

        // Verify last command was written
        SharedMemoryReader reader("auto_battlebot_command", SimulationVelocityCommand::SIZE);
        ASSERT_TRUE(reader.open());

        const auto *cmd = reader.read_at<SimulationVelocityCommand>(0);
        EXPECT_EQ(cmd->command_id, 999);
        EXPECT_DOUBLE_EQ(cmd->linear_x, 999.0);
    }

} // namespace auto_battlebot
