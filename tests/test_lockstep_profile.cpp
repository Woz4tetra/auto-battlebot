#include <gtest/gtest.h>

#include "config/config.hpp"
#include "directories.hpp"
#include "rgbd_camera/config.hpp"

namespace auto_battlebot {

TEST(LockstepProfileTest, ResolvesToDeterministicPlayback) {
    const ClassConfiguration config =
        load_classes_from_config(get_config_dir() / "playback" / "mrs_buff_mk3_lockstep.toml");

    const auto *camera = dynamic_cast<ZedSvoPlaybackCameraConfiguration *>(config.camera.get());
    ASSERT_NE(camera, nullptr);
    EXPECT_FALSE(camera->svo_real_time_mode);
    EXPECT_FALSE(camera->rebase_stamps);
    EXPECT_FALSE(camera->position_tracking);
    EXPECT_TRUE(camera->svo_decode_to_start);

    EXPECT_EQ(config.clock->type, "SteppedClock");
    EXPECT_EQ(config.control_loop->type, "SteppedControlLoop");
    EXPECT_EQ(config.control_loop->rate_hz, 250.0);
    EXPECT_EQ(config.runner.max_loop_rate_hz, 0.0);
    EXPECT_EQ(config.transmitter->type, "PlaybackTransmitter");
}

}  // namespace auto_battlebot
