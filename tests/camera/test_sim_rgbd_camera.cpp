#include <gtest/gtest.h>
#include "rgbd_camera/sim_rgbd_camera.hpp"
#include "data_structures.hpp"
#include "shared_memory/shared_memory_writer.hpp"

namespace auto_battlebot
{
    TEST(SimRgbdCameraTest, NoMemoryLeaks)
    {
        // Create and destroy multiple times to catch leaks
        for (int i = 0; i < 10; i++)
        {
            SimRgbdCameraConfiguration config;
            SimRgbdCamera camera(config);
            camera.initialize();
            // camera goes out of scope - destructor should clean up
        }
    }
} // namespace auto_battlebot
