#include "rgbd_camera/sim_rgbd_camera.hpp"

namespace auto_battlebot
{
    SimRgbdCamera::SimRgbdCamera(SimRgbdCameraConfiguration &config)
        : expected_width_(config.width),
          expected_height_(config.height),
          frames_received_(0),
          last_log_time_(std::chrono::steady_clock::now())
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_rgbd_camera");
    }

    SimRgbdCamera::~SimRgbdCamera()
    {
    }

    bool SimRgbdCamera::initialize()
    {
        std::cout << "SimRgbdCamera initialized successfully" << std::endl;

        // Reset stats on re-initialization
        frames_received_ = 0;
        last_log_time_ = std::chrono::steady_clock::now();

        return true;
    }

    bool SimRgbdCamera::get([[maybe_unused]] CameraData &data, [[maybe_unused]] bool get_depth)
    {
        return true;
    }

    void SimRgbdCamera::log_stats()
    {
        // Track frame statistics
        frames_received_++;

        // Periodic stats logging (every 1 second)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();
        if (elapsed >= 1)
        {
            double fps = static_cast<double>(frames_received_) / elapsed;
            diagnostics_logger_->info("stats",
                                      {{"frames_received", static_cast<int>(frames_received_)},
                                       {"fps", fps}});
            frames_received_ = 0;
            last_log_time_ = now;
        }
    }

    bool SimRgbdCamera::should_close()
    {
        return false;
    }
} // namespace auto_battlebot
