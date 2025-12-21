#include "transmitter/playback_transmitter.hpp"

namespace auto_battlebot
{
    PlaybackTransmitter::PlaybackTransmitter(double init_delay_seconds)
        : init_delay_seconds_(init_delay_seconds),
          initialized_(false),
          start_time_(),
          init_button_pressed_(false),
          init_button_done_pressing_(false)
    {
    }

    bool PlaybackTransmitter::initialize()
    {
        start_time_ = std::chrono::steady_clock::now();
        initialized_ = true;
        return true;
    }

    void PlaybackTransmitter::update()
    {
        if (!initialized_ || init_button_pressed_)
        {
            return;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_);

        if (elapsed.count() >= init_delay_seconds_)
        {
            init_button_pressed_ = true;
        }
    }

    void PlaybackTransmitter::send([[maybe_unused]] VelocityCommand command)
    {
        // No-op for playback
    }

    bool PlaybackTransmitter::did_init_button_press()
    {
        if (!init_button_done_pressing_ && init_button_pressed_)
        {
            init_button_done_pressing_ = true;
            return true;
        }
        else
        {
            return false;
        }
    }

} // namespace auto_battlebot
