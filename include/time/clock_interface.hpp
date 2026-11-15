#pragma once

namespace auto_battlebot {
class ClockInterface {
   public:
    virtual ~ClockInterface() = default;

    virtual double now() const = 0;

    virtual void set([[maybe_unused]] double seconds) {}
};

}  // namespace auto_battlebot
