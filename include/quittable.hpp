#pragma once

namespace auto_battlebot {

class Quittable {
   public:
    virtual ~Quittable() = default;
    virtual void quit_from_signal_handler() = 0;
};

}  // namespace auto_battlebot
