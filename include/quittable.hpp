#pragma once

namespace auto_battlebot {

class Quittable {
   public:
    virtual ~Quittable() = default;
    virtual void request_quit() = 0;
};

}  // namespace auto_battlebot
