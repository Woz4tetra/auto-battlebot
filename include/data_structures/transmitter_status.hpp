#pragma once

namespace auto_battlebot {
struct TransmitterStatus {
    bool connected = false;
    bool receiving_channels = false;
    bool has_trainer_autonomy_switch = false;
    bool autonomy_switch_on = false;
};

}  // namespace auto_battlebot
