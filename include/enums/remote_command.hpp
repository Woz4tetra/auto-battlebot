#pragma once

namespace auto_battlebot {
// Commands a Foxglove client can send through viz_relay. Adding one means handling it in
// Runner::dispatch_remote_command, which -Wswitch enforces.
enum class RemoteCommand { REINIT_FIELD };
}  // namespace auto_battlebot
