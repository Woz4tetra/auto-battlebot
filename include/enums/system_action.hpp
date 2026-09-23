#pragma once

namespace auto_battlebot {
// Host actions the UI can ask for. Both stop the runner loop; main.cpp runs the host command once
// the loop and the UI are down.
enum class SystemAction { REBOOT_HOST, POWEROFF_HOST };
}  // namespace auto_battlebot
