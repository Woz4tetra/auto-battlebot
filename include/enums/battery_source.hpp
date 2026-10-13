#pragma once

namespace auto_battlebot {
// What [battery] source reads. NONE: no monitor runs and /status/battery is never published.
enum class BatterySource { NONE, WAVESHARE_UPS };
}  // namespace auto_battlebot
