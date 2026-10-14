#pragma once

namespace auto_battlebot {
// Auto-exposure flicker rejection. Order matches the ZED SDK's AE_ANTIBANDING values 0 to 3.
enum class AntiBanding { OFF, AUTO, HZ_50, HZ_60 };
}  // namespace auto_battlebot
