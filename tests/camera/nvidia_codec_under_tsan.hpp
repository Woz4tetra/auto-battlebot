#pragma once

namespace auto_battlebot {

#ifdef AUTO_BATTLEBOT_THREAD_SANITIZED
inline constexpr bool kNvidiaCodecCrashesUnderThisBuild = true;
#else
inline constexpr bool kNvidiaCodecCrashesUnderThisBuild = false;
#endif

inline constexpr const char *kNvidiaCodecTsanSkipReason =
    "libnvcuvid threads crash ThreadSanitizer inside calloc";

}  // namespace auto_battlebot
