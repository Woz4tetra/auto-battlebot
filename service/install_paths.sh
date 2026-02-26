# System-wide install paths for auto_battlebot.
# Source this from run.sh and run_with_kiosk.sh. Override with AUTO_BATTLEBOT_PREFIX if needed.

AUTO_BATTLEBOT_PREFIX="${AUTO_BATTLEBOT_PREFIX:-/usr/local}"
AUTO_BATTLEBOT_EXE="$AUTO_BATTLEBOT_PREFIX/bin/auto_battlebot"
AUTO_BATTLEBOT_CONFIG="$AUTO_BATTLEBOT_PREFIX/share/auto_battlebot/config/main"
