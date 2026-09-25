#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
source "$PROJECT_ROOT/install/fail_loudly.sh"

# Source helper functions
source "$PROJECT_ROOT/install/check_jetson_orin.sh"
source "$PROJECT_ROOT/install/install_packages.sh"
source "$PROJECT_ROOT/install/install_llvm_toolchain.sh"
source "$PROJECT_ROOT/install/install_opencv.sh"
source "$PROJECT_ROOT/install/install_pytorch_jetson.sh"
source "$PROJECT_ROOT/install/install_python_environment.sh"
source "$PROJECT_ROOT/install/install_foxglove_sdk.sh"
source "$PROJECT_ROOT/install/build_cpp_project.sh"
source "$PROJECT_ROOT/install/install_node.sh"
source "$PROJECT_ROOT/install/install_mcap_cli.sh"
source "$PROJECT_ROOT/install/install_taplo.sh"
source "$PROJECT_ROOT/install/install_clang_tidy_cache.sh"
source "$PROJECT_ROOT/install/install_udev_rules.sh"
source "$PROJECT_ROOT/install/install_pstore_ramoops.sh"
source "$PROJECT_ROOT/install/install_jetson_stability.sh"
source "$PROJECT_ROOT/install/install_jetson_clocks.sh"
source "$PROJECT_ROOT/install/install_dashboard_network.sh"

# Run checks and installation
check_jetson_orin
# Step 0: Install TensorRT runtime (before CMake)
install_packages "$PROJECT_ROOT/install/jetson_packages.txt"
install_packages "$PROJECT_ROOT/install/base_packages.txt"
# jetson_r36_packages.txt (JetPack 6, jammy) or jetson_r39_packages.txt (JetPack 7, noble)
install_packages "$PROJECT_ROOT/install/jetson_r$(get_l4t_major)_packages.txt"
install_llvm_toolchain
install_opencv --cuda --cuda-arch 8.7 --gstreamer --python-bindings \
    --python-version "$(get_jetson_python_version)"
# Python venv (creates venv/, installs deps; also pulls Jetson PyTorch wheel)
install_python_environment -n
# PyTorch for Jetson (NVIDIA wheel into project venv)
install_pytorch_jetson
install_foxglove_sdk
install_mcap_cli
install_taplo
install_clang_tidy_cache
install_node
install_udev_rules
install_pstore_ramoops
install_jetson_stability
install_jetson_clocks
# Refuses (and says why) when the only Ethernet port is the uplink; the rest of the install
# still stands, so it is not fatal here.
install_dashboard_network || echo "Dashboard network not set up; see the message above."
# The ZED Box has no display. Booting to graphical.target makes service/with_display.sh wait
# 60s for an X session on every service start, and GDM + Xorg hold memory for nothing.
# Takes effect on the next boot.
sudo systemctl set-default multi-user.target

build_cpp_project "$@"
"$PROJECT_ROOT/scripts/build_web.sh"
