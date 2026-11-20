#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
source "$PROJECT_ROOT/install/fail_loudly.sh"

source "$PROJECT_ROOT/install/check_jetson_orin.sh"
source "$PROJECT_ROOT/install/install_packages.sh"
source "$PROJECT_ROOT/install/install_llvm_toolchain.sh"
source "$PROJECT_ROOT/install/install_opencv.sh"
source "$PROJECT_ROOT/install/install_pytorch_jetson.sh"
source "$PROJECT_ROOT/install/install_python_environment.sh"
source "$PROJECT_ROOT/install/install_foxglove_sdk.sh"
source "$PROJECT_ROOT/install/install_node.sh"
source "$PROJECT_ROOT/install/install_mcap_cli.sh"
source "$PROJECT_ROOT/install/install_taplo.sh"
source "$PROJECT_ROOT/install/install_clang_tidy_cache.sh"
source "$PROJECT_ROOT/install/install_platformio.sh"
source "$PROJECT_ROOT/install/install_zed_sdk.sh"
source "$PROJECT_ROOT/install/install_jetson_stability.sh"
source "$PROJECT_ROOT/install/install_jetson_clocks.sh"

check_jetson_orin
install_packages "$PROJECT_ROOT/install/jetson_packages.txt"
install_packages "$PROJECT_ROOT/install/base_packages.txt"
install_packages "$PROJECT_ROOT/install/jetson_r$(get_l4t_major)_packages.txt"
install_zed_sdk
install_llvm_toolchain
install_opencv --cuda --cuda-arch 8.7 --gstreamer --python-bindings \
    --python-version "$(get_jetson_python_version)"
install_python_environment -n
install_pytorch_jetson
install_foxglove_sdk
install_mcap_cli
install_taplo
install_clang_tidy_cache
install_platformio
install_node
install_cgroup_disable_memory
install_jetson_clocks
sudo systemctl set-default multi-user.target

if ! nvpmodel -q 2>/dev/null | grep -q "MAXN"; then
    echo "Switching to MAXN; nvpmodel asks whether to reboot."
    sudo nvpmodel -m 0
fi
