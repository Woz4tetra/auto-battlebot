#!/bin/bash

ZED_SDK_VERSION="5.2.3"
ZED_SDK_L4T="36.4"
ZED_SDK_SHA256="d8fc4c74d4d33b764892517387f7b704fd43eed19d2d3e8006b8058fb6b8bbe1"

installed_zed_sdk_version() {
    local header="/usr/local/zed/include/sl/Camera.hpp"
    [ -f "$header" ] || return 0
    local major minor patch
    major=$(grep -oP '#define ZED_SDK_MAJOR_VERSION \K[0-9]+' "$header")
    minor=$(grep -oP '#define ZED_SDK_MINOR_VERSION \K[0-9]+' "$header")
    patch=$(grep -oP '#define ZED_SDK_PATCH_VERSION \K[0-9]+' "$header")
    echo "${major}.${minor}.${patch}"
}

jetson_l4t_major_minor() {
    local line
    line=$(head -n 1 /etc/nv_tegra_release 2>/dev/null || true)
    if [[ "$line" =~ R([0-9]+).*REVISION:\ ([0-9]+)\. ]]; then
        echo "${BASH_REMATCH[1]}.${BASH_REMATCH[2]}"
    fi
}

install_zed_sdk() {
    local installed
    installed=$(installed_zed_sdk_version)
    if [ "$installed" = "$ZED_SDK_VERSION" ]; then
        echo "ZED SDK ${ZED_SDK_VERSION} already installed; skipping."
        return 0
    fi

    local l4t
    l4t=$(jetson_l4t_major_minor)
    if [ "$l4t" != "$ZED_SDK_L4T" ]; then
        echo "Error: ZED SDK ${ZED_SDK_VERSION} is pinned to L4T ${ZED_SDK_L4T}; this box is L4T ${l4t:-unknown}."
        return 1
    fi

    local installer_name="ZED_SDK_Tegra_L4T${ZED_SDK_L4T}_v${ZED_SDK_VERSION}.zstd.run"
    local url="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/${ZED_SDK_VERSION%.*}/${installer_name}"
    local work_dir
    work_dir=$(mktemp -d)

    sudo apt-get install -y zstd
    echo "Downloading ${url}..."
    curl -fsSL "$url" -o "${work_dir}/${installer_name}"
    echo "${ZED_SDK_SHA256}  ${work_dir}/${installer_name}" | sha256sum -c -

    echo "Installing ZED SDK ${ZED_SDK_VERSION} (no tools, AI models, Python API, or GMSL drivers)..."
    (cd "$work_dir" && bash "./${installer_name}" -- silent skip_tools skip_od_module skip_python skip_drivers)
    rm -rf "$work_dir"

    installed=$(installed_zed_sdk_version)
    if [ "$installed" != "$ZED_SDK_VERSION" ]; then
        echo "Error: ZED SDK install finished but /usr/local/zed reports '${installed:-nothing}'."
        return 1
    fi
    echo "ZED SDK ${installed} installed to /usr/local/zed"
}
