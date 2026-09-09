#!/bin/bash
# Build and install OpenCV from source into /usr/local, with resume support.
#
# Every platform gets the same version, and that version is what JetPack's Jetson build already
# produced: 4.10.0. cv::aruco moved from the contrib module opencv_aruco into core
# opencv_objdetect at 4.7 and the API changed with it, so FiducialFieldFilter needs 4.7 or newer
# everywhere or it needs version guards it would never shed. apt ships 4.5.4 on jammy and 4.6.0
# on noble, both on the wrong side of that break.
#
# The apt runtime libraries stay installed: other packages link them and removing libopencv-dev
# cascades. CMake searches /usr/local first, and CMakeLists.txt asserts the version, so the
# source build wins without uninstalling anything.
#
# based on https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.10.0_Jetpack6.1.sh

# Version CMake would resolve at ${prefix}, or the pkg-config version, or empty.
opencv_installed_version() {
    local prefix="${1:-/usr/local}"
    local version_config="${prefix}/lib/cmake/opencv4/OpenCVConfig-version.cmake"
    if [[ -f "${version_config}" ]]; then
        sed -n 's/.*set(OpenCV_VERSION[[:space:]]\+\([0-9.]\+\).*/\1/p' "${version_config}" | head -1
        return 0
    fi
    pkg-config --modversion opencv4 2>/dev/null || true
}

# True when $1 is at least $2.
opencv_version_at_least() {
    local have="$1" want="$2"
    [[ -n "${have}" ]] || return 1
    [[ "$(printf '%s\n%s\n' "${want}" "${have}" | sort -V | head -1)" == "${want}" ]]
}

install_opencv() {
    local version="4.10.0"
    local build_folder="${HOME}/opencv_build"
    local with_cuda="OFF"
    local cuda_arch_bin="8.7"
    local with_gstreamer="OFF"
    local build_python="OFF"
    local python_ver="3.10"
    local install_prefix="/usr/local"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --version) version="$2"; shift 2 ;;
            --build-folder) build_folder="$2"; shift 2 ;;
            --cuda) with_cuda="ON"; shift ;;
            --cuda-arch) cuda_arch_bin="$2"; shift 2 ;;
            --gstreamer) with_gstreamer="ON"; shift ;;
            --python-bindings) build_python="ON"; shift ;;
            --python-version) python_ver="$2"; shift 2 ;;
            *) echo "install_opencv: unknown argument $1" >&2; return 1 ;;
        esac
    done

    set -euo pipefail

    echo "OpenCV setup: version=${version}, build_folder=${build_folder}, CUDA=${with_cuda}, gstreamer=${with_gstreamer}, python=${build_python}"

    trap 'echo "\nInstallation interrupted. You can re-run this script; it will resume from the last successful step."' ERR

    # Step 0: skip only when the installed version is new enough. Testing `pkg-config --exists
    # opencv4` alone is true on any machine that ever had apt OpenCV, which is every desktop, so
    # the upgrade would silently do nothing. It also let JetPack's 4.8.0 pre-empt the Jetson build.
    local existing_ver
    existing_ver="$(opencv_installed_version "${install_prefix}")"
    if opencv_version_at_least "${existing_ver}" "${version}"; then
        echo "OpenCV ${existing_ver} already installed at ${install_prefix} (>= ${version}). Skipping."
        return 0
    fi
    if [[ -n "${existing_ver}" ]]; then
        echo "Found OpenCV ${existing_ver}, want ${version}. Building from source..."
    else
        echo "OpenCV not found via pkg-config/CMake. Proceeding with source build..."
    fi

    echo "------------------------------------"
    echo "** Install requirements (1/4)"
    echo "------------------------------------"
    sudo apt-get update -y
    sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    sudo apt-get install -y libtbb-dev libjpeg-dev libpng-dev libtiff-dev libv4l-dev v4l-utils
    sudo apt-get install -y curl unzip
    if [[ "${with_gstreamer}" == "ON" ]]; then
        sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
    fi
    if [[ "${build_python}" == "ON" ]]; then
        sudo apt-get install -y "python${python_ver}-dev" python3-numpy
    fi

    echo "------------------------------------"
    echo "** Download OpenCV ${version} (2/4)"
    echo "------------------------------------"
    mkdir -p "${build_folder}"
    pushd "${build_folder}" >/dev/null

    local opencv_src_dir="opencv-${version}"
    local build_dir="${opencv_src_dir}/release"

    # No opencv_contrib. It existed here for aruco; at 4.7+ aruco is core, and CMakeLists.txt
    # asks only for core modules.
    if [[ ! -d "${opencv_src_dir}" ]]; then
        echo "Fetching opencv-${version} sources..."
        curl -L -C - "https://github.com/opencv/opencv/archive/${version}.zip" -o "opencv-${version}.zip"
        unzip -q -n "opencv-${version}.zip"
        rm -f "opencv-${version}.zip"
    else
        echo "opencv-${version} already present; skipping download."
    fi

    echo "------------------------------------"
    echo "** Configure build (3/4)"
    echo "------------------------------------"
    cd "${opencv_src_dir}" || { echo "Missing source dir: ${build_folder}/${opencv_src_dir}"; exit 1; }
    mkdir -p "${build_dir}"

    cmake \
        -S . \
        -B "${build_dir}" \
        -D WITH_CUDA="${with_cuda}" \
        -D WITH_CUDNN="${with_cuda}" \
        -D CUDA_ARCH_BIN="${cuda_arch_bin}" \
        -D CUDA_ARCH_PTX="" \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D WITH_GSTREAMER="${with_gstreamer}" \
        -D WITH_LIBV4L=ON \
        -D BUILD_opencv_python3="${build_python}" \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_DOCS=OFF \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX="${install_prefix}"

    echo "------------------------------------"
    echo "** Build OpenCV ${version} (4/4)"
    echo "------------------------------------"
    cmake --build "${build_dir}" -j"$(nproc)"

    echo "------------------------------------"
    echo "** Install OpenCV ${version}"
    echo "------------------------------------"
    sudo cmake --install "${build_dir}"
    sudo ldconfig

    if ! grep -qE '^export LD_LIBRARY_PATH=.*/usr/local/lib' ~/.bashrc; then
        echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
    fi
    if [[ "${build_python}" == "ON" ]]; then
        local site_packages="${install_prefix}/lib/python${python_ver}/site-packages"
        if ! grep -q "${site_packages}" ~/.bashrc; then
            echo "export PYTHONPATH=${site_packages}:\$PYTHONPATH" >> ~/.bashrc
        fi
    fi

    popd >/dev/null

    echo "** Install OpenCV ${version} completed"
    echo "Run ./scripts/clean_build.sh: build/CMakeCache.txt pins the old OpenCV_DIR and keeps"
    echo "resolving the previous version until the cache is dropped."
}
