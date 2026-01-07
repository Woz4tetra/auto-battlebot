#!/bin/bash

# Install OpenCV on Jetson (ARM64) by building from source, with resume support
install_opencv_jetson() {
    local version="${1:-4.10.0}"
    # Use a build folder in the user's home directory by default
    local build_folder="${2:-${HOME}/opencv_build}"
    local cuda_arch_bin="${3:-8.7}"
    local python_ver="${4:-3.10}"
    local install_prefix="/usr/local"

    set -euo pipefail

    # Minimal progress hints
    echo "OpenCV setup: version=${version}, build_folder=${build_folder}, CUDA_ARCH_BIN=${cuda_arch_bin}, python=${python_ver}"

    # Trap errors to provide resume guidance
    trap 'echo "\nInstallation interrupted. You can re-run this script; it will resume from the last successful step."' ERR

    # Helper paths (global install paths)
    local opencv_pkgconfig="${install_prefix}/lib/pkgconfig/opencv4.pc"
    local opencv_cmake_config="${install_prefix}/lib/cmake/opencv4/OpenCVConfig.cmake"

    # Step 0: Early exit if OpenCV is already installed
    if pkg-config --exists opencv4 || [[ -f "${opencv_pkgconfig}" ]] || [[ -f "${opencv_cmake_config}" ]]; then
        local existing_ver
        existing_ver=$(pkg-config --modversion opencv4 2>/dev/null || echo "unknown")
        echo "OpenCV is already installed (version: ${existing_ver})."
        # Optional: check Python bindings for the requested Python version
        if python${python_ver} -c "import cv2; print(cv2.__version__)" >/dev/null 2>&1; then
            echo "Python bindings detected for Python ${python_ver}. Exiting early."
        else
            echo "Note: Python bindings for Python ${python_ver} not detected; C++ install present. Exiting early."
        fi
        return 0
    fi
    echo "OpenCV not found via pkg-config/CMake. Proceeding with source build..."

    # Prompt to remove default OpenCV (idempotent)
    while true; do
        echo "Do you want to remove the default OpenCV (yes/no)?"
        read rm_old
        if [[ "${rm_old}" == "yes" ]]; then
            echo "** Removing other OpenCV packages (if any)"
            sudo apt -y purge 'libopencv*' 'opencv*' || true
            break
        elif [[ "${rm_old}" == "no" ]]; then
            break
        fi
    done

    echo "------------------------------------"
    echo "** Install requirements (1/4)"
    echo "------------------------------------"
    sudo apt-get update -y
    sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python${python_ver}-dev python3-numpy
    sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libv4l-dev v4l-utils qv4l2
    sudo apt-get install -y curl unzip

    echo "------------------------------------"
    echo "** Download OpenCV ${version} (2/4)"
    echo "------------------------------------"
    mkdir -p "${build_folder}"
    pushd "${build_folder}" >/dev/null

    # Helper paths (within build folder)
    local opencv_src_dir="opencv-${version}"
    local opencv_contrib_dir="opencv_contrib-${version}"
    local build_dir="${opencv_src_dir}/release"

    # Download sources only if directories don't exist
    if [[ ! -d "${opencv_src_dir}" ]]; then
        echo "Fetching opencv-${version} sources..."
        # Use curl -C - to resume partial downloads
        curl -L -C - "https://github.com/opencv/opencv/archive/${version}.zip" -o "opencv-${version}.zip"
        unzip -q -n "opencv-${version}.zip"
        rm -f "opencv-${version}.zip"
    else
        echo "opencv-${version} already present; skipping download."
    fi

    if [[ ! -d "${opencv_contrib_dir}" ]]; then
        echo "Fetching opencv_contrib-${version} sources..."
        curl -L -C - "https://github.com/opencv/opencv_contrib/archive/${version}.zip" -o "opencv_contrib-${version}.zip"
        unzip -q -n "opencv_contrib-${version}.zip"
        rm -f "opencv_contrib-${version}.zip"
    else
        echo "opencv_contrib-${version} already present; skipping download."
    fi

    echo "------------------------------------"
    echo "** Configure build (3/4)"
    echo "------------------------------------"
    cd "${opencv_src_dir}" || { echo "Missing source dir: ${build_folder}/${opencv_src_dir}"; exit 1; }
    mkdir -p "${build_dir}"

    # Configure only if cache missing; otherwise reconfigure to ensure flags
    if [[ ! -f "${build_dir}/CMakeCache.txt" ]]; then
        echo "Running CMake configuration..."
        cmake \
            -S . \
            -B "${build_dir}" \
            -D WITH_CUDA=ON \
            -D WITH_CUDNN=ON \
            -D CUDA_ARCH_BIN="${cuda_arch_bin}" \
            -D CUDA_ARCH_PTX="" \
            -D OPENCV_GENERATE_PKGCONFIG=ON \
            -D OPENCV_EXTRA_MODULES_PATH="${opencv_contrib_dir}/modules" \
            -D WITH_GSTREAMER=ON \
            -D WITH_LIBV4L=ON \
            -D BUILD_opencv_python3=ON \
            -D BUILD_TESTS=OFF \
            -D BUILD_PERF_TESTS=OFF \
            -D BUILD_EXAMPLES=OFF \
            -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX="${install_prefix}"
    else
        echo "CMake cache found; ensuring configuration is up-to-date..."
        cmake -S . -B "${build_dir}" \
            -D WITH_CUDA=ON -D WITH_CUDNN=ON -D CUDA_ARCH_BIN="${cuda_arch_bin}" -D CUDA_ARCH_PTX="" \
            -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_EXTRA_MODULES_PATH="${opencv_contrib_dir}/modules" \
            -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python3=ON \
            -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF \
            -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX="${install_prefix}"
    fi

    echo "------------------------------------"
    echo "** Build OpenCV ${version} (4/4)"
    echo "------------------------------------"
    cmake --build "${build_dir}" -j"$(nproc)"

    echo "------------------------------------"
    echo "** Install OpenCV ${version}"
    echo "------------------------------------"
    # Only install if not already installed
    if pkg-config --exists opencv4 || [[ -f "${opencv_pkgconfig}" ]] || [[ -f "${opencv_cmake_config}" ]]; then
        echo "OpenCV already installed; skipping make install."
    else
        sudo cmake --install "${build_dir}"
    fi

    # Update environment for libraries and Python bindings (idempotent)
    if ! grep -qE '^export LD_LIBRARY_PATH=.*/usr/local/lib' ~/.bashrc; then
        echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
    fi
    local site_packages="${install_prefix}/lib/python${python_ver}/site-packages"
    if ! grep -q "${site_packages}" ~/.bashrc; then
        echo "export PYTHONPATH=${site_packages}:\$PYTHONPATH" >> ~/.bashrc
    fi
    # Avoid failing on non-interactive shells
    if [[ -f ~/.bashrc ]]; then
        source ~/.bashrc || true
    fi

    popd >/dev/null

    echo "** Install OpenCV ${version} completed"
    echo "You can safely re-run this script; it will pick up where it left off."
}
