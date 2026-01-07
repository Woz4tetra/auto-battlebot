#!/bin/bash

# Install OpenCV on Jetson (ARM64) by building from source
install_opencv_jetson() {
    local version="${1:-4.10.0}"
    local build_folder="${2:-workspace}"
    local cuda_arch_bin="${3:-8.7}"
    local python_ver="${4:-3.10}"
    local install_prefix="/usr/local"

    set -e

    # Check if OpenCV is already installed via pkg-config
    if pkg-config --exists opencv4; then
        echo "OpenCV is already installed (pkg-config opencv4 found)."
        echo "To reinstall, purge existing OpenCV packages or remove from ${install_prefix}."
    else
        echo "OpenCV not found via pkg-config. Proceeding with source build..."
    fi

    # Prompt to remove default OpenCV
    while true; do
        echo "Do you want to remove the default OpenCV (yes/no)?"
        read rm_old

        if [ "$rm_old" = "yes" ]; then
            echo "** Remove other OpenCV first"
            sudo apt -y purge 'libopencv*' 'opencv*' || true
            break
        elif [ "$rm_old" = "no" ]; then
            break
        fi
    done

    echo "------------------------------------"
    echo "** Install requirements (1/4)"
    echo "------------------------------------"
    sudo apt-get update
    sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python${python_ver}-dev python3-numpy
    sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libv4l-dev v4l-utils qv4l2
    sudo apt-get install -y curl unzip

    echo "------------------------------------"
    echo "** Download OpenCV ${version} (2/4)"
    echo "------------------------------------"
    mkdir -p "${build_folder}"
    pushd "${build_folder}" >/dev/null
    curl -L "https://github.com/opencv/opencv/archive/${version}.zip" -o "opencv-${version}.zip"
    curl -L "https://github.com/opencv/opencv_contrib/archive/${version}.zip" -o "opencv_contrib-${version}.zip"
    unzip -q "opencv-${version}.zip"
    unzip -q "opencv_contrib-${version}.zip"
    rm -f "opencv-${version}.zip" "opencv_contrib-${version}.zip"
    cd "opencv-${version}" || exit 1

    echo "------------------------------------"
    echo "** Build OpenCV ${version} (3/4)"
    echo "------------------------------------"
    mkdir -p release
    cd release || exit 1
    cmake \
        -D WITH_CUDA=ON \
        -D WITH_CUDNN=ON \
        -D CUDA_ARCH_BIN="${cuda_arch_bin}" \
        -D CUDA_ARCH_PTX="" \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_EXTRA_MODULES_PATH="../../opencv_contrib-${version}/modules" \
        -D WITH_GSTREAMER=ON \
        -D WITH_LIBV4L=ON \
        -D BUILD_opencv_python3=ON \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX="${install_prefix}" \
        ..
    make -j"$(nproc)"

    echo "------------------------------------"
    echo "** Install OpenCV ${version} (4/4)"
    echo "------------------------------------"
    sudo make install

    # Update environment for libraries and Python bindings
    if ! grep -q '/usr/local/lib' ~/.bashrc; then
        echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
    fi
    local site_packages="${install_prefix}/lib/python${python_ver}/site-packages"
    if ! grep -q "${site_packages}" ~/.bashrc; then
        echo "export PYTHONPATH=${site_packages}:\$PYTHONPATH" >> ~/.bashrc
    fi
    source ~/.bashrc || true

    popd >/dev/null

    echo "** Install OpenCV ${version} successfully"
    echo "** Bye :)"
}

# Allow running directly: install with optional args
# Usage: install_opencv_jetson [version] [build_folder] [cuda_arch_bin] [python_version]
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_opencv_jetson "$@"
fi
