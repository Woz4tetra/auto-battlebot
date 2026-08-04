# Toolchain image for x86_64 SVO playback.
#
# This image contains build tools and system libraries only. It never copies src/,
# include/, config/, or data/. scripts/docker/run_playback.sh bind-mounts the repo at
# its host path and builds into build-docker/, so editing code costs zero image
# rebuilds and the FetchContent tree in build-docker/_deps survives image changes.
#
# Layers are ordered most-stable-first. Rebuild triggers are limited to
# install/base_packages.txt, install/ubuntu_24_packages.txt, .llvm-version, and the
# version pins below.
#
# The base matches the reference desktop exactly: ZED SDK 5.1, CUDA 13.0,
# Ubuntu 24.04. Using the Stereolabs image avoids running the interactive ZED .run
# installer during the build.
FROM stereolabs/zed:5.1-devel-cuda13.0-ubuntu24.04

# TensorRT is pinned on purpose. The engines in data/models (*_x86_64_sm89.engine) are
# serialized by a specific TensorRT version, and src/tensorrt_inference/trt_engine.cpp
# only calls deserializeCudaEngine, never rebuilds from ONNX. A different TensorRT here
# fails at engine load with no useful message. If you bump this, rebuild the engines to
# match with scripts/sync_models.py.
ARG TENSORRT_VERSION=10.14.1.48-1+cuda13.0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Layer 1: bootstrap -------------------------------------------------------------
# The base image ships sudo, cmake, wget, gcc, and make but not git or
# apt-add-repository. git is required: CMakeLists.txt pulls eight dependencies
# (miniroscpp, tomlplusplus, CLI11, magic_enum, lvgl, mcap, spdlog, googletest) with
# FetchContent over git, so the build fails at configure without it.
# apt-add-repository comes from software-properties-common and is used by
# install/install_packages.sh. This layer never changes.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    ca-certificates \
    gnupg \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# --- Layer 2: TensorRT (pinned) -----------------------------------------------------
# Installs the pinned TensorRT, then holds it so a later apt install cannot drag in a
# newer one. install/install_tensorrt_runtime_ubuntu.sh is not reused here because it
# installs libnvinfer-dev unpinned, which defeats the point.
#
# Do not add cuda-keyring here. The base image already registers the NVIDIA CUDA repo
# using a legacy /etc/apt/trusted.gpg key, and installing the keyring package adds a
# second entry for the same source with a Signed-By value. apt then refuses to read any
# source list: "Conflicting values set for option Signed-By".
# Every package is pinned, not just libnvinfer-dev. Pinning the dev package alone fails:
# it depends on libnvinfer10 (= same version), but apt independently resolves that to
# the newest in the repo and reports "held broken packages".
#
# libnvinfer-plugin10 is the runtime plugin library. cmake/FindTensorRT.cmake links only
# nvinfer, but an engine referencing standard TensorRT plugin layers needs this present
# to deserialize. The reference desktop has it, so the container matches.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    "libnvinfer-headers-dev=${TENSORRT_VERSION}" \
    "libnvinfer10=${TENSORRT_VERSION}" \
    "libnvinfer-dev=${TENSORRT_VERSION}" \
    "libnvinfer-plugin10=${TENSORRT_VERSION}" \
    && apt-mark hold \
    libnvinfer-headers-dev \
    libnvinfer10 \
    libnvinfer-dev \
    libnvinfer-plugin10 \
    && rm -rf /var/lib/apt/lists/*

# --- Layer 3: apt packages ----------------------------------------------------------
# Reuses the same helper and package lists as scripts/install_ubuntu_24.sh so the
# container and bare-metal installs cannot drift. install_packages skips anything
# dpkg -s already finds, so the TensorRT pin from layer 2 survives even though
# ubuntu_24_packages.txt lists libnvinfer-dev unpinned.
COPY install/install_packages.sh /install/
COPY install/base_packages.txt install/ubuntu_24_packages.txt /install/
RUN source /install/install_packages.sh \
    && install_packages /install/base_packages.txt \
    && install_packages /install/ubuntu_24_packages.txt \
    && rm -rf /var/lib/apt/lists/*

# --- Layer 4: LLVM toolchain --------------------------------------------------------
# Needed for scripts/lint inside the container. install_llvm_toolchain.sh resolves its
# project root as dirname(dirname($BASH_SOURCE)), so a script at /install/ reads
# /.llvm-version. Keep both paths in sync if you move the script.
#
# apt-get update first. install_llvm_toolchain.sh decides whether to add apt.llvm.org by
# checking `apt-cache show clang-tidy-$version`. The previous layer ends with
# rm -rf /var/lib/apt/lists/*, so without a refresh that lookup finds no index, the
# script concludes clang-18 is missing from the distro repos, and it adds apt.llvm.org
# even though Ubuntu 24.04 ships clang-18 itself.
COPY .llvm-version /
COPY install/install_llvm_toolchain.sh /install/
RUN apt-get update \
    && source /install/install_llvm_toolchain.sh \
    && install_llvm_toolchain \
    && rm -rf /var/lib/apt/lists/*

# --- Layer 5: writable paths for an arbitrary UID -----------------------------------
# run_playback.sh passes --user "$(id -u):$(id -g)" so build artifacts land in the bind
# mount owned by the invoking user rather than root. That UID is not known at build
# time, so anything it must write has to be world-writable.
#
# /usr/local/zed/resources ships as mode 770 root:zed. ZED_NEURAL_LIGHT
# (include/rgbd_camera/config.hpp) GPU-optimizes neural_depth_light_5.2.model into this
# directory on first run, which fails as read-only for a non-root user. Docker seeds a
# fresh named volume from the image directory including its mode, so setting the mode
# here is what makes the persisted volume writable.
# The whole ZED tree ships as root:zed mode 770, so a non-root user cannot even traverse
# /usr/local/zed. That breaks find_package(ZED 5 REQUIRED) at cmake configure, and
# breaks linking and loading libsl_zed.so. a+rX grants read everywhere and traverse on
# directories only. Relying on the zed group instead would couple the container to a gid
# that differs between the image (1001) and this host (1002). The tree is 111 MB, so the
# copy-up this causes is under 1% of the image.
#
# settings/ holds the per-serial camera calibration (SNxxxxxxxx.conf). The SDK
# downloads it on first use, including for SVO playback, and fails the camera open with
# CALIBRATION FILE NOT AVAILABLE if it cannot write here.
RUN mkdir -p /home/dev \
    && chmod 1777 /home/dev \
    && chmod -R a+rX /usr/local/zed \
    && chmod 1777 /usr/local/zed/resources /usr/local/zed/settings

# Deliberately absent, and why:
#   install_docker_ubuntu      - no docker-in-docker; ros-connector runs on the host
#   install_ros_connector      - host-side container, reachable over --network host
#   install_python_environment - 13 GB, training only, not used by playback
#   install_platformio         - firmware toolchain
#   build_cpp_project          - runs at container start against the bind mount

# The base image bundles TensorRT 10.13.2 inside the CUDA tree
# (/usr/local/cuda-13.0/targets/x86_64-linux/lib/libnvinfer.so.10.13.2) and its
# LD_LIBRARY_PATH puts /usr/local/cuda/lib64 ahead of /usr/lib/x86_64-linux-gnu. Without
# the prepend below, the pinned 10.14.1.48 installed above is used to compile and link,
# but the bundled 10.13.2 is what actually loads at runtime, and engines fail with
# "Error Code 6: API Usage Error ... engine plan file is not compatible".
#
# The reference desktop has no TensorRT in its CUDA tree, so it resolves libnvinfer from
# /usr/lib/x86_64-linux-gnu. This makes the container resolve it the same way. Keep the
# rest of the path intact: the bundled copy is left in place because the ZED SDK links
# against the same soname, and 10.14.1.48 satisfies it.
#
# Placed at the end of the file on purpose. An ENV here invalidates only the trailing
# layers; putting it next to the TensorRT install would force the 60-minute apt layer to
# rebuild on any change.
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}

WORKDIR /workspace
CMD ["/bin/bash"]
