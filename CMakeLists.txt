cmake_minimum_required(VERSION 3.14)
project(auto_battlebot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTING "Build tests" OFF)

option(BUILD_DEBUG "Build with debug flags" OFF)

if(BUILD_TESTING)
    message("Building tests")
endif()

if(BUILD_DEBUG)
    message("Building with debug flags")
endif()

include(FetchContent)

FetchContent_Declare(
    tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG v3.4.0
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
)

FetchContent_Declare(
    cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG v2.4.1
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
)

FetchContent_Declare(
    magic_enum
    GIT_REPOSITORY https://github.com/Neargye/magic_enum.git
    GIT_TAG v0.9.6
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
)

# HTTP server for the web dashboard, linked to viz_relay only. Both are header-only; populating
# them without their own CMake (SOURCE_SUBDIR has no CMakeLists.txt) avoids Crow's find_package
# for Asio, and the viz_relay target below wires the include paths itself.
FetchContent_Declare(
    crow
    GIT_REPOSITORY https://github.com/CrowCpp/Crow.git
    GIT_TAG v1.2.1.2
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
    SOURCE_SUBDIR include
)
FetchContent_Declare(
    asio
    GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
    GIT_TAG asio-1-30-2
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
    SOURCE_SUBDIR asio/include
)

FetchContent_Declare(
    mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG releases/cpp/v1.4.0
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
)

# JSON for the remote protocol (include/remote/). The URL tarball is the release's single-header
# CMake package, which is far smaller than the repository.
# DOWNLOAD_EXTRACT_TIMESTAMP is CMake 3.24+. JetPack 6 ships 3.22, which reads the unknown keyword
# as more URL entries and fails with "At least one entry of URL is a path".
set(NLOHMANN_JSON_URL_OPTS)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(NLOHMANN_JSON_URL_OPTS DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
endif()
FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    ${NLOHMANN_JSON_URL_OPTS}
)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.14.1
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
)
set(SPDLOG_BUILD_PIC ON CACHE BOOL "" FORCE)

if(BUILD_TESTING)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
        UPDATE_DISCONNECTED TRUE
    )

    FetchContent_MakeAvailable(tomlplusplus cli11 magic_enum spdlog nlohmann_json googletest)
else()
    FetchContent_MakeAvailable(tomlplusplus cli11 magic_enum spdlog nlohmann_json)
endif()

# mcap ships no CMake target, just headers plus a single-TU implementation define.
FetchContent_MakeAvailable(mcap)
FetchContent_MakeAvailable(crow asio)
add_library(mcap_reader STATIC "${CMAKE_CURRENT_SOURCE_DIR}/src/rgbd_camera/mcap_impl.cpp")
set_target_properties(mcap_reader PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mcap_reader SYSTEM PUBLIC "${mcap_SOURCE_DIR}/cpp/mcap/include")
target_compile_options(mcap_reader PRIVATE -w)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED IMPORTED_TARGET liblz4)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)
target_link_libraries(mcap_reader PUBLIC PkgConfig::LZ4 PkgConfig::ZSTD)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Threads REQUIRED)
find_package(OpenCV 4.10 REQUIRED COMPONENTS core imgproc highgui dnn objdetect calib3d)
find_package(Eigen3 REQUIRED)

# RTNeural runs RlPolicyNavigation's network. Only src/rl_policy/runtimes/rtneural_runtime.cpp
# includes it. Used header-only with the Eigen backend against the system Eigen 3, so its own
# CMakeLists is never added: that one puts its bundled Eigen copy on the include path.
# SOURCE_SUBDIR names a directory with no CMakeLists.txt, which makes MakeAvailable populate
# without add_subdirectory. No submodules: xsimd is for the backend this build does not use.
# SYSTEM keeps its headers out of -Werror.
FetchContent_Declare(
    rtneural
    GIT_REPOSITORY https://github.com/jatinchowdhury18/RTNeural.git
    GIT_TAG 95c3c0f987a6fe903e7eec71e797405dbed7caf7
    GIT_SUBMODULES ""
    SOURCE_SUBDIR populate-only
    UPDATE_DISCONNECTED TRUE
)
FetchContent_MakeAvailable(rtneural)
add_library(rtneural INTERFACE)
target_include_directories(rtneural SYSTEM INTERFACE "${rtneural_SOURCE_DIR}")
target_compile_definitions(rtneural INTERFACE
    RTNEURAL_USE_EIGEN=1
    RTNEURAL_DEFAULT_ALIGNMENT=16
    RTNEURAL_NAMESPACE=RTNeural
)
target_link_libraries(rtneural INTERFACE Eigen3::Eigen)
find_package(PCL REQUIRED COMPONENTS common sample_consensus)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)
pkg_check_modules(GST IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)

find_package(ZED 5 QUIET)
option(BUILD_WITH_ZED "Build the ZED camera and SVO playback backends" ${ZED_FOUND})
if(BUILD_WITH_ZED AND NOT ZED_FOUND)
    # Asked for explicitly but missing: re-run so the error names the missing SDK.
    find_package(ZED 5 REQUIRED)
endif()
message(STATUS "BUILD_WITH_ZED: ${BUILD_WITH_ZED}")
find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)

# CUDA sources (the GPU preprocess kernels). nvcc is taken from the toolkit FindCUDA picked, since
# /usr/local/cuda/bin is not on PATH on the Jetsons, and the kernels have to be built by the same
# toolkit whose static runtime FindCUDA links (CUDA_USE_STATIC_CUDA_RUNTIME): one runtime per
# process, shared by TensorRT's stream and the kernels.
if(NOT CMAKE_CUDA_COMPILER)
    set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")
endif()
set(CMAKE_CUDA_RUNTIME_LIBRARY Static)
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        set(CMAKE_CUDA_ARCHITECTURES 87)
    else()
        set(CMAKE_CUDA_ARCHITECTURES 86-real 89)
    endif()
endif()
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

if(BUILD_WITH_ZED)
    include_directories(SYSTEM ${ZED_INCLUDE_DIRS})
endif()
include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
include_directories(SYSTEM ${TensorRT_INCLUDE_DIRS})
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# Foxglove SDK: C++ wrapper sources compiled here on top of the prebuilt Rust static
# library. install/install_foxglove_sdk.sh unpacks the release for this architecture into
# third_party/foxglove. The SDK's own headers trip -Wdeprecated-declarations under C++20, so
# they are system headers and the wrapper compiles with warnings off; nothing of ours is in it.
set(FOXGLOVE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/foxglove")
if(NOT EXISTS "${FOXGLOVE_SDK_DIR}/include/foxglove/schemas.hpp")
    message(FATAL_ERROR
        "Foxglove SDK not found at ${FOXGLOVE_SDK_DIR}. Run install/install_foxglove_sdk.sh")
endif()
file(GLOB FOXGLOVE_SDK_SOURCES CONFIGURE_DEPENDS
    "${FOXGLOVE_SDK_DIR}/src/*.cpp"
    "${FOXGLOVE_SDK_DIR}/src/server/*.cpp"
)
add_library(foxglove_sdk STATIC ${FOXGLOVE_SDK_SOURCES})
set_target_properties(foxglove_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(foxglove_sdk SYSTEM PUBLIC "${FOXGLOVE_SDK_DIR}/include")
target_compile_options(foxglove_sdk PRIVATE -w -O2)
target_link_libraries(foxglove_sdk PUBLIC
    "${FOXGLOVE_SDK_DIR}/lib/libfoxglove.a"
    Threads::Threads
    ${CMAKE_DL_LIBS}
    m
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(BUILD_WITH_ZED)
    link_directories(${ZED_LIBRARY_DIR})
endif()
link_directories(${CUDA_LIBRARY_DIRS})

file(GLOB_RECURSE AUTO_BATTLEBOT_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/**/*.cpp"
    "src/*.cu"
    "src/**/*.cu"
)
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*main\\.cpp$")
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*/rgbd_camera/mcap_impl\\.cpp$")
if(NOT BUILD_WITH_ZED)
    list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*/rgbd_camera/zed_.*\\.cpp$")
    list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*/rgbd_camera/svo_recorder\\.cpp$")
    list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*/field_filter/point_cloud_.*\\.cpp$")
endif()

# NVJPG, the Jetson's JPEG engine, through NvJPEGEncoder in the Jetson Multimedia API. Compiled
# in when the API headers and L4T's libnvjpeg are both present, the way BUILD_WITH_ZED follows the
# SDK. That libnvjpeg is NVIDIA's libjpeg with the NVJPG extensions, not the CUDA toolkit's nvJPEG,
# so it is linked by full path and the toolkit one never reaches the link line.
set(JETSON_MMAPI_DIR "/usr/src/jetson_multimedia_api" CACHE PATH "Jetson Multimedia API root")
find_path(JETSON_MMAPI_INCLUDE_DIR NvJpegEncoder.h
    PATHS "${JETSON_MMAPI_DIR}/include" NO_DEFAULT_PATH)
find_library(L4T_NVJPEG_LIBRARY nvjpeg
    PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra NO_DEFAULT_PATH)
find_library(L4T_NVBUFSURFACE_LIBRARY nvbufsurface
    PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra NO_DEFAULT_PATH)
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*/viz/nvjpg_plugin/.*")
if(JETSON_MMAPI_INCLUDE_DIR AND L4T_NVJPEG_LIBRARY)
    set(HAVE_NVJPG ON)
else()
    set(HAVE_NVJPG OFF)
    list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*/viz/nvjpg_jpeg_encoder\\.cpp$")
endif()
message(STATUS "NVJPG JPEG encoder: ${HAVE_NVJPG}")

if(NOT GST_FOUND)
    list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*/rgbd_camera/gst_h264_backend\\.cpp$")
endif()
message(STATUS "GStreamer H.264 encoder (nvv4l2h264enc): ${GST_FOUND}")

# The ESP-NOW wire format, compiled by the robot firmware, the dongle firmware, and this library
# from one source so the three ends cannot drift (firmware/shared/espnow_protocol/).
set(ESPNOW_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/firmware/shared/espnow_protocol/src")
list(APPEND AUTO_BATTLEBOT_SOURCES "${ESPNOW_PROTOCOL_DIR}/espnow_protocol.cpp")

add_library(auto_battlebot_lib SHARED
    ${AUTO_BATTLEBOT_SOURCES}
)
if(GST_FOUND)
    target_link_libraries(auto_battlebot_lib PUBLIC PkgConfig::GST)
    target_compile_definitions(auto_battlebot_lib PUBLIC AUTO_BATTLEBOT_HAVE_GSTREAMER)
endif()

if(HAVE_NVJPG)
    # libnvjpeg exports libjpeg's API unversioned, and anything that links it puts those symbols
    # in the global scope, where they take over OpenCV's libjpeg calls. So NvJPEGEncoder and
    # libnvjpeg live in a plugin that only NvjpgJpegEncoder loads (see its header), and
    # auto_battlebot_lib never links libnvjpeg.
    set(JETSON_MMAPI_CLASSES "${JETSON_MMAPI_DIR}/samples/common/classes")
    add_library(auto_battlebot_nvjpg MODULE
        src/viz/nvjpg_plugin/nvjpg_plugin.cpp
        "${JETSON_MMAPI_CLASSES}/NvJpegEncoder.cpp"
        "${JETSON_MMAPI_CLASSES}/NvBuffer.cpp"
        "${JETSON_MMAPI_CLASSES}/NvElement.cpp"
        "${JETSON_MMAPI_CLASSES}/NvElementProfiler.cpp"
        "${JETSON_MMAPI_CLASSES}/NvLogging.cpp"
    )
    set_target_properties(auto_battlebot_nvjpg PROPERTIES
        CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
    target_include_directories(auto_battlebot_nvjpg PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_include_directories(auto_battlebot_nvjpg SYSTEM PRIVATE
        "${JETSON_MMAPI_INCLUDE_DIR}" "${JETSON_MMAPI_INCLUDE_DIR}/libjpeg-8b")
    set_source_files_properties(
        "${JETSON_MMAPI_CLASSES}/NvJpegEncoder.cpp"
        "${JETSON_MMAPI_CLASSES}/NvBuffer.cpp"
        "${JETSON_MMAPI_CLASSES}/NvElement.cpp"
        "${JETSON_MMAPI_CLASSES}/NvElementProfiler.cpp"
        "${JETSON_MMAPI_CLASSES}/NvLogging.cpp"
        PROPERTIES COMPILE_OPTIONS "-w")
    target_link_libraries(auto_battlebot_nvjpg PRIVATE "${L4T_NVJPEG_LIBRARY}" Threads::Threads)
    if(L4T_NVBUFSURFACE_LIBRARY)
        target_link_libraries(auto_battlebot_nvjpg PRIVATE "${L4T_NVBUFSURFACE_LIBRARY}")
    endif()
    add_dependencies(auto_battlebot_lib auto_battlebot_nvjpg)
    target_compile_definitions(auto_battlebot_lib PUBLIC AUTO_BATTLEBOT_HAVE_NVJPG
        AUTO_BATTLEBOT_NVJPG_PLUGIN="$<TARGET_FILE_NAME:auto_battlebot_nvjpg>")
endif()

add_custom_target(generate_git_version
    COMMAND ${CMAKE_COMMAND}
        -DGIT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -DGIT_OUTPUT=${CMAKE_CURRENT_BINARY_DIR}/git_version.txt
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/git_version.cmake
    BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/git_version.txt
    COMMENT "Generating git version stamp"
)
set_target_properties(auto_battlebot_lib PROPERTIES
    OUTPUT_NAME auto_battlebot
    VERSION 1.0.0
    SOVERSION 1
)

target_include_directories(auto_battlebot_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ESPNOW_PROTOCOL_DIR}
)
target_link_libraries(auto_battlebot_lib PUBLIC
    foxglove_sdk
    tomlplusplus::tomlplusplus
    magic_enum::magic_enum
    Eigen3::Eigen
    ${OpenCV_LIBS}
    Threads::Threads
    PkgConfig::FFMPEG
    mcap_reader
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${TensorRT_LIBRARIES}
    ${PCL_LIBRARIES}
    spdlog::spdlog
    nlohmann_json::nlohmann_json
)
target_link_libraries(auto_battlebot_lib PRIVATE rtneural)

set(COMMON_COMPILE_FLAGS
    -Wall
    -Wextra
    -Werror
    -O2
)

# No -DDEBUG: nothing in the tree tests it, and the macro collides with
# foxglove::schemas::Log::LogLevel::DEBUG in the SDK headers.
set(DEBUG_COMPILE_FLAGS
    -g
)

set(RELEASE_COMPILE_FLAGS
    -DNDEBUG
)

if(BUILD_WITH_ZED)
    target_compile_definitions(auto_battlebot_lib PUBLIC BUILD_WITH_ZED)
endif()

# nvcc rejects the GCC warning flags, so .cu files pass them through to the host compiler and
# promote nvcc's own warnings separately. No -O here: CMAKE_BUILD_TYPE already passes one, and
# nvcc warns on a second. Device code is optimized by ptxas regardless of the host -O level.
set(CUDA_COMPILE_FLAGS
    -Xcompiler=-Wall,-Wextra,-Werror
    --Werror=all-warnings
)

if(BUILD_DEBUG)
    target_compile_options(auto_battlebot_lib PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:${COMMON_COMPILE_FLAGS};${DEBUG_COMPILE_FLAGS}>"
        "$<$<COMPILE_LANGUAGE:CUDA>:${CUDA_COMPILE_FLAGS};${DEBUG_COMPILE_FLAGS}>"
    )
else()
    target_compile_options(auto_battlebot_lib PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:${COMMON_COMPILE_FLAGS};${RELEASE_COMPILE_FLAGS}>"
        "$<$<COMPILE_LANGUAGE:CUDA>:${CUDA_COMPILE_FLAGS};${RELEASE_COMPILE_FLAGS}>"
    )
endif()

# ZED SDK doesn't support debug builds - override flags for ZED-related source files
if(BUILD_DEBUG AND BUILD_WITH_ZED)
    set_source_files_properties(
        src/rgbd_camera/zed_rgbd_camera.hpp
        src/rgbd_camera/zed_rgbd_camera.cpp
        src/rgbd_camera/zed_device.cpp
        src/rgbd_camera/zed_svo_playback_camera.cpp
        src/rgbd_camera/zed_one_rgb_camera.cpp
        src/rgbd_camera/svo_recorder.cpp
        src/rgbd_camera/config.cpp
        PROPERTIES COMPILE_FLAGS "-Wall -Wno-extra -Wno-error -O2 -DNDEBUG"
    )
endif()

add_executable(auto_battlebot
    src/main.cpp
)

target_link_libraries(auto_battlebot
    auto_battlebot_lib
    CLI11::CLI11
)
add_dependencies(auto_battlebot generate_git_version)

add_executable(viz_relay
    src/viz_relay/main.cpp
)
target_include_directories(viz_relay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(viz_relay SYSTEM PRIVATE
    "${crow_SOURCE_DIR}/include"
    "${asio_SOURCE_DIR}/asio/include"
)
target_compile_definitions(viz_relay PRIVATE ASIO_STANDALONE ASIO_NO_DEPRECATED)
target_link_libraries(viz_relay
    foxglove_sdk
    CLI11::CLI11
    spdlog::spdlog
    Threads::Threads
)

add_executable(remote_protocol_dump
    tools/remote_protocol_dump.cpp
)
target_link_libraries(remote_protocol_dump auto_battlebot_lib)
target_compile_options(remote_protocol_dump PRIVATE ${COMMON_COMPILE_FLAGS})

if(BUILD_DEBUG)
    target_compile_options(auto_battlebot PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${DEBUG_COMPILE_FLAGS}
    )
    target_compile_options(viz_relay PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${DEBUG_COMPILE_FLAGS}
    )
else()
    target_compile_options(auto_battlebot PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${RELEASE_COMPILE_FLAGS}
    )
    target_compile_options(viz_relay PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${RELEASE_COMPILE_FLAGS}
    )
endif()

# Copy config directory to build directory.
# Runs every build (custom targets are always out of date). Do not add a
# file(GLOB) + DEPENDS here: the glob is captured at configure time, so renaming
# or deleting a config leaves a stale path in the dependency list and the build
# fails with "No rule to make target 'config/<old_name>.toml'". Removing the
# destination first also drops orphaned copies of renamed/deleted configs.
add_custom_target(copy_config_files ALL
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_BINARY_DIR}/config
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/config
    ${CMAKE_BINARY_DIR}/config
    COMMENT "Copying config directory to build directory"
)
add_dependencies(auto_battlebot copy_config_files)

add_custom_command(TARGET auto_battlebot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_SOURCE_DIR}/data
    ${CMAKE_BINARY_DIR}/data
    COMMENT "Creating symlink to data directory"
)

set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../lib64")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set_target_properties(auto_battlebot PROPERTIES INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../lib64")

install(TARGETS auto_battlebot auto_battlebot_lib viz_relay
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)
# NvjpgJpegEncoder loads the plugin from the directory auto_battlebot_lib is in.
if(HAVE_NVJPG)
    install(TARGETS auto_battlebot_nvjpg LIBRARY DESTINATION lib)
endif()
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/git_version.txt DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/auto_battlebot/config
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING PATTERN "*.toml"
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/dist/ DESTINATION bin/web OPTIONAL)

if(BUILD_TESTING)
    enable_testing()

    file(GLOB_RECURSE AUTO_BATTLEBOT_TEST_SOURCES CONFIGURE_DEPENDS
        "tests/*.cpp"
        "tests/**/*.cpp"
    )

    add_executable(auto_battlebot_test
        ${AUTO_BATTLEBOT_TEST_SOURCES}
    )

    target_link_libraries(auto_battlebot_test
        GTest::gtest_main
        auto_battlebot_lib
    )

    target_include_directories(auto_battlebot_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )

    include(GoogleTest)
    gtest_discover_tests(auto_battlebot_test)
endif()

