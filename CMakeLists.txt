cmake_minimum_required(VERSION 3.14)
project(auto_battlebot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Option to enable testing
option(BUILD_TESTING "Build tests" OFF)

# Option to enable debug build
option(BUILD_DEBUG "Build with debug flags" OFF)

if(BUILD_TESTING)
    message("Building tests")
endif()

if(BUILD_DEBUG)
    message("Building with debug flags")
endif()

# Fetch external dependencies from git
include(FetchContent)

FetchContent_Declare(
    tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG v3.4.0
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE # Don't try to update if already fetched
)

FetchContent_Declare(
    cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG v2.4.1
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE # Don't try to update if already fetched
)

FetchContent_Declare(
    magic_enum
    GIT_REPOSITORY https://github.com/Neargye/magic_enum.git
    GIT_TAG v0.9.6
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE # Don't try to update if already fetched
)

FetchContent_Declare(
    lvgl
    GIT_REPOSITORY https://github.com/lvgl/lvgl.git
    GIT_TAG v9.3.0
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

# MCAP reader. The Foxglove SDK writes but does not read, and VideoPlaybackCamera has to read
# /camera/video back out of a recording.
FetchContent_Declare(
    mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG releases/cpp/v1.4.0
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
)

# JSON for the remote protocol (include/remote/). The URL tarball is the release's single-header
# CMake package, which is far smaller than the repository.
FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.14.1
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
)
set(SPDLOG_BUILD_PIC ON CACHE BOOL "" FORCE)

# LVGL config path (must be set before MakeAvailable)
set(LV_BUILD_CONF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/ui/lv_conf.h" CACHE PATH "")
# Disable demos and examples to speed up build
set(CONFIG_LV_BUILD_DEMOS OFF CACHE BOOL "")
set(CONFIG_LV_BUILD_EXAMPLES OFF CACHE BOOL "")

# Fetch googletest only if testing is enabled
if(BUILD_TESTING)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
        UPDATE_DISCONNECTED TRUE # Don't try to update if already fetched
    )

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(tomlplusplus cli11 magic_enum lvgl spdlog nlohmann_json googletest)
else()
    FetchContent_MakeAvailable(tomlplusplus cli11 magic_enum lvgl spdlog nlohmann_json)
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

# Find required packages
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Threads REQUIRED)
# SDL2: try config first, then pkg-config
find_package(SDL2 QUIET)
if(NOT SDL2_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
        set(SDL2_TARGET PkgConfig::SDL2)
    else()
        find_package(SDL2 REQUIRED)
        set(SDL2_TARGET SDL2::SDL2)
    endif()
else()
    set(SDL2_TARGET SDL2::SDL2)
endif()
# 4.10 asserted, not preferred: cv::aruco moved from contrib into core opencv_objdetect at 4.7
# and the API changed with it, so FiducialFieldFilter needs 4.7+ everywhere or it needs version
# guards it would never shed. apt ships 4.5.4 on jammy and 4.6.0 on noble, both too old;
# install/install_opencv.sh builds 4.10.0 into /usr/local on every platform, which is what the
# Jetson already ran. If this fails after that install, build/CMakeCache.txt still pins the old
# OpenCV_DIR: run ./scripts/clean_build.sh.
find_package(OpenCV 4.10 REQUIRED COMPONENTS core imgproc highgui dnn objdetect calib3d)
find_package(Eigen3 REQUIRED)
# PCL is only reachable from PointCloudFieldFilter and data_structures/point_cloud.hpp. It stays
# unconditional while FieldDescriptionWithInlierPoints carries a cloud; it goes when the depth
# field filter does.
find_package(PCL REQUIRED COMPONENTS common sample_consensus)

# H.264 for /camera/video: encode on the RGB camera, decode on playback. One dependency covering
# h264_nvenc on the desktop, h264_nvv4l2m2m on the Jetson, and libx264 as the fallback.
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)

# ZED SDK. The default follows whether the SDK is actually installed: on everywhere it is,
# because every SVO in data/svo/, the keypoint corpora, and the eval ground truth are tied to
# this path; off on the Jetson, which runs the RGB camera and has no SDK. Defaulting to ON
# unconditionally aborted configure on the Jetson before the option could ever be read.
find_package(ZED 5 QUIET)
option(BUILD_WITH_ZED "Build the ZED camera and SVO playback backends" ${ZED_FOUND})
if(BUILD_WITH_ZED AND NOT ZED_FOUND)
    # Asked for explicitly but missing: re-run so the error names the missing SDK.
    find_package(ZED 5 REQUIRED)
endif()
message(STATUS "BUILD_WITH_ZED: ${BUILD_WITH_ZED}")
find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)

# Mark SDK headers as system headers to suppress their warnings
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

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# Note: tomlplusplus include directories are handled automatically by FetchContent

# Link directories
if(BUILD_WITH_ZED)
    link_directories(${ZED_LIBRARY_DIR})
endif()
link_directories(${CUDA_LIBRARY_DIRS})

# Create shared library with core auto_battlebot functionality
file(GLOB_RECURSE AUTO_BATTLEBOT_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/**/*.cpp"
)
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*main\\.cpp$")
# Keep LVGL/SDL UI units out of the shared library.
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*lvgl_ui.*\\.cpp$")
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*/rgbd_camera/mcap_impl\\.cpp$")
# The glob is recursive, so dropping the ZED backends means filtering them out here rather than
# wrapping an explicit list. PointCloudFieldFilter goes with them: it needs a depth image, and
# outside simulation the ZED is the only camera that produces one.
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
if(JETSON_MMAPI_INCLUDE_DIR AND L4T_NVJPEG_LIBRARY)
    set(HAVE_NVJPG ON)
else()
    set(HAVE_NVJPG OFF)
    list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*/viz/nvjpg_jpeg_encoder\\.cpp$")
endif()
message(STATUS "NVJPG JPEG encoder: ${HAVE_NVJPG}")

add_library(auto_battlebot_lib SHARED
    ${AUTO_BATTLEBOT_SOURCES}
)

if(HAVE_NVJPG)
    set(JETSON_MMAPI_CLASSES "${JETSON_MMAPI_DIR}/samples/common/classes")
    add_library(jetson_mmapi_jpeg STATIC
        "${JETSON_MMAPI_CLASSES}/NvJpegEncoder.cpp"
        "${JETSON_MMAPI_CLASSES}/NvBuffer.cpp"
        "${JETSON_MMAPI_CLASSES}/NvElement.cpp"
        "${JETSON_MMAPI_CLASSES}/NvElementProfiler.cpp"
        "${JETSON_MMAPI_CLASSES}/NvLogging.cpp"
    )
    set_target_properties(jetson_mmapi_jpeg PROPERTIES POSITION_INDEPENDENT_CODE ON)
    # NVIDIA's sample classes, not ours: system headers and no warnings.
    target_include_directories(jetson_mmapi_jpeg SYSTEM PUBLIC
        "${JETSON_MMAPI_INCLUDE_DIR}" "${JETSON_MMAPI_INCLUDE_DIR}/libjpeg-8b")
    target_compile_options(jetson_mmapi_jpeg PRIVATE -w)
    target_link_libraries(jetson_mmapi_jpeg PUBLIC "${L4T_NVJPEG_LIBRARY}" Threads::Threads)
    if(L4T_NVBUFSURFACE_LIBRARY)
        target_link_libraries(jetson_mmapi_jpeg PUBLIC "${L4T_NVBUFSURFACE_LIBRARY}")
    endif()
    target_link_libraries(auto_battlebot_lib PUBLIC jetson_mmapi_jpeg)
    target_compile_definitions(auto_battlebot_lib PUBLIC AUTO_BATTLEBOT_HAVE_NVJPG)
endif()

# UI sources are built as a dedicated static library.
# These sources are isolated under lvgl_platform_bound because they are tightly
# coupled to LVGL/SDL runtime and desktop/UI-thread behavior.
file(GLOB AUTO_BATTLEBOT_UI_SOURCES CONFIGURE_DEPENDS
    "src/ui/lvgl_platform_bound/*.cpp"
)

add_library(auto_battlebot_ui STATIC
    ${AUTO_BATTLEBOT_UI_SOURCES}
)

# Brand assets (splash animation, top-bar logo) are embedded as C arrays at build
# time so the binary has no runtime file dependencies. Symbols are declared in
# src/ui/lvgl_platform_bound/lvgl_ui_branding.cpp.
function(embed_brand_asset symbol input)
    set(generated ${CMAKE_CURRENT_BINARY_DIR}/generated/brand_assets/${symbol}.cpp)
    add_custom_command(
        OUTPUT ${generated}
        COMMAND ${CMAKE_COMMAND}
            -DINPUT=${input} -DOUTPUT=${generated} -DSYMBOL=${symbol}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/bin2c.cmake
        DEPENDS ${input} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/bin2c.cmake
        COMMENT "Embedding brand asset ${symbol}"
    )
    target_sources(auto_battlebot_ui PRIVATE ${generated})
endfunction()

embed_brand_asset(bwbots_splash_gif
    ${CMAKE_CURRENT_SOURCE_DIR}/logo/bwbots_splash_1024x600.gif)
embed_brand_asset(bwbots_logo_png
    ${CMAKE_CURRENT_SOURCE_DIR}/logo/bwbots_logo_dark_h24.png)

target_include_directories(auto_battlebot_ui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ui
)

# Git version stamp, regenerated every build so the -dirty flag stays accurate.
# Written next to the binary and read at startup by the System tab in
# src/ui/lvgl_ui.cpp, so a new commit never recompiles or relinks anything.
add_custom_target(generate_git_version
    COMMAND ${CMAKE_COMMAND}
        -DGIT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -DGIT_OUTPUT=${CMAKE_CURRENT_BINARY_DIR}/git_version.txt
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/git_version.cmake
    BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/git_version.txt
    COMMENT "Generating git version stamp"
)
add_dependencies(auto_battlebot_ui generate_git_version)

target_link_libraries(auto_battlebot_ui PUBLIC
    lvgl::lvgl
    ${SDL2_TARGET}
    spdlog::spdlog
    magic_enum::magic_enum
    tomlplusplus::tomlplusplus
    nlohmann_json::nlohmann_json
)

# Set library properties
set_target_properties(auto_battlebot_lib PROPERTIES
    OUTPUT_NAME auto_battlebot
    VERSION 1.0.0
    SOVERSION 1
)

# Include directories for the library
target_include_directories(auto_battlebot_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# Link libraries to shared library
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

# Common compiler flags
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

# Compiler flags for the library
if(BUILD_DEBUG)
    target_compile_options(auto_battlebot_lib PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${DEBUG_COMPILE_FLAGS}
    )
else()
    target_compile_options(auto_battlebot_lib PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${RELEASE_COMPILE_FLAGS}
    )
endif()

# ZED SDK doesn't support debug builds - override flags for ZED-related source files
if(BUILD_DEBUG AND BUILD_WITH_ZED)
    set_source_files_properties(
        src/rgbd_camera/zed_rgbd_camera.hpp
        src/rgbd_camera/zed_rgbd_camera.cpp
        src/rgbd_camera/zed_device.cpp
        src/rgbd_camera/zed_svo_playback_camera.cpp
        src/rgbd_camera/svo_recorder.cpp
        src/rgbd_camera/config.cpp
        PROPERTIES COMPILE_FLAGS "-Wall -Wno-extra -Wno-error -O2 -DNDEBUG"
    )
endif()

# Create main executable
add_executable(auto_battlebot
    src/main.cpp
)

target_link_libraries(auto_battlebot
    auto_battlebot_lib
    auto_battlebot_ui
    CLI11::CLI11
    lvgl::lvgl
    ${SDL2_TARGET}
)

# Foxglove relay: owns the WebSocket server so Foxglove stays connected across app restarts.
# Links only the SDK and CLI11, never auto_battlebot_lib, so it starts without the ZED SDK.
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

# Prints every remote status topic and command with its JSON Schema. web/scripts/gen-protocol.mjs
# turns the output into web/src/generated/protocol.ts.
add_executable(remote_protocol_dump
    tools/remote_protocol_dump.cpp
)
target_link_libraries(remote_protocol_dump auto_battlebot_lib)
target_compile_options(remote_protocol_dump PRIVATE ${COMMON_COMPILE_FLAGS})

# Compiler flags for executable
if(BUILD_DEBUG)
    target_compile_options(auto_battlebot_ui PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${DEBUG_COMPILE_FLAGS}
    )
    target_compile_options(auto_battlebot PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${DEBUG_COMPILE_FLAGS}
    )
    target_compile_options(viz_relay PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${DEBUG_COMPILE_FLAGS}
    )
else()
    target_compile_options(auto_battlebot_ui PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${RELEASE_COMPILE_FLAGS}
    )
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

# Create symlink to data directory in build directory
add_custom_command(TARGET auto_battlebot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_SOURCE_DIR}/data
    ${CMAKE_BINARY_DIR}/data
    COMMENT "Creating symlink to data directory"
)

# So the installed executable finds libauto_battlebot.so in PREFIX/lib when run from PREFIX/bin
# $ORIGIN = directory of the executable; escape $ so CMake passes it to the linker
set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../lib64")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set_target_properties(auto_battlebot PROPERTIES INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../lib64")

# Install targets
install(TARGETS auto_battlebot auto_battlebot_lib viz_relay
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)
# The binary reads this at startup from its own directory (see read_build_version
# in src/ui/lvgl_ui.cpp), so it must land next to the installed executable.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/git_version.txt DESTINATION bin)
# Install config (used when running from system-wide install)
install(DIRECTORY config/ DESTINATION share/auto_battlebot/config
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING PATTERN "*.toml"
)

# The built web dashboard, next to viz_relay, which serves <exe_dir>/web. OPTIONAL: a machine
# without Node still installs, and the relay answers 503 with a note.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/dist/ DESTINATION bin/web OPTIONAL)

# Conditionally enable testing
if(BUILD_TESTING)
    enable_testing()

    file(GLOB_RECURSE AUTO_BATTLEBOT_TEST_SOURCES CONFIGURE_DEPENDS
        "tests/*.cpp"
        "tests/**/*.cpp"
    )

    # Test executable for data structures
    add_executable(auto_battlebot_test
        ${AUTO_BATTLEBOT_TEST_SOURCES}
    )

    target_link_libraries(auto_battlebot_test
        GTest::gtest_main
        auto_battlebot_lib
        auto_battlebot_ui
    )

    # So shared test helpers can be included by module path, e.g.
    # "engine_selector/make_test_selector.hpp".
    target_include_directories(auto_battlebot_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )

    include(GoogleTest)
    gtest_discover_tests(auto_battlebot_test)
endif()

