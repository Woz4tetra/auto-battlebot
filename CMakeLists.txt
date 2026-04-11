cmake_minimum_required(VERSION 3.14)
project(auto_battlebot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
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
    miniroscpp
    GIT_REPOSITORY https://github.com/dkargin/miniroscpp.git
    GIT_TAG master
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE # Don't try to update if already fetched
)

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

FetchContent_Declare(
    mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG releases/cpp/v1.4.0
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
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

    FetchContent_MakeAvailable(miniroscpp tomlplusplus cli11 magic_enum lvgl spdlog mcap googletest)
else()
    FetchContent_MakeAvailable(miniroscpp tomlplusplus cli11 magic_enum lvgl spdlog mcap)
endif()

# mcap is header-only; expose its cpp/ include directory
FetchContent_GetProperties(mcap SOURCE_DIR mcap_SOURCE_DIR)
set(MCAP_INCLUDE_DIR "${mcap_SOURCE_DIR}/cpp/mcap/include")

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
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common sample_consensus)

# Find ZED SDK
find_package(ZED 5 REQUIRED)
find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)

# Mark SDK headers as system headers to suppress their warnings
include_directories(SYSTEM ${ZED_INCLUDE_DIRS})
include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
include_directories(SYSTEM ${TensorRT_INCLUDE_DIRS})
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# Note: miniroscpp and tomlplusplus include directories are handled automatically by FetchContent

# Link directories
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

# Create shared library with all auto_battlebot functionality
file(GLOB_RECURSE AUTO_BATTLEBOT_SOURCES
    "src/*.cpp"
    "src/**/*.cpp"
)
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*main\\.cpp$")
# Keep LVGL/SDL UI units out of the shared library; they are linked directly
# into the auto_battlebot executable to avoid shared-lib link issues.
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*lvgl_ui.*\\.cpp$")

add_library(auto_battlebot_lib SHARED
    ${AUTO_BATTLEBOT_SOURCES}
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
target_include_directories(auto_battlebot_lib SYSTEM PUBLIC
    ${MCAP_INCLUDE_DIR}
)

# Link libraries to shared library
target_link_libraries(auto_battlebot_lib PUBLIC
    miniros::roscxx
    tomlplusplus::tomlplusplus
    magic_enum::magic_enum
    Eigen3::Eigen
    ${OpenCV_LIBS}
    Threads::Threads
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${TensorRT_LIBRARIES}
    ${PCL_LIBRARIES}
    spdlog::spdlog
)

# Common compiler flags
set(COMMON_COMPILE_FLAGS
    -Wall
    -Wextra
    -Werror
    -O2
)

set(DEBUG_COMPILE_FLAGS
    -g
    -DDEBUG
)

set(RELEASE_COMPILE_FLAGS
    -DNDEBUG
)

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
if(BUILD_DEBUG)
    set_source_files_properties(
        src/rgbd_camera/zed_rgbd_camera.hpp
        src/rgbd_camera/zed_rgbd_camera.cpp
        src/rgbd_camera/config.cpp
        PROPERTIES COMPILE_FLAGS "-Wall -Wno-extra -Wno-error -O2 -DNDEBUG"
    )
endif()

# Create main executable (UI implemented with LVGL + SDL2)
add_executable(auto_battlebot
    src/main.cpp
    src/ui/lvgl_ui.cpp
    src/ui/lvgl_ui_battery.cpp
    src/ui/lvgl_ui_overlay.cpp
    src/ui/lvgl_ui_platform.cpp
    src/ui/lvgl_ui_presenter.cpp
)

target_link_libraries(auto_battlebot
    auto_battlebot_lib
    CLI11::CLI11
    lvgl::lvgl
    ${SDL2_TARGET}
)

# Compiler flags for executable
if(BUILD_DEBUG)
    target_compile_options(auto_battlebot PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${DEBUG_COMPILE_FLAGS}
    )
else()
    target_compile_options(auto_battlebot PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${RELEASE_COMPILE_FLAGS}
    )
endif()

# Copy config directory to build directory
file(GLOB CONFIG_FILES ${CMAKE_SOURCE_DIR}/config/*)
add_custom_target(copy_config_files ALL
    DEPENDS ${CONFIG_FILES}
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/config
    ${CMAKE_BINARY_DIR}/config
    COMMENT "Copying config directory to build directory"
)
add_dependencies(auto_battlebot copy_config_files)

# Create symlink to models directory in build directory
add_custom_command(TARGET auto_battlebot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_SOURCE_DIR}/models
    ${CMAKE_BINARY_DIR}/models
    COMMENT "Creating symlink to models directory"
)

# So the installed executable finds libauto_battlebot.so in PREFIX/lib when run from PREFIX/bin
# $ORIGIN = directory of the executable; escape $ so CMake passes it to the linker
set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../lib64")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set_target_properties(auto_battlebot PROPERTIES INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../lib64")

# Install targets
install(TARGETS auto_battlebot auto_battlebot_lib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)
# Install config (used when running from system-wide install)
install(DIRECTORY config/ DESTINATION share/auto_battlebot/config
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING PATTERN "*.toml"
)

# Conditionally enable testing
if(BUILD_TESTING)
    enable_testing()

    file(GLOB_RECURSE AUTO_BATTLEBOT_TEST_SOURCES
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
    )

    include(GoogleTest)
    gtest_discover_tests(auto_battlebot_test)
endif()
