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
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui dnn)
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

# Create shared library with core auto_battlebot functionality
file(GLOB_RECURSE AUTO_BATTLEBOT_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/**/*.cpp"
)
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*main\\.cpp$")
# Every standalone executable's entry point must be excluded here, not just main.cpp: each one
# defines its own main(), and a source glob is oblivious to that, so an executable's .cpp left in
# this list gets compiled into the shared library too and collides at link time with whichever
# other main() also ends up there ("multiple definition of main").
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*scratch\\.cpp$")
# Keep LVGL/SDL UI units out of the shared library.
list(FILTER AUTO_BATTLEBOT_SOURCES EXCLUDE REGEX ".*lvgl_ui.*\\.cpp$")

add_library(auto_battlebot_lib SHARED
    ${AUTO_BATTLEBOT_SOURCES}
)

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

add_executable(scratch
    src/scratch.cpp
)
target_link_libraries(scratch
    auto_battlebot_lib
    lvgl::lvgl
)

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
else()
    target_compile_options(auto_battlebot_ui PRIVATE
        ${COMMON_COMPILE_FLAGS}
        ${RELEASE_COMPILE_FLAGS}
    )
    target_compile_options(auto_battlebot PRIVATE
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
install(TARGETS auto_battlebot auto_battlebot_lib
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
