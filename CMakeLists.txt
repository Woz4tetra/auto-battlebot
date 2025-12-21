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

# Fetch external dependencies from git
include(FetchContent)

FetchContent_Declare(
    miniroscpp
    GIT_REPOSITORY https://github.com/dkargin/miniroscpp.git
    GIT_TAG master
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE  # Don't try to update if already fetched
)

FetchContent_Declare(
    tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG v3.4.0
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE  # Don't try to update if already fetched
)

FetchContent_Declare(
    cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG v2.4.1
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE  # Don't try to update if already fetched
)

FetchContent_Declare(
    magic_enum
    GIT_REPOSITORY https://github.com/Neargye/magic_enum.git
    GIT_TAG v0.9.6
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE  # Don't try to update if already fetched
)

# Fetch googletest only if testing is enabled
if(BUILD_TESTING)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
        UPDATE_DISCONNECTED TRUE  # Don't try to update if already fetched
    )

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(miniroscpp tomlplusplus cli11 magic_enum googletest)
else()
    FetchContent_MakeAvailable(miniroscpp tomlplusplus cli11 magic_enum)
endif()

# Find required packages
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common sample_consensus)

# Find ZED SDK
find_package(ZED 5 REQUIRED)
find_package(CUDA REQUIRED)

# Find LibTorch
find_package(Torch REQUIRED)

# Mark ZED SDK headers as system headers to suppress their warnings
include_directories(SYSTEM ${ZED_INCLUDE_DIRS})
include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIRS})
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
    ${TORCH_LIBRARIES}
    ${PCL_LIBRARIES}
    ${TORCH_LIBRARIES}
)

# Compiler flags for the library
if(BUILD_DEBUG)
    target_compile_options(auto_battlebot_lib PRIVATE
        -Wall
        -Wextra
        -g
        -O0
        -DDEBUG
    )
else()
    target_compile_options(auto_battlebot_lib PRIVATE
        -Wall
        -Wextra
        -O2
        -DNDEBUG
    )
endif()

# Create main executable
add_executable(auto_battlebot
    src/main.cpp
)

# Link the executable to the shared library
target_link_libraries(auto_battlebot
    auto_battlebot_lib
    CLI11::CLI11
)

# Compiler flags for executable
if(BUILD_DEBUG)
    target_compile_options(auto_battlebot PRIVATE
        -Wall
        -Wextra
        -g
        -O0
        -DDEBUG
    )
else()
    target_compile_options(auto_battlebot PRIVATE
        -Wall
        -Wextra
        -O2
        -DNDEBUG
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

# Install targets
install(TARGETS auto_battlebot auto_battlebot_lib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
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
