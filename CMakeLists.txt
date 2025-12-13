cmake_minimum_required(VERSION 3.14)
project(auto_battlebot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to enable testing
option(BUILD_TESTING "Build tests" OFF)

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

    FetchContent_MakeAvailable(miniroscpp tomlplusplus googletest)
else()
    FetchContent_MakeAvailable(miniroscpp tomlplusplus)
endif()

# Find required packages
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(Eigen3 REQUIRED)

# Find ZED SDK
find_package(ZED 5 REQUIRED)
find_package(CUDA REQUIRED)
# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
# Note: miniroscpp and tomlplusplus include directories are handled automatically by FetchContent

# Link directories
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

# Create shared library with all auto_battlebot functionality
add_library(auto_battlebot_lib SHARED
    src/config.cpp
    src/runner.cpp
    src/enums/group.cpp
    src/enums/keypoint_label.cpp
    src/enums/label.cpp
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
    Eigen3::Eigen
    ${OpenCV_LIBS}
    Threads::Threads
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES}
)

# Compiler flags for the library
target_compile_options(auto_battlebot_lib PRIVATE
    -Wall
    -Wextra
    -O2
)

# Create main executable
add_executable(auto_battlebot
    src/main.cpp
)

# Link the executable to the shared library
target_link_libraries(auto_battlebot
    auto_battlebot_lib
)

# Compiler flags for executable
target_compile_options(auto_battlebot PRIVATE
    -Wall
    -Wextra
    -O2
)

# Install targets
install(TARGETS auto_battlebot auto_battlebot_lib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

# Conditionally enable testing
if(BUILD_TESTING)
    enable_testing()

    # Test executable for data structures
    add_executable(auto_battlebot_test
        tests/test_header.cpp
        tests/test_pose.cpp
        tests/test_transform.cpp
        tests/test_robot.cpp
        tests/test_keypoint.cpp
    )

    target_link_libraries(auto_battlebot_test
        GTest::gtest_main
        auto_battlebot_lib
    )

    include(GoogleTest)
    gtest_discover_tests(auto_battlebot_test)
endif()
