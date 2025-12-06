cmake_minimum_required(VERSION 3.14)
project(image_delay_subscriber)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch miniroscpp from git
include(FetchContent)
FetchContent_Declare(
    miniroscpp
    GIT_REPOSITORY https://github.com/dkargin/miniroscpp.git
    GIT_TAG master
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE  # Don't try to update if already fetched
)

FetchContent_MakeAvailable(miniroscpp)

# Find required packages
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# Note: miniroscpp include directories are handled automatically by FetchContent

# Create executables
add_executable(auto_battlebot
    src/main.cpp
)

add_executable(producer_consumer_example
    src/example_usage.cpp
)

# Link libraries
target_link_libraries(auto_battlebot
    miniros::roscxx
    ${OpenCV_LIBS}
    Threads::Threads
)

# Compiler flags
target_compile_options(auto_battlebot PRIVATE
    -Wall
    -Wextra
    -O2
)

# Install targets
install(TARGETS auto_battlebot
    RUNTIME DESTINATION bin
)