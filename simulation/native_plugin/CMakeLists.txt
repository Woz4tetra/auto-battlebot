cmake_minimum_required(VERSION 3.16)
project(GpuMemoryShare LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Options
option(ENABLE_CUDA "Enable CUDA backend" ON)
option(ENABLE_OPENGL "Enable OpenGL backend" ON)

# Source files
set(SOURCES
    src/gpu_memory_share.cpp
)

# Create shared library
add_library(gpu_memory_share SHARED ${SOURCES})

target_include_directories(gpu_memory_share PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# CUDA support
if(ENABLE_CUDA)
    find_package(CUDA QUIET)
    if(CUDA_FOUND)
        message(STATUS "CUDA found: ${CUDA_VERSION}")
        target_compile_definitions(gpu_memory_share PRIVATE CUDA_AVAILABLE)
        target_include_directories(gpu_memory_share PRIVATE ${CUDA_INCLUDE_DIRS})
        target_link_libraries(gpu_memory_share PRIVATE ${CUDA_LIBRARIES})
    else()
        message(STATUS "CUDA not found, disabling CUDA backend")
    endif()
endif()

# OpenGL support
if(ENABLE_OPENGL)
    find_package(OpenGL QUIET)
    if(OpenGL_FOUND)
        message(STATUS "OpenGL found")
        target_compile_definitions(gpu_memory_share PRIVATE OPENGL_AVAILABLE)
        target_link_libraries(gpu_memory_share PRIVATE OpenGL::GL)
    else()
        message(STATUS "OpenGL not found, disabling OpenGL backend")
    endif()
endif()

# Platform-specific settings
if(UNIX AND NOT APPLE)
    # Linux
    set_target_properties(gpu_memory_share PROPERTIES
        OUTPUT_NAME "GpuMemoryShare"
        SUFFIX ".so"
    )
    target_link_libraries(gpu_memory_share PRIVATE pthread)
elseif(WIN32)
    # Windows
    set_target_properties(gpu_memory_share PROPERTIES
        OUTPUT_NAME "GpuMemoryShare"
        SUFFIX ".dll"
    )
endif()

# Install to Unity Plugins folder
set(UNITY_PLUGINS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../auto-battlebot-sim/Assets/Plugins" CACHE PATH "Unity Plugins directory")

install(TARGETS gpu_memory_share
    LIBRARY DESTINATION ${UNITY_PLUGINS_DIR}/x86_64
    RUNTIME DESTINATION ${UNITY_PLUGINS_DIR}/x86_64
)

# Also create header for Unity C# bindings
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gpu_memory_share.h
    ${UNITY_PLUGINS_DIR}/gpu_memory_share.h
    COPYONLY
)

message(STATUS "GpuMemoryShare plugin will be installed to: ${UNITY_PLUGINS_DIR}")
