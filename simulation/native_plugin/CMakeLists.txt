cmake_minimum_required(VERSION 3.18)

# Help CMake find CUDA compiler if not already set
# This must be done BEFORE project() when using LANGUAGES CUDA
if(NOT DEFINED CMAKE_CUDA_COMPILER)
    find_program(CMAKE_CUDA_COMPILER nvcc
        HINTS
            ENV CUDA_PATH
            ENV CUDA_HOME
            /usr/local/cuda
        PATH_SUFFIXES bin
    )
endif()

# Enable CUDA language
project(CudaInteropPlugin LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# Options
# ============================================================================

option(ENABLE_CUDA "Enable CUDA backend (required for CUDA Interop)" ON)
option(ENABLE_OPENGL "Enable OpenGL support" ON)
option(ENABLE_VULKAN "Enable Vulkan support for CUDA external memory interop" ON)

# ============================================================================
# Find Dependencies
# ============================================================================

# CUDA (required)
if(ENABLE_CUDA)
    find_package(CUDAToolkit REQUIRED)
    message(STATUS "CUDA Toolkit found: ${CUDAToolkit_VERSION}")
    message(STATUS "CUDA include dir: ${CUDAToolkit_INCLUDE_DIRS}")
endif()

# OpenGL (optional, for CUDA-GL interop)
if(ENABLE_OPENGL)
    find_package(OpenGL REQUIRED)
    message(STATUS "OpenGL found")
endif()

# Vulkan (optional, for CUDA external memory interop)
if(ENABLE_VULKAN)
    find_package(Vulkan QUIET)
    if(Vulkan_FOUND)
        message(STATUS "Vulkan found: ${Vulkan_INCLUDE_DIRS}")
    else()
        message(STATUS "Vulkan not found - Vulkan support will be disabled")
        set(ENABLE_VULKAN OFF)
    endif()
endif()

# ============================================================================
# CUDA Interop Plugin (Primary)
# ============================================================================

set(CUDA_INTEROP_SOURCES
    src/cuda_interop.cu
)

set(CUDA_INTEROP_HEADERS
    include/cuda_interop.h
)

# Create shared library
add_library(CudaInteropPlugin SHARED ${CUDA_INTEROP_SOURCES} ${CUDA_INTEROP_HEADERS})

# Set CUDA architectures
# CUDA 13.x supports sm_75 and newer only
# Turing: sm_75 (RTX 20xx, GTX 16xx)
# Ampere: sm_80, sm_86 (A100, RTX 30xx)
# Jetson Orin: sm_87
# Ada Lovelace: sm_89 (RTX 40xx)
# Hopper: sm_90 (H100)
# Blackwell: sm_100 (RTX 50xx)
set_target_properties(CudaInteropPlugin PROPERTIES
    CUDA_ARCHITECTURES "75;80;86;87;89;90"
    CUDA_SEPARABLE_COMPILATION ON
)

# Include directories
target_include_directories(CudaInteropPlugin PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(CudaInteropPlugin PRIVATE
    ${CUDAToolkit_INCLUDE_DIRS}
)

# Link libraries - start with CUDA
target_link_libraries(CudaInteropPlugin PRIVATE
    CUDA::cudart
    CUDA::cuda_driver
)

# Conditionally link OpenGL
if(ENABLE_OPENGL)
    target_link_libraries(CudaInteropPlugin PRIVATE OpenGL::GL)
endif()

# Conditionally link Vulkan
if(ENABLE_VULKAN AND Vulkan_FOUND)
    target_include_directories(CudaInteropPlugin PRIVATE ${Vulkan_INCLUDE_DIRS})
    target_link_libraries(CudaInteropPlugin PRIVATE Vulkan::Vulkan)
endif()

# Compile definitions
target_compile_definitions(CudaInteropPlugin PRIVATE
    CUDA_INTEROP_EXPORTS
)

# Conditional compile definitions for graphics API support
if(ENABLE_OPENGL)
    # OpenGL is enabled by default (no special define needed)
else()
    target_compile_definitions(CudaInteropPlugin PRIVATE DISABLE_OPENGL)
endif()

if(ENABLE_VULKAN AND Vulkan_FOUND)
    target_compile_definitions(CudaInteropPlugin PRIVATE ENABLE_VULKAN)
endif()

# Platform-specific settings
if(UNIX AND NOT APPLE)
    # Linux
    set_target_properties(CudaInteropPlugin PROPERTIES
        OUTPUT_NAME "CudaInteropPlugin"
        PREFIX "lib"
        SUFFIX ".so"
    )
    target_link_libraries(CudaInteropPlugin PRIVATE pthread dl)
elseif(WIN32)
    # Windows
    set_target_properties(CudaInteropPlugin PROPERTIES
        OUTPUT_NAME "CudaInteropPlugin"
        SUFFIX ".dll"
    )
endif()

# ============================================================================
# Installation
# ============================================================================

set(UNITY_PLUGINS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../auto-battlebot-sim/Assets/Plugins" 
    CACHE PATH "Unity Plugins directory")

# Install to Unity Plugins folder
install(TARGETS CudaInteropPlugin
    LIBRARY DESTINATION ${UNITY_PLUGINS_DIR}/x86_64
    RUNTIME DESTINATION ${UNITY_PLUGINS_DIR}/x86_64
)

# Install header for reference
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/cuda_interop.h
    DESTINATION ${UNITY_PLUGINS_DIR}
)

message(STATUS "")
message(STATUS "=== CUDA Interop Plugin Configuration ===")
message(STATUS "  CUDA enabled: ${ENABLE_CUDA}")
message(STATUS "  OpenGL enabled: ${ENABLE_OPENGL}")
message(STATUS "  Vulkan enabled: ${ENABLE_VULKAN}")
message(STATUS "  Install directory: ${UNITY_PLUGINS_DIR}/x86_64")
message(STATUS "==========================================")
message(STATUS "")
