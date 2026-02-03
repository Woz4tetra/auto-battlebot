cmake_minimum_required(VERSION 3.18)
project(trt_onnx_builder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# Python development headers required for nanobind (e.g. python3-dev on Ubuntu)
find_package(Python 3 COMPONENTS Interpreter Development REQUIRED)

# Nanobind: fetch and use for Python extension
include(FetchContent)
FetchContent_Declare(nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG        v2.0.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nanobind)

# TensorRT (nvinfer + nvonnxparser) and CUDA (TensorRT headers include cuda_runtime_api.h)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(TensorRT REQUIRED)
find_package(CUDAToolkit REQUIRED)

# Python extension module
nanobind_add_module(trt_onnx_builder
    src/trt_onnx_builder.cpp
)
target_include_directories(trt_onnx_builder PRIVATE
    ${TensorRT_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
)
# Suppress deprecation warnings from TensorRT headers (NVIDIA API churn)
target_compile_options(trt_onnx_builder PRIVATE -Wno-deprecated-declarations)
target_link_libraries(trt_onnx_builder PRIVATE
    ${TensorRT_LIBRARIES}
)
