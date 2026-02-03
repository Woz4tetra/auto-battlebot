# FindTensorRT.cmake - Find TensorRT (nvinfer + nvonnxparser) for building engines from ONNX.
# Used by trt_onnx_builder. Supports system install (e.g. libnvinfer-dev on Ubuntu/Jetson) and TENSORRT_ROOT.

if(TensorRT_FOUND)
    return()
endif()

set(TensorRT_ROOT "$ENV{TENSORRT_ROOT}" CACHE PATH "TensorRT installation root")
if(TensorRT_DIR AND NOT TensorRT_ROOT)
    set(TensorRT_ROOT "${TensorRT_DIR}")
endif()

# Search order must match auto-battlebot cmake/FindTensorRT.cmake: TENSORRT_ROOT first, then system
set(_TensorRT_SEARCH_PATHS)
if(TensorRT_ROOT)
    list(APPEND _TensorRT_SEARCH_PATHS
        "${TensorRT_ROOT}"
        "${TensorRT_ROOT}/include"
        "${TensorRT_ROOT}/lib"
        "${TensorRT_ROOT}/lib/x86_64-linux-gnu"
        "${TensorRT_ROOT}/lib/aarch64-linux-gnu"
    )
endif()
list(APPEND _TensorRT_SEARCH_PATHS "/usr" "/usr/local")

find_path(TensorRT_INCLUDE_DIR
    NAMES NvInfer.h NvOnnxParser.h
    PATHS ${_TensorRT_SEARCH_PATHS}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
)
if(NOT TensorRT_INCLUDE_DIR)
    find_path(TensorRT_INCLUDE_DIR
        NAMES NvInfer.h NvOnnxParser.h
        PATH_SUFFIXES include
    )
endif()

find_library(TensorRT_NVINFER_LIBRARY
    NAMES nvinfer
    PATHS ${_TensorRT_SEARCH_PATHS}
    PATH_SUFFIXES lib lib/x86_64-linux-gnu lib/aarch64-linux-gnu
    NO_DEFAULT_PATH
)
if(NOT TensorRT_NVINFER_LIBRARY)
    find_library(TensorRT_NVINFER_LIBRARY NAMES nvinfer PATH_SUFFIXES lib lib/x86_64-linux-gnu lib/aarch64-linux-gnu)
endif()

find_library(TensorRT_NVONNXPARSER_LIBRARY
    NAMES nvonnxparser
    PATHS ${_TensorRT_SEARCH_PATHS}
    PATH_SUFFIXES lib lib/x86_64-linux-gnu lib/aarch64-linux-gnu
    NO_DEFAULT_PATH
)
if(NOT TensorRT_NVONNXPARSER_LIBRARY)
    find_library(TensorRT_NVONNXPARSER_LIBRARY NAMES nvonnxparser PATH_SUFFIXES lib lib/x86_64-linux-gnu lib/aarch64-linux-gnu)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TensorRT
    REQUIRED_VARS TensorRT_INCLUDE_DIR TensorRT_NVINFER_LIBRARY TensorRT_NVONNXPARSER_LIBRARY
    FAIL_MESSAGE "TensorRT (nvinfer + nvonnxparser) not found. Install libnvinfer-dev or set TENSORRT_ROOT."
)

if(TensorRT_FOUND)
    set(TensorRT_INCLUDE_DIRS "${TensorRT_INCLUDE_DIR}")
    set(TensorRT_LIBRARIES "${TensorRT_NVINFER_LIBRARY}" "${TensorRT_NVONNXPARSER_LIBRARY}")
endif()
