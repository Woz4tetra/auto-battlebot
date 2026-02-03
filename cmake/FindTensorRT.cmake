# FindTensorRT.cmake - Find TensorRT runtime (nvinfer) for loading and running .engine files.
# Supports system install (e.g. libnvinfer-dev on Ubuntu/Jetson) and TENSORRT_ROOT.

if(TensorRT_FOUND)
    return()
endif()

set(TensorRT_ROOT "$ENV{TENSORRT_ROOT}" CACHE PATH "TensorRT installation root")
if(TensorRT_DIR AND NOT TensorRT_ROOT)
    set(TensorRT_ROOT "${TensorRT_DIR}")
endif()

# Search paths: TENSORRT_ROOT first, then system (order must match trt_onnx_builder for same TRT)
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

# Find NvInfer.h (runtime API)
find_path(TensorRT_INCLUDE_DIR
    NAMES NvInfer.h NvInferRuntime.h
    PATHS ${_TensorRT_SEARCH_PATHS}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
)
if(NOT TensorRT_INCLUDE_DIR)
    find_path(TensorRT_INCLUDE_DIR
        NAMES NvInfer.h NvInferRuntime.h
        PATH_SUFFIXES include
    )
endif()

# Find nvinfer library (runtime only; no nvonnxparser needed for .engine)
find_library(TensorRT_LIBRARY
    NAMES nvinfer
    PATHS ${_TensorRT_SEARCH_PATHS}
    PATH_SUFFIXES lib lib/x86_64-linux-gnu lib/aarch64-linux-gnu
    NO_DEFAULT_PATH
)
if(NOT TensorRT_LIBRARY)
    find_library(TensorRT_LIBRARY NAMES nvinfer PATH_SUFFIXES lib lib/x86_64-linux-gnu lib/aarch64-linux-gnu)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TensorRT
    REQUIRED_VARS TensorRT_LIBRARY TensorRT_INCLUDE_DIR
    FAIL_MESSAGE "TensorRT (nvinfer) not found. Install libnvinfer-dev or set TENSORRT_ROOT."
)

if(TensorRT_FOUND)
    set(TensorRT_INCLUDE_DIRS "${TensorRT_INCLUDE_DIR}")
    set(TensorRT_LIBRARIES "${TensorRT_LIBRARY}")

    if(NOT TARGET TensorRT::nvinfer)
        add_library(TensorRT::nvinfer UNKNOWN IMPORTED)
        set_target_properties(TensorRT::nvinfer PROPERTIES
            IMPORTED_LOCATION "${TensorRT_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIR}"
        )
    endif()
endif()
