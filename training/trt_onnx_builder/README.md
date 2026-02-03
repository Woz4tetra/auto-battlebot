# trt_onnx_builder

Python extension (nanobind) that builds a TensorRT engine from an ONNX file using the TensorRT C++ API and writes the serialized plan to disk. Engines produced this way are compatible with C++ runtimes that use `IRuntime::deserializeCudaEngine()` (e.g. same plan format as typical C++ inference apps).

## Requirements

- CMake ≥ 3.15
- C++17 compiler
- TensorRT with **nvinfer** and **nvonnxparser** (e.g. `libnvinfer-dev` on Ubuntu)
- Python 3.x with **development headers** (e.g. `python3-dev` on Ubuntu)
- CUDA toolkit (TensorRT build uses it)

## Build

From the **repository root**:

```bash
cd training/yolo/trt_onnx_builder
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

The built extension is `trt_onnx_builder*.so` in the build directory (e.g. `trt_onnx_builder.cpython-312-x86_64-linux-gnu.so`).

If the build fails with `Python.h: No such file or directory`, install Python development headers (e.g. `sudo apt install python3-dev` on Ubuntu).

## Use from Python

`convert_to_tensorrt.py` automatically looks for the extension in `training/yolo/trt_onnx_builder/build`. After building, run from the repo root:

```bash
python training/yolo/convert_to_tensorrt.py models/your_model.pt -o models/your_model.engine
```

To use the extension from another directory, add the build dir to `PYTHONPATH`:

```bash
export PYTHONPATH="/path/to/auto-battlebot/training/yolo/trt_onnx_builder/build:$PYTHONPATH"
python -c "import trt_onnx_builder; trt_onnx_builder.build_engine_from_onnx('model.onnx', 'model.engine')"
```

`convert_to_tensorrt.py` will use this extension when available; if the module is not found, it falls back to the Python TensorRT API and prints instructions to build this project.

## API

- **`build_engine_from_onnx(onnx_path, engine_path, fp16=True, workspace_gib=16)`**  
  Builds a TensorRT engine from the ONNX file at `onnx_path` and writes it to `engine_path`.  
  `fp16`: use FP16 when supported.  
  `workspace_gib`: builder workspace size in GiB.
