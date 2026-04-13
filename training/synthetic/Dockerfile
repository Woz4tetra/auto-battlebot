FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PYOPENGL_PLATFORM=egl \
    TORCH_CUDA_ARCH_LIST=8.9 \
    SF3D_USE_CPU=0 \
    HF_HOME=/opt/hf

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10 python3.10-venv python3-pip python3-dev \
    build-essential cmake ninja-build pkg-config git git-lfs curl ca-certificates \
    ffmpeg \
    libgl1 libegl1 libglib2.0-0 libsm6 libxext6 libxrender1 libxi6 \
    libxrandr2 libxinerama1 libxxf86vm1 libomp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python3.10 -m venv /opt/venv
ENV PATH="/opt/venv/bin:${PATH}"

RUN pip install --upgrade pip==24.3.1 setuptools wheel

# Install torch before building local SF3D extensions that import torch in setup.py.
RUN pip install --index-url https://download.pytorch.org/whl/cu124 \
    torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1

RUN pip install --index-url https://download.pytorch.org/whl/cu124 \
    xformers==0.0.28.post3

WORKDIR /opt/sf3d
RUN git clone --depth=1 https://github.com/Stability-AI/stable-fast-3d.git .

# Build SF3D with no build isolation so torch from the active venv is visible.
RUN python -c "import torch; print('torch', torch.__version__)" && \
    pip install gpytoolbox==0.2.0 antlr4-python3-runtime==4.9.3 && \
    USE_CUDA=1 pip install --no-build-isolation ./texture_baker && \
    pip install --no-build-isolation ./uv_unwrapper && \
    bash -lc "grep -vE '^(\\./texture_baker/|\\./uv_unwrapper/)$' requirements.txt > /tmp/sf3d-req.txt" && \
    pip install --no-build-isolation -r /tmp/sf3d-req.txt

WORKDIR /workspace/training/synthetic
COPY . /workspace/training/synthetic

RUN pip install -e /workspace/training/synthetic[reference3d]

# BlenderProc's pinned 4.10.0.82 wheel is unavailable in this environment.
# Preinstall a compatible wheel into Blender's Python runtime.
RUN blenderproc pip install opencv-contrib-python==4.10.0.84

# Install TripoSR in an isolated venv to avoid dependency conflicts with SF3D.
RUN python3.10 -m venv /opt/triposr-venv
RUN /opt/triposr-venv/bin/python -m pip install --upgrade \
    --default-timeout=120 \
    --retries 20 \
    pip==24.3.1 setuptools wheel
RUN /opt/triposr-venv/bin/pip install --index-url https://download.pytorch.org/whl/cu124 \
    torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1

WORKDIR /opt/triposr
RUN git clone --depth=1 https://github.com/VAST-AI-Research/TripoSR.git .
RUN /opt/triposr-venv/bin/pip install pybind11 scikit-build-core
RUN bash -lc "grep -v 'torchmcubes' /opt/triposr/requirements.txt > /tmp/triposr-req.txt" && \
    /opt/triposr-venv/bin/pip install -r /tmp/triposr-req.txt
RUN /opt/triposr-venv/bin/pip install --no-build-isolation git+https://github.com/tatsy/torchmcubes.git
RUN /opt/triposr-venv/bin/pip install --default-timeout=120 --retries 20 onnxruntime

# Provide a CLI compatible with generate_reference_models.py backend-cmd usage.
RUN cat <<'EOF' >/usr/local/bin/triposr
#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -eq 0 ] || [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  cat <<'USAGE'
Usage:
  triposr --input <image.png> --output <mesh.glb> [extra run.py args...]

Notes:
  - Writes to the exact --output mesh path.
  - Passes through extra args to TripoSR run.py.
USAGE
  exit 0
fi

input_path=""
output_path=""
extra_args=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --input)
      input_path="$2"
      shift 2
      ;;
    --output)
      output_path="$2"
      shift 2
      ;;
    *)
      extra_args+=("$1")
      shift
      ;;
  esac
done

if [ -z "$input_path" ] || [ -z "$output_path" ]; then
  echo "triposr wrapper: --input and --output are required" >&2
  exit 2
fi

out_dir="$(mktemp -d /tmp/triposr_out.XXXXXX)"
/opt/triposr-venv/bin/python /opt/triposr/run.py "$input_path" --output-dir "$out_dir" "${extra_args[@]}"

mesh_file="$(find "$out_dir" -type f \( -name '*.glb' -o -name '*.obj' -o -name '*.ply' -o -name '*.stl' \) | head -n 1 || true)"
if [ -z "$mesh_file" ] || [ ! -f "$mesh_file" ]; then
  echo "triposr wrapper: no output mesh produced in $out_dir" >&2
  exit 3
fi

mkdir -p "$(dirname "$output_path")"
if [[ "${output_path,,}" == *.glb ]] && [[ "${mesh_file,,}" != *.glb ]]; then
  /opt/triposr-venv/bin/python - "$mesh_file" "$output_path" <<'PY'
import sys
import trimesh

src = sys.argv[1]
dst = sys.argv[2]
mesh = trimesh.load(src, force="mesh")
if not isinstance(mesh, trimesh.Trimesh):
    raise SystemExit(f"Failed to load mesh from {src}")
mesh.export(dst)
PY
else
  cp "$mesh_file" "$output_path"
fi
EOF
RUN chmod +x /usr/local/bin/triposr

RUN mkdir -p /workspace/training/synthetic/docker /workspace/training/synthetic/scripts
COPY docker/entrypoint.sh /usr/local/bin/synthetic-entrypoint
RUN chmod +x /usr/local/bin/synthetic-entrypoint

ENTRYPOINT ["/usr/local/bin/synthetic-entrypoint"]
CMD ["/bin/bash"]
