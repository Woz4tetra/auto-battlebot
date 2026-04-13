FROM nvidia/cuda:13.2.0-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PYOPENGL_PLATFORM=egl \
    HF_HOME=/opt/hf

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10 python3.10-venv python3-pip \
    ca-certificates curl git git-lfs \
    libgl1 libegl1 libglib2.0-0 libsm6 libxext6 libxrender1 libxi6 \
    libxrandr2 libxinerama1 libxxf86vm1 libxkbcommon0 libgomp1 \
    && rm -rf /var/lib/apt/lists/*

RUN python3.10 -m venv /opt/venv
ENV PATH="/opt/venv/bin:${PATH}"

RUN pip install --upgrade pip setuptools wheel

WORKDIR /workspace/training/synthetic
COPY . /workspace/training/synthetic

RUN pip install --no-build-isolation -e /workspace/training/synthetic
RUN pip install gitpython scikit-learn pyyaml scikit-image trimesh

# BlenderProc's pinned 4.10.0.82 wheel is unavailable in this environment.
# Preinstall a compatible wheel into Blender's Python runtime.
RUN blenderproc pip install opencv-contrib-python==4.10.0.84
RUN blenderproc pip install gitpython scikit-learn "pyglet<2" pyrender==0.1.45

COPY docker/entrypoint.sh /usr/local/bin/synthetic-entrypoint
RUN chmod +x /usr/local/bin/synthetic-entrypoint

ENTRYPOINT ["/usr/local/bin/synthetic-entrypoint"]
CMD ["/bin/bash"]
