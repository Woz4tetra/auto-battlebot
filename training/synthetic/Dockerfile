FROM nvidia/cuda:13.0.0-cudnn-devel-ubuntu22.04

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

RUN pip install --default-timeout=120 --retries=20 --no-build-isolation -e /workspace/training/synthetic
RUN pip install --default-timeout=120 --retries=20 gitpython scikit-learn pyyaml scikit-image trimesh

# Ensure Blender is downloaded and available in the image.
RUN blenderproc pip install --default-timeout=120 --retries=20 opencv-contrib-python==4.10.0.82

# Preinstall Blender runtime packages that BlenderProc otherwise installs on first run.
RUN /root/blender/blender-4.2.1-linux-x64/4.2/python/bin/python3.11 -m pip install \
    --default-timeout=120 \
    --retries=20 \
    --upgrade pip setuptools wheel && \
    /root/blender/blender-4.2.1-linux-x64/4.2/python/bin/python3.11 -m pip install \
    --default-timeout=120 \
    --retries=20 \
    numpy==1.26.4 \
    pyyaml==6.0.1 \
    imageio==2.34.1 \
    gitpython==3.1.43 \
    scikit-image==0.23.2 \
    pypng==0.20220715.0 \
    scipy==1.13.1 \
    matplotlib==3.9.0 \
    pytz==2024.1 \
    h5py==3.11.0 \
    pillow==10.3.0 \
    opencv-contrib-python==4.10.0.82 \
    scikit-learn==1.5.0 \
    python-dateutil==2.9.0.post0 \
    rich==13.7.1 \
    trimesh==4.4.0 \
    "pyglet<2" \
    pyrender==0.1.45

COPY docker/entrypoint.sh /usr/local/bin/synthetic-entrypoint
RUN chmod +x /usr/local/bin/synthetic-entrypoint

ENTRYPOINT ["/usr/local/bin/synthetic-entrypoint"]
CMD ["/bin/bash"]
