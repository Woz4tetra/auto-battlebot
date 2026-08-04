# Docker playback on x86

Runs SVO playback on an x86_64 desktop without installing ZED SDK, CUDA, or TensorRT on
the host. Playback only. Deployment still targets the Jetson, see
[jetson_setup.md](jetson_setup.md).

## Design

The image holds the toolchain. Source, build tree, and data stay on the host and are
bind-mounted at runtime. Nothing under `src/`, `include/`, `config/`, or `data/` is ever
copied into the image, so editing code never triggers a rebuild.

`docker/docker-compose.playback.yml` owns the container configuration. The scripts in
`scripts/docker/` are thin wrappers that export the variables it interpolates and call
`docker compose`. It pulls in `docker-compose.ros-connector.yml` with `include`, so
`run_playback.sh` starts the Foxglove bridge as a `depends_on` dependency, and both
services carry a `build:` stanza so a fresh machine builds them on first use. Both files
declare `name: auto-battlebot`; they have to agree, or the fixed `container_name`
collides across compose projects.

Running `docker compose -f docker/docker-compose.playback.yml ...` by hand fails on the
empty `${REPO}` and `${HOST_UID}` interpolations. Go through the scripts, or source
`scripts/docker/docker_common.sh` first, which also gives you a `compose` shell
function.

Rebuild the image only when one of these changes:

- `install/base_packages.txt`
- `install/ubuntu_24_packages.txt`
- `.llvm-version`
- the `TENSORRT_VERSION` pin or base image in `docker/playback.Dockerfile`

The `FetchContent` dependency tree (miniroscpp, tomlplusplus, CLI11, magic_enum, lvgl,
mcap, spdlog) lands in `build-docker/_deps` on the host, so it survives image rebuilds.

## Host requirements

- x86_64 with an NVIDIA GPU, compute capability 8.6 or 8.9 to match the prebuilt engines
- NVIDIA driver, but no CUDA or ZED SDK install
- Docker with the NVIDIA Container Toolkit, so `--gpus all` works:

```bash
./install/install_docker_ubuntu.sh
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify with `docker info | grep nvidia`, which should list `nvidia` under Runtimes.

## First run

```bash
./scripts/docker/build_image.sh
./scripts/docker/run_playback.sh
```

Budget an hour and about 25 GB of disk for the first build. The image is 21 GB: the
Stereolabs CUDA 13 devel base is most of it, plus roughly 2 GB of TensorRT and 3 GB of
apt packages. `libpcl-dev` alone pulls 29 hard dependencies including `libvtk9-qt-dev`
and `libvtk9-dev`, which bring in the Qt5 QtQuick stack and OpenJDK 21, even though
`CMakeLists.txt` only asks for the PCL `common` and `sample_consensus` components.
Ubuntu ships PCL as a single `libpcl-dev` package, so this cannot be trimmed with apt.

That cost is paid once. Editing code afterwards rebuilds nothing.

`run_playback.sh` builds either image if it is missing, starts the ros-connector
container, compiles into `build-docker/`, and replays the SVO named in
`config/playback/_playback.toml`. Pass arguments through to the binary:

```bash
./scripts/docker/run_playback.sh -c config/playback/mr_stabs_mk2_playback.toml
```

For an interactive shell with the same mounts:

```bash
./scripts/docker/shell.sh
./scripts/docker/shell.sh ./scripts/build.sh
```

The first launch spends several minutes GPU-optimizing the ZED neural depth model and
needs network access. It looks like a hang but is not. The result is cached in the
`auto-battlebot-zed-resources` volume, so later runs skip it.

## Getting the data

`data/` is gitignored and roughly 46 GB, so the container cannot supply it. On a new
machine:

- **Model engines**: `source ./scripts/activate_python.sh && python scripts/sync_models.py`.
  This needs the Python environment from `./scripts/setup_python.sh`, which is a host
  concern. The playback container has no Python.
- **SVO recordings**: copy `data/svo/` from an existing machine. Point
  `svo_file_path` in `config/playback/_playback.toml` at whichever one you have.

## Why TensorRT is pinned

`docker/playback.Dockerfile` pins TensorRT to `10.14.1.48-1+cuda13.0` and marks it held.
The engines in `data/models` are serialized by a specific TensorRT version, and
`src/tensorrt_inference/trt_engine.cpp` only calls `deserializeCudaEngine`, never
rebuilding from ONNX. A mismatched TensorRT fails at engine load. If you bump the pin,
regenerate the engines to match.

Pin all four packages, not just `libnvinfer-dev`. That package depends on
`libnvinfer10` at the same version, but apt resolves `libnvinfer10` independently to the
newest in the repo and then reports `held broken packages`. NVIDIA's repo carries
roughly 28 versions at a time and rotates old ones out. List what is currently available
with:

```bash
docker run --rm stereolabs/zed:5.1-devel-cuda13.0-ubuntu24.04 \
    bash -c 'apt-get update -qq 2>/dev/null; apt-cache madison libnvinfer-dev'
```

The engines are also GPU-architecture specific. `_x86_64_sm89.engine` runs on Ada
(RTX 40 series), `_x86_64_sm86.engine` on Ampere. `sync_models.py` selects by
architecture.

## Build directory separation

The container builds into `build-docker/`, the host into `build/`. They cannot share
one: `CMakeCache.txt` records absolute compiler and dependency paths that differ between
the two environments. `install/build_cpp_project.sh` honors `AUTO_BATTLEBOT_BUILD_DIR`,
which `scripts/docker/docker_common.sh` sets.

The repo is mounted at its host path inside the container so `compile_commands.json`,
clangd, and stack traces resolve on both sides.

## File ownership

The container runs as the invoking user via `--user "$(id -u):$(id -g)"`, with
`/etc/passwd` and `/etc/group` mounted read-only so that UID resolves to a name. Files
written into `build-docker/` are owned by you, not root.

## Troubleshooting

**No window appears.** The UI is SDL2, so it needs X11. Check `DISPLAY` is set and
`$XAUTHORITY` exists. Under Wayland this goes through XWayland. To skip the UI, set
`ui.enable = false` in the config and use Foxglove instead.

**`could not select device driver "" with capabilities: [[gpu]]`.** The NVIDIA Container
Toolkit is missing or Docker was not restarted after `nvidia-ctk runtime configure`.

**Engine fails to deserialize.** The TensorRT pin and the engines disagree. Either
rebuild the engines or check that `sync_models.py` pulled ones matching your GPU
architecture.

**`docker build` uploads gigabytes.** `.dockerignore` is missing or was edited. It must
exclude `data/`, `venv/`, and the build directories.

**`CORRUPTED SDK INSTALLATION` or `NEURAL CORRUPTED MODEL` from the ZED SDK.** The
`auto-battlebot-zed-resources` volume is stale. Docker seeds a named volume from the
image only when the volume does not yet exist, and never re-seeds it afterwards, so a
volume created by an older image keeps that image's file contents and permissions
forever. Drop it and let it re-seed:

```bash
docker volume rm auto-battlebot-zed-resources auto-battlebot-zed-settings
```

Do this after any Dockerfile change that touches `/usr/local/zed`. The same applies to
`auto-battlebot-home`.

**`CALIBRATION FILE NOT AVAILABLE`.** The SDK downloads a per-serial `SNxxxxxxxx.conf`
into `/usr/local/zed/settings` the first time it opens a given SVO, which needs both
network access and a writable directory. If the machine is offline, copy the file from a
working install into the `auto-battlebot-zed-settings` volume.
