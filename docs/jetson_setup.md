# Host dependencies

This guide assumes you're using a linux machine to run all these commands. You'll have to adapt these instructions if you're on another OS.

I recommend running all of these commands inside of a tmux session in case of network dropouts:

-   Create tmux session: `tmux new -s build`
-   Hide tmux session: ctrl-B D
-   Reattach tmux session: `tmux a -t build`

## Jetson initial setup

[Follow this guide from NVidia](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

Setup options:

-   username: ben
    **If you use a different name, make sure to change all the commands below accordingly**

After rebooting, try ssh: `ssh ben@ubuntu.local` <br>
Install openssh server if it doesn’t work: `sudo apt-get install openssh-server -y`

## Apt refresh + basic packages

-   `sudo apt update`
-   `sudo apt upgrade`
-   `sudo apt autoremove`
-   `sudo apt install nano tmux curl htop`
-   `sudo reboot`

## SSH setup

Generate an SSH key

`ssh-keygen`

Press enter for all default options

-   `cd ~/.ssh/`
-   `cp id_rsa.pub authorized_keys`
-   On your machine:
    -   `scp ben@ubuntu:/home/ben/.ssh/id_rsa ~/.ssh/auto_battlebot_id_rsa`
    -   `scp ben@ubuntu:/home/ben/.ssh/id_rsa.pub ~/.ssh/auto_battlebot_id_rsa.pub`
-   Optionally disable password login:
    -   `sudo nano /etc/ssh/sshd_config`
    -   Search for `#PasswordAuthentication yes`
    -   Change to `PasswordAuthentication no`
    -   Save and exit (ctrl-S ctrl-X)
-   `sudo service ssh restart`
-   Find the Jetson's IP with `ifconfig`
-   Try to login with `ssh -i ~/.ssh/auto_battlebot_id_rsa ben@ubuntu`

## Disable wifi power saving

[Based on this guide](https://unix.stackexchange.com/questions/269661/how-to-turn-off-wireless-power-management-permanently)

-   `sudo nano /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf`
-   Change:

```
[connection]
wifi.powersave = 3
```

to

```
[connection]
wifi.powersave = 2
```

-   `sudo systemctl restart network` This will kill your SSH session. Alternatively, you can run `sudo reboot`
-   Have a display and keyboard on hand in case this goes wrong.

## Disable desktop

This reduces boot times by ~15 seconds.

https://forums.developer.nvidia.com/t/how-to-boot-jetson-nano-in-text-mode/73636

-   `sudo systemctl set-default multi-user.target`

To re-enable the desktop:

-   `sudo systemctl set-default graphical.target`

To start the desktop manually after logging into the CLI:

-   `sudo systemctl start gdm3.service`

## Add to sudo group

-   `sudo usermod -aG sudo $USER`
-   `sudo visudo`
-   If vim opens,
    -   Press a to enter edit mode
    -   Add the following line to the file: <br>`ben  ALL=(ALL) NOPASSWD:ALL`
    -   Press esc
    -   Type `:x`
    -   Press enter
-   If nano opens,
    -   Add the following line to the file: <br>`ben  ALL=(ALL) NOPASSWD:ALL`
    -   Press ctrl-S then ctrl-X
-   Log out with ctrl-D
-   Log in and try `sudo su`
-   If no password prompt appears, these steps worked

## Engine build notes

### `tensorrt_build.py`

#### TensorRT timing cache

`save_timing_cache` in `tensorrt_build.py` is last writer wins. Concurrent builders may race
there; the cache is advisory, and a lost update only costs the next build some re-timing.

## Increase memory security limit

This increases the maximum RAM a process from a given user can use.

-   `sudo nano /etc/security/limits.conf`
-   File should look like this (increase all limits to 8GB):
    ```
    ben hard stack 8192
    ben soft stack 8192
    ubuntu hard stack 8192
    ubuntu soft stack 8192
    root hard stack 8192
    root soft stack 8192
    ```

## Engine selection

A TensorRT engine is compiled ahead of time against one GPU architecture and one TensorRT version, and is not portable across either. Deserializing a mismatched engine fails with "the engine plan file is not compatible with this version of TensorRT", which names the version even when the real mismatch is compute capability, and says nothing about what the machine is or what was tried. The engine selector therefore asks TensorRT rather than parsing the sm tag out of the filename: the first candidate that deserializes on this device is the correct one. Filenames stay purely descriptive. One instance per model that loads engines; the model factories construct them.

## Power telemetry

Jetson power telemetry reads the INA3221 VDD_IN rail and the soctherm over-current event counters. Paths are discovered once by hwmon name because indices shift across boots; empty means unavailable.

## Runtime state location

The Wi-Fi access state file is `$HOME/.local/state/auto_battlebot/wifi_access`. It lives under `$HOME`, not `config/`, because a deploy overwrites the repo tree.

## Video encoder

- The H.264 preview uses NVENC where the box has it and libx264 at ultrafast on one thread where it does not (the Orin Nano). It is created when either can open.
- The nvv4l2h264enc backend returns nullptr when an element is missing or the pipeline does not reach PLAYING, which is where nvv4l2h264enc opens the encoder device.

## Pinned frame memory

`PinnedMatAllocator` backs a `cv::Mat` with pinned, device-mapped host memory so `GpuLetterbox` reads a frame in place instead of copying it to the GPU. Set it on the destination Mat before the conversion that fills it (`frame.allocator = PinnedMatAllocator::instance();`). Pinning is slow because it locks pages, so released buffers go back to a pool keyed by size and the next frame reuses one. On Orin, pinned memory stays CPU-cached and I/O coherent, so the CPU writes into it at full speed. With no CUDA device, or when a pinned allocation fails, it falls back to OpenCV's standard allocator and the frame takes the copy path. The pool keeps 16 free buffers per size: a frame lives about ten ticks between the pipeline and the publisher queue, so this covers steady state without pinning memory forever.

GPU letterbox: a frame in pinned host memory is read in place. The Orin's GPU and CPU share DRAM, so the kernel fetches only the source pixels it samples. Anything else (a playback frame, a camera without the pinned allocator) is copied to a device staging buffer first.

## Notes moved from `src/` comments (2026-11-15)

Each entry names the file and the code it described.

### `src/health/health_logger.cpp`

Beside `for (int channel = 1; channel <= 7; ++channel) {`:

Find the channel labeled VDD_IN (module input rail); the channel number is board-specific.

### `src/health/health_logger.cpp`

Beside `bool have_all_counts = true;`:

OC1 = under-voltage, OC2 = VDD_IN average power over limit, OC3 = VDD_IN instant power over peak. Each event throttles clocks, so a nonzero delta means the supply hit a hardware limit since the last sample. The INA3221 samples too slowly to see these transients; the latched counters are the only reliable signal.

### `src/main.cpp`

Beside `std::string read_build_version() {`:

The build version stamped next to the executable by the generate_git_version target (see cmake/git_version.cmake). Kept out of the binary so a new commit only rewrites the text file instead of forcing a recompile and relink.

### `src/rgbd_camera/gst_h264_backend.cpp`

Beside `#include <gst/app/gstappsink.h>`:

H.264 through GStreamer. nvv4l2h264enc is the Jetson's hardware encoder, which FFmpeg cannot reach on stock L4T. x264enc is the software encoder for the dashboard preview on a Jetson with no NVENC (the Orin Nano), where NVIDIA's FFmpeg build has no libx264. Built only when pkg-config finds gstreamer-app-1.0 (AUTO_BATTLEBOT_HAVE_GSTREAMER).

## Deploying with deploy_to_jetson.sh

`scripts/deploy_to_jetson.sh --help` lists the options and the `JETSON_*` environment variables.

### What gets synced

- Code: `rsync --delete` with a `--filter` that reads each `.gitignore` it meets during traversal, so every gitignored path is excluded. `/data` is excluded by the root `.gitignore`, so `--delete` never touches it on the remote. `.git/` is excluded explicitly.
- `.deployignore` adds excludes on top of that.
- `data/models/` is gitignored, so it is not in the code pass. The script syncs it in a second rsync.
- The Jetson has no git repo (`.git` is excluded), so the script stamps the version into `.build_version` for the build to read as a fallback. It uses the same rule as `cmake/git_version.cmake`: short hash, plus `-dirty` for tracked modifications.

## Build and install facts

### Build version

`cmake/git_version.cmake` writes the short commit hash to a text file next to the binary, suffixed with `-dirty` when the working tree has uncommitted changes to tracked files. It runs via `cmake -P` from a custom target, so it re-evaluates on every build and the dirty flag stays accurate. The binary reads the file at startup, so nothing recompiles or relinks when the hash changes.

On machines with a git repo (dev hosts) the version comes from git directly. On machines without one (the Jetson, where `deploy_to_jetson.sh` excludes `.git`) it falls back to the `.build_version` file stamped at deploy time. Inputs, passed with `-D`: `GIT_SOURCE_DIR` (repository working directory) and `GIT_OUTPUT` (path of the text file to write).

### `.deployignore`

`.deployignore` lists paths that stay off the Jetson even though they are tracked in git. Its syntax matches `.gitignore`, including `!pattern` to re-include. The rules become an rsync filter file applied before the `.gitignore` merge, so a `!pattern` there can also pull back a gitignored path.

### Camera profile backup

The dashboard saves camera profiles on the box, outside the repo (`CameraProfileStore::kDefaultPath`). Each deploy copies them back to `data/camera_profiles/<hostname>.toml` so they live in git. `data/` is gitignored, so a new backup is force-added (staged, not committed). A box with no profiles yet is offered one of the existing backups.

### SSH keys and MaxAuthTries

With a key, ssh offers only that key. Without it, the agent offers every key it holds, and sshd disconnects after `MaxAuthTries` (6) before reaching a password prompt. `scripts/deploy_to_jetson.sh` and `scripts/download_recordings.sh` both default the key to `~/.ssh/<host>` for that reason.

### Persisted UI state

The service keeps UI state (`selected_profile`, `wifi_access`) in `~/.local/state/auto_battlebot`. `service/build_and_install_service.sh` creates the directory as the service user. It must exist before first boot so you can hand-write `selected_profile` when the default profile's camera is absent and the UI never comes up.

### GStreamer

`CMakeLists.txt` finds GStreamer because it reaches the Jetson's hardware encoder (`nvv4l2h264enc`), which stock L4T FFmpeg cannot: its `h264_nvenc` needs `libnvidia-encode`, which no Jetson has. It is optional, so the desktop builds without the dev packages. A Jetson recording video needs it (`install/jetson_packages.txt`).

### CUDA architectures

`CMAKE_CUDA_ARCHITECTURES` is set before `enable_language`, which otherwise probes a default that CUDA 13 no longer supports. Orin is sm87, the dev laptop is sm89, and megamind's A6000s are sm86.

### L4T floor

`install/check_jetson_orin.sh` checks the L4T major. The install path branches on it for the PyTorch wheel source and expects the Python version JetPack ships. Without the check, a JetPack older than 6 is accepted and fails much later, at wheel install or engine load.

### jetson_clocks

`install/install_jetson_clocks.sh` installs a systemd oneshot unit that runs `jetson_clocks` at boot, lifting DVFS gating so the CPU, GPU, and EMC stay at their maximum frequency. `RemainAfterExit` keeps the unit active after the script returns, so the kept settings are clearly attributed to this service.

### Masked user services

`install/install_jetson_stability.sh` masks user-level GNOME and GVFS services that a kiosk does not need and that contribute to background cgroup churn.

### Python environment

A venv built against a different Python minor cannot be reused. Its `lib/pythonX.Y/` tree, the TensorRT `.pth`, and the CUDA block appended to `bin/activate` all encode the old version, and the append is guarded by a grep that will not rewrite it. A JetPack 6 to 7 upgrade in place hits this, where `install_jetson.sh` passes `--no-recreate`, so `install/install_python_environment.sh` recreates the venv when the minor differs.

The venv `activate` script gets a `LD_LIBRARY_PATH` block because the host sets the CUDA and cuDNN paths in `LD_LIBRARY_PATH` and the venv does not. Without it, `torch.cuda.is_available()` is false in the venv.

### PyTorch

`install/install_pytorch_jetson.sh` follows https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html. JetPack must be installed. Run `scripts/setup_python.sh` first to create the project venv. If the venv does not exist, the script creates it and installs only PyTorch, and you then run `setup_python.sh` for the rest of the project.

- JetPack 7 (L4T 39) and newer: NVIDIA publishes no Jetson wheel. There is no `compute/redist/jp/v7x` path and no jp7 index on pypi.jetson-ai-lab.io, and NVIDIA's own guidance for Orin on JetPack 7 is the upstream CUDA index. Those wheels are built for sm_80. A cubin built for sm_8x runs on any sm_8y with y >= x, so Orin's sm_87 executes them without JIT. PyTorch still prints a compute-capability warning naming sm_90, sm_100, and sm_120. It is cosmetic, not a failure.
- The index is derived from the CUDA version (13.2 gives `cu132`) rather than tabled, so a 13.0 or 13.3 box picks its own.
- The install uses `--index-url`, not `--extra-index-url`. PyPI's aarch64 torch is a CPU-only build and wins resolution otherwise, leaving `torch.cuda.is_available()` false.
- cusparselt (for PyTorch 24.06+) installs only for the NVIDIA JetPack 6 wheels. The upstream CUDA-index wheels used on JetPack 7 vendor their own CUDA dependencies.
