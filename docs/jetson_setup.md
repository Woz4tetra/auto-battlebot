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
