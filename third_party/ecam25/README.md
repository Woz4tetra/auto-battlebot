# e-CAM25_CUONX driver, ported to JetPack 7

Vendor kernel driver for the e-con Systems e-CAM25_CUONX (AR0234 global shutter MIPI CSI
camera), patched to build on JetPack 7.2.x (L4T 39.2.x, kernel 6.8). e-con ships no JetPack 7
package; their latest targets JetPack 6.1 only.

Built and installed by `scripts/install_jetson.sh`, which skips the step once the module is
installed for the running kernel and the overlay is registered. To run it by hand:
`source install/install_ecam25_camera.sh && install_ecam25_camera`, which calls
`scripts/build_ecam25_driver.sh` for you. Hardware setup, the verification ladder and failure
signatures are in `docs/ecam25_camera_setup.md`.

## Provenance

| | |
| --- | --- |
| Package | `e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04.tar.gz` |
| md5 | `1c1364eb5ae228d3a3b5b58086b53d08` |
| Release | R04, 29-MAR-2025, targets L4T 36.4.0 / JetPack 6.1.0 / kernel 5.15.148-tegra |
| Local copy | `~/Documents/e-CAM25` (not in the repo) |
| License | GPL-2.0, as declared in `src/ar0234.c` |

`src/` is the vendor source exactly as `Kernel/*_module.patch` produces it, with no edits.
`dts/` is the vendor 4-lane overlay from `Kernel/*_dtb.patch` **with** the JetPack 7 edits
applied in place, since it is a file we have to maintain anyway.

## Layout

| Path | Holds |
| --- | --- |
| `src/` | Pristine vendor driver: `ar0234.c`, `ar0234.h`, `mcu_firmware.h`, the MCU firmware blob, the vendor Makefile |
| `patches/` | JetPack 7 changes, applied in order by the build script |
| `patches/optional/` | e-con's original nvidia-oot patch, unported. Nothing here is applied unless a check below forces it |
| `dts/` | The 4-lane device tree overlay |

## What the patches change

1. **i2c probe and remove signatures.** Kernel 6.6 dropped the `i2c_device_id` argument from
   `i2c_driver.probe`; 6.1 made `.remove` return void.
2. **Frame interval ops move to pad ops.** Kernel 6.8 removed `g_frame_interval` and
   `s_frame_interval` from `v4l2_subdev_video_ops` and added `get_frame_interval` and
   `set_frame_interval` to `v4l2_subdev_pad_ops`, each taking a `v4l2_subdev_state`.
   `VIDIOC_G/S_PARM` reaches them through `v4l2_g_parm_cap`/`v4l2_s_parm_cap`, so GStreamer's
   `framerate=70/1` still negotiates.
3. **The i2c driver is renamed to `ecam25_ar0234`.** JetPack 7 ships its own `nv_ar0234.ko` for
   the Leopard Hawk module, and two i2c drivers with one name collide in sysfs. The device tree
   compatible stays `nvidia,ar0234`. Because the v4l2 subdev name follows the driver name, the
   subdev becomes `ecam25_ar0234 <bus>-0042`, which is why the overlay's `devname` says so too.
4. **A JetPack 7 external-module Makefile.** The vendor one includes
   `$(srctree)/../nvidia/include`, a JetPack 5 era path. On JetPack 7 the tegracam and
   `camera_common` headers come from the `nvidia-l4t-kernel-oot-headers` package at
   `/usr/src/nvidia/nvidia-public`.
5. **`gpio_cansleep` is gone in 6.8.** Both branches of `toggle_gpio` did the same thing apart
   from the cansleep variant of the set. The reset and boot lines hang off the PCA6408 expander
   on the camera i2c bus, so driving them sleeps, and every caller is in process context.

## The conftest trap

`camera_common.h` includes `<nvidia/conftest.h>` and gates a member of
`struct camera_common_data` on `NV_TEGRA_PMC_IO_PAD_POWER_ENABLE_PRESENT`:

```c
#if defined(NV_TEGRA_PMC_IO_PAD_POWER_ENABLE_PRESENT) /* Linux v7.0 */
	struct tegra_pmc			*pmc;
#endif
	void	*priv;
```

No header package ships that `conftest.h`. The shipped `tegra-camera.ko` has
`tegra_pmc_io_pad_power_enable` as an undefined symbol, so it was built with the member
present. Build the sensor driver without the macro and the struct is 8 bytes short: the driver
writes `s_data->priv`, `tegra-camera` reads a different offset. It compiles, links and loads
cleanly, then corrupts memory on the first capture.

`scripts/build_ecam25_driver.sh` probes the kernel headers for that function, generates the
header, and **fails the build** if it cannot confirm the macro. Do not work around that gate.

The comment says "Linux v7.0", which reads like it should be off on 6.8. It is on: NVIDIA
backported the function into the Canonical noble tree (`include/soc/tegra/pmc.h`).

## The nvidia-oot patches we deliberately dropped

The vendor `oot.patch` modifies NVIDIA's own modules, which is why the JetPack 6 install is
pinned to kernel 5.15.148 forever. None of it is needed to start:

| Vendor hunk | Why it is dropped |
| --- | --- |
| `tegracam_ctrls.c` power-state skip | This driver registers its own `v4l2_ctrl_handler` and never touches the tegracam control framework |
| `channel.c` `vidioc_s_parm`/`g_parm` | Already upstream in 39.2. The shipped `tegra-camera.ko` references `v4l2_g_parm_cap`/`v4l2_s_parm_cap`; JetPack 6.1's does not |
| `vi5_fops.c` stream error monitor | Exports four symbols this driver never references. It was a hook for a userspace recovery daemon that is not in the package |
| `channel.c` forcing `*bytesperline = bpl` | Only needed if the stride check fails (see the L5 gate in the setup doc) |
| `vi5_formats.h` dropping `UYVY8_2X8` | Only needed if format enumeration or capture comes out wrong |
| `capture-ivc.c` per-channel semaphore | Guards a race that needs concurrent channel-id acquisition. One camera on CAM1 will not hit it |

If one of the last three turns out to be needed, extract that hunk into `patches/optional/`,
rebuild the nvidia-oot modules from synced 39.2.1 sources, and record which check forced it.

## Known warnings

The build prints pre-existing vendor warnings: about ten `-Wmissing-prototypes`, three
`-Waddress` (`!(buf + len)` and `(&priv->streamdb[loop]) != NULL` are always true), and one
`-Wframe-larger-than=` for the 2064 byte stack frame in `mcu_bload_update_fw`. The kernel is
built with `CONFIG_WERROR` off, so none of them fail the build. They are vendor code and
untouched on purpose, but the `-Waddress` ones are real defects in the firmware update path.

## Firmware update on probe

If `mcu_get_fw_version()` fails, probe drops the MCU into its bootloader and flashes the image
compiled into the module. A transient i2c failure during bring-up therefore triggers a flash
write. The embedded image is the same R04 firmware the camera already runs, so a reflash is
benign, but do not power cycle the board while `Trying to Detect Bootloader mode` is in dmesg.
