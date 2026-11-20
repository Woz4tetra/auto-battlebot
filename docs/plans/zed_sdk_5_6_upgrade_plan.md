# ZED SDK 5.6.1 upgrade on the ZED Box Mini

Upgrade the ZED Box Mini from ZED SDK 5.2.3 and GMSL driver 1.4.1 to SDK 5.6.1 and the matching
driver, staying on JetPack 6.2.1 (L4T R36.4.4). Then retest each SDK 5.2 workaround in
`src/rgbd_camera/zed_one_rgb_camera.cpp` on the ZED X One GS and delete the ones the upgrade fixes.

## Trigger

Start when Stereolabs publishes 5.6.1 on https://www.stereolabs.com/developers/release. 5.6.0
came out 2026-11-16; waiting for the first patch release avoids its launch bugs. Before starting,
read the 5.6.1 notes and confirm all three:

- A JetPack 6.1/6.2 (L4T 36.4) installer is still listed. 5.6.0 lists it as "Legacy".
- https://www.stereolabs.com/developers/drivers lists a driver for the ZED Box Mini on L4T 36.4 that
  supports 5.6.1 (1.5.0 or newer), with an RT-kernel build if the box needs one.
- No 5.6.1 note reopens one of the items under "What 5.6 should fix" below.

If any of the three fails, stay on 5.2.3 and record why here.

## Why JetPack 6.2.1, not 7.2

- 5.6 ships for L4T 36.4, so the SDK moves without a reflash.
- The Box Mini's JetPack 7.2 path needs a micro-USB reflash from an Ubuntu 20.04/22.04 host, and the
  drivers page lists no RT-kernel build for the Mini on L4T 39.2.
- The workarounds below are SDK bugs; 7.2 fixes none of them that 5.6 on 6.2.1 does not.

## What 5.6 should fix

From the 5.5 and 5.6.0 release notes as read on 2026-11-20. Recheck each against the 5.6.1 notes.

| Workaround | Where | Release note | Expected after the upgrade |
| --- | --- | --- | --- |
| Exposure and gain read back -1; `read_settings` substitutes the last applied value | `zed_one_rgb_camera.cpp` `get_or` in `read_settings` | 5.6.0: `getCameraSettings()` no longer returns -1 for EXPOSURE/GAIN in auto mode | Reads return real values in auto and manual |
| A repeated value is skipped, so each write is preceded by a neighbouring value | `zed_one_rgb_camera.cpp` `set` lambda in `apply_settings`, `nudge` | 5.6.0: writes that were not applied now return an error, and setting a previous value is applied, but the notes name USB cameras only | Unknown for GMSL. Test it |
| `AE_ANTIBANDING` behind `AB_ZED_HAS_ANTIBANDING` | `zed_one_rgb_camera.cpp` `AB_ZED_HAS_ANTIBANDING` | Added in 5.3.0. On ZED X cameras it read back -1 until 5.6.0 | Guard can go once every box with ZED code builds on SDK 5.3 or newer |
| Rectified BGRA retrieved, then `cvtColor` to BGR every frame | `zed_one_rgb_camera.cpp` `capture_frame` `cvtColor` after `retrieveImage` | No `LEFT_BGR` view in any release through 5.6.0 | Stays, unless 5.6.1 adds the view |
| Destructor `_Exit`s on a `grab()` that will not return | `zed_one_rgb_camera.cpp` `~ZedOneRgbCamera` | 5.6.0: GMSL `close()` could hang and `CameraOne::close()` crashed while streaming; both fixed | Retest. Keep the guard if a hang is seen once |
| A stuck `open()` is leaked after 30 s so shutdown cannot hang | `src/rgbd_camera/zed_device.cpp` `kOpenHardTimeout`, `g_leaked_open_futures` | 5.6.0: Argus daemon restarts no longer leave cameras stuck | Keep. It guards a hardware failure, not only an SDK bug |

## Before the upgrade

1. Hold the kernel. `docs/setup/zed_box_mini.md` says `apt-mark showhold` is empty; run
   `hold-zedbox-kernel.sh` (or `apt-mark hold` on every `nvidia-l4t-*` package) and check
   `apt-mark showhold` lists them.
2. Keep rollback files on the box: the 5.2.3 installer (`ZED_SDK_SHA256` in
   `install/install_zed_sdk.sh`) and the `stereolabs-zedbox-mini` 1.4.1 `.deb`.
3. Record a baseline on 5.2.3 with the ZED X One GS: 60 s of `auto_battlebot` with recording on
   (frame rate, `grab` latency from the diagnostics), then a settings round trip from the dashboard
   (exposure, analog gain, digital gain, antibanding, back to the profile DEFAULT), noting the
   read-back values and the image brightness. Ten start-stop cycles of the service, noting any hang.
4. Copy the baseline MCAP to the dev box.

## Upgrade

1. Install the new driver `.deb` for L4T 36.4 and reboot. `zed_x_daemon` runs and `sl_zedx` loads.
2. Bump `ZED_SDK_VERSION`, `ZED_SDK_L4T` and `ZED_SDK_SHA256` in `install/install_zed_sdk.sh` and run
   it. Keep the Box Mini row in `docs/setup/zed_box_mini.md` in step.
3. Rebuild with `./scripts/build.sh` on the box. TensorRT engines do not depend on the ZED SDK and
   stay as they are.
4. Confirm the startup log still reads `ZED XOne GS serial ... at 1920x1200 60 fps`.

## Verify

Repeat the baseline from step 3 of "Before the upgrade" and compare.

| Check | Pass |
| --- | --- |
| 60 s capture | Same frame rate as the baseline, `grab` latency no worse |
| Read-back with `get_or` bypassed (temporary local build) | Real exposure and gain values in auto and manual, never -1 |
| Settings writes with `nudge = false` (temporary local build) | Restoring the profile DEFAULT turns auto exposure off: brightness drops as it does with the nudge (measured 86 vs 8 on 5.2.3) |
| Antibanding | Reads back the value written, including AUTO |
| Ten service start-stop cycles | No hang, no `_Exit` path taken (check the log) |
| Close and reopen the camera from the dashboard | Profile settings apply after the reopen (5.6.0 fixed `sl::CameraOne` keeping old settings across `close()`/`open()`) |
| Replay of the new MCAP on the dev box | Decodes and replays like the baseline |

Also run the ZED rows of "Hardware checks" in `docs/plans/comment_purge_and_config_clarity_plan.md`
in the same session (camera cancel during a failing open, ZED X open and settings, SVO recorder),
and delete each row that passes.

## Clean up

For each workaround that the checks show is fixed, delete it in its own commit, quoting the check
result in the message, and drop its row from `docs/plans/comment_purge_hack_ledger.md`. The
`AB_ZED_HAS_ANTIBANDING` guard goes only once every box that builds the ZED code runs SDK 5.3 or
newer (the dev box has 5.5.0).

## Rollback

Reinstall the 1.4.1 driver `.deb` and the 5.2.3 SDK, reboot, rebuild. The kernel hold keeps the
driver's kernel range valid either way.

## Done when

- The Box Mini runs SDK 5.6.1 with the matching driver and passes every check under Verify.
- Fixed workarounds are deleted and their ledger rows are gone; the ones that stay have a ledger
  row saying 5.6.1 did not fix them.
- `docs/setup/zed_box_mini.md` lists the new SDK and driver versions, and this plan is deleted.
