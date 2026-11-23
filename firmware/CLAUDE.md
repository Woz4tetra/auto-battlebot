# firmware

- clang-format covers C/C++ under `firmware/` except PlatformIO's `.pio` and the vendored
  `firmware/BlHeli-Passthrough`
- prettier (with `web/.prettierrc`) covers the firmware web pages, which stay with their firmware
  (`firmware/mr_stabs_mk2/web/`)
- Generated files opt out with `// clang-format off` (`velocity_jig/include/splash_anim.h`)
- Firmware native tests run with `pio test -e native`; `scripts/verify` includes them
- Firmware script tests live in `firmware/mr_stabs_mk2/scripts/tests/`
