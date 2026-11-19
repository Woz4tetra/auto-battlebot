# 2026-10-23 esc tuning reentry

## Notes moved from code comments

### General

- `mr_stabs_mk2/include/esc_config.h`: Entry waits until COMBAT has lasted this long. Exiting restarts the ESCs, and entering again right away left them out of the bootloader 3 times in 8; after 1 s of DShot, 0 in 14. Measured on 2026-10-23 with the earlier software serial link.
