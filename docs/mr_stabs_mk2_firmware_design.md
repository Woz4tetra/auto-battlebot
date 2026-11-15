# mr stabs mk2 firmware design

## Notes moved from code comments

### General

- `mr_stabs_mk2/include/vbat_sensor.h`: After a failed read (unplugged cable), wait this long before trying again, so a missing chip costs one blocked transaction per second rather than one per sample.
