# CRSF telemetry loss between May and August 2026

Graded with `scripts/verify_mcap` at commit 36a49834 plus the PerformanceMonitor change, struct
defaults, and counted directly from `/diagnostics/opentx_transmitter` with the Python MCAP reader.
Mrs Buff Mk3 on the EdgeTX radio (`OpenTxTransmitter`) in both sets.

## Counts

Non-held subsections across the primary file of each session (rollover `__` copies excluded):

| Set | Files | `send` | `link_statistics` | `battery`, `attitude`, `flight_mode` |
| --- | --- | --- | --- | --- |
| `data/saved_recordings/NHRL_2026-05-02` | 7 | 33,886 | 3,607 | 0 |
| `data/saved_recordings/MassD_2026-08-29` | 8 | 37,609 | 0 | 0 |

The app sent trainer commands at the same rate in both sets. In August not one CRSF frame passed
the CRC check.

`auto_battlebot_mrs_buff_mk3_massd_ns_jetson_2026-08-29_09-05-11.mcap` had the radio's SA switch on
in 8 windows totalling 117 s (from `switch_states`), and 0 link statistics in any of them.

## verify_mcap

- May: 0 of 7 files raise `crsf_telemetry_link`. May recordings carry no SA state, so the link is
  noted as not judged; the counts above are what shows telemetry flowed.
- August: 4 of 8 files go red on `crsf_telemetry_link` (09-05-11, 09-20-17, 13-08-16, 15-21-21).
  The other 4 have no SA-on window longer than the 10 s max age.

## Parser resync bug found on the way

The test that interleaves the OpenTX channel stream with CRSF frames on one port
(`OpenTxTransmitterTest.CrsfTelemetryThatStopsWhileChannelsFlowGoesRed`) read 0 of 40 frames.

- Bug: a channel packet's phase byte `0x00` is a CRSF start byte. The parser took the next byte as
  a length, failed the CRC, and skipped the whole bogus length, which swallowed the real frame
  behind it.
- Fix: on a CRC failure `CrsfParser::parse` advances one byte. The same test then reads 39 or 40 of
  40; the newest frame can wait one read behind a false start near the end of a chunk.

The May recordings parsed 3,607 link statistics through the same stream layout, so this bug is not
shown to be the August cause. What changed between May 2 and August 29 (radio firmware, model
setup, or module) is not in the recordings.

## Also fixed

`CrsfParser` read the attitude payload as roll, pitch, yaw. CRSF and the capture in
`tests/fixtures/crsf/edgetx_telemetry_capture.log` (from
`firmware/mr_stabs_mk2/scripts/capture_crsf_telemetry.py`) order it pitch, roll, yaw. Every
captured attitude and battery frame now decodes to the values the capture script printed, and
re-encodes byte for byte.

## Next steps

1. Plug the EdgeTX radio in with the Mk3 receiver bound and run `capture_crsf_telemetry.py` with
   `channels on` as well, to see whether link statistics reach the USB port today.
2. Grade the next Mk3 session with `scripts/verify_mcap`; it now carries `/status/system` and SA, so
   every SA-on window is judged.
