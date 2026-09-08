"""Reading and writing the Foxglove recording format, plus the ZED SVO container.

The schemas here mirror `include/foxglove_adapters/json_schemas.hpp`; the contract is
documented in `docs/foxglove_recording_format.md` and guarded by
`tests/python/test_json_schema_parity.py`.

mcap_io     reader: iter_messages and the decode_* dispatch
mcap_write  writer: the McapWriter wrapper over the Foxglove SDK
svo2        ZED .svo2 containers, read without the ZED SDK
diag_io     control-metric loaders layered over the diagnostics decoder
"""
