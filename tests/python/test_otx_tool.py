import importlib.util
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
OTX_DIR = REPO / "firmware" / "config" / "opentx"

spec = importlib.util.spec_from_file_location("otx_tool", OTX_DIR / "otx_tool.py")
assert spec is not None and spec.loader is not None
otx_tool = importlib.util.module_from_spec(spec)
sys.modules["otx_tool"] = otx_tool
spec.loader.exec_module(otx_tool)


def test_x9d_plus_2019_source_numbers():
    assert otx_tool.src_name(100) == "SI"
    assert otx_tool.src_name(165) == "TR1"
    assert otx_tool.src_name(164) != "TR1"
    assert otx_tool.MIXSRC_FIRST_LOGICAL_SWITCH == 101


def test_mr_stabs_otx_decodes_ch7_as_si_and_trainer_as_tr1():
    container = otx_tool.open_container(str(OTX_DIR / "mr_stabs_mk2.otx"))
    described = [m.describe() for m in container.models[0].mixes if not m.empty()]
    assert any(d.startswith("CH7 += SI") for d in described)
    assert any(d.startswith("CH1 += TR1") for d in described)


def test_rlc_defers_short_zero_run_into_literal_control_byte():
    data = b"\x01\x00\x00\x02"
    compressed = otx_tool.rlc_compress(data)
    assert compressed == bytes([0x01, 0x01, 0x80 | (2 << 4) | 1, 0x02])
    assert otx_tool.rlc_decompress(compressed, len(data)) == data


def test_rlc_roundtrip_mixed_runs():
    data = b"\x05" + b"\x00" * 3 + b"\x07" * 4 + b"\x00" * 20 + b"\x09" + b"\x00" * 30
    compressed = otx_tool.rlc_compress(data)
    assert otx_tool.rlc_decompress(compressed, len(data)) == data
