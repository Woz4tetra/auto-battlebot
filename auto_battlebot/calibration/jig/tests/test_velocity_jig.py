"""Guards in velocity_jig: gates, log parsing, command alignment."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from auto_battlebot.calibration.jig.velocity_jig import (
    RAW_SATURATION,
    SIDECAR_SCHEMA,
    ClockFit,
    EncoderState,
    GroundTruth,
    JigCalibration,
    JigHeader,
    JigLog,
    PauseWindow,
    RunRecord,
    WaveformSpec,
    _bridge_blips,
    _moving_average,
    _parse_header,
    _read_command_rows,
    load_run,
    load_run_record,
    load_session_dir,
    overlaps_pause,
    pause_mask,
    pause_windows_from_commands,
    read_command_log,
    read_jig_log,
    solve_ground_truth,
    zoh,
)


def _log_text(rows: list[str], header: str = "") -> str:
    lines = [f"# {header}"] if header else []
    return "\n".join(lines + rows) + "\n"


def _row(i: int) -> str:
    return f"{i * 1000},{i},0,0,0,0,0,4096"


def test_accel_gate_is_fifteen_g_on_a_16g_range() -> None:
    header = JigHeader(accel_g_per_lsb=16.0 / 32768.0)
    assert header.accel_raw_limit * header.accel_g_per_lsb == pytest.approx(15.0)


def test_accel_gate_falls_back_to_the_rail_when_15g_exceeds_int16() -> None:
    header = JigHeader(accel_g_per_lsb=8.0 / 32768.0)
    assert header.accel_raw_limit == RAW_SATURATION


def test_log_header_overrides_the_default_scale() -> None:
    header = _parse_header(["# gyro_dps_per_lsb=0.140 accel_g_per_lsb=0.000488"])
    assert header.gyro_dps_per_lsb == pytest.approx(0.140)
    assert header.accel_g_per_lsb == pytest.approx(0.000488)


def test_future_sidecar_schema_is_refused(tmp_path: Path) -> None:
    sidecar = tmp_path / "LOG-1.toml"
    sidecar.write_text(f"schema = {SIDECAR_SCHEMA + 1}\n")
    with pytest.raises(ValueError, match="schema"):
        load_run_record(sidecar)


def test_a_log_with_a_bad_last_row_counts_it_as_tail(tmp_path: Path) -> None:
    path = tmp_path / "LOG-1.TXT"
    path.write_text(_log_text([_row(i) for i in range(1, 6)] + ["60000,6,0,0"]))
    log = read_jig_log(path)
    assert (log.malformed_rows, log.malformed_tail) == (1, 1)
    assert len(log.t_boot_s) == 5


def test_a_log_with_a_bad_interior_row_is_not_tail(tmp_path: Path) -> None:
    path = tmp_path / "LOG-1.TXT"
    path.write_text(_log_text([_row(1), "garbage", _row(3), _row(4)]))
    log = read_jig_log(path)
    assert (log.malformed_rows, log.malformed_tail) == (1, 0)


def test_blip_bridging_leaves_edge_runs_alone() -> None:
    mask = np.array([0, 1, 1, 0, 1, 1, 0], dtype=bool)
    out = _bridge_blips(mask, 1)
    assert out.tolist() == [False, True, True, True, True, True, False]


def test_identity_clock_is_marked_unmeasured() -> None:
    assert ClockFit.identity().measured is False


def test_pause_windows_come_from_gaps_wider_than_a_tick() -> None:
    cmd_t = np.array([0.0, 0.02, 0.04, 5.0, 5.02])
    windows = pause_windows_from_commands(cmd_t, [1.5])
    assert [(w.t_start, w.t_end, w.program_t) for w in windows] == [(0.04, 5.0, 1.5)]


def test_channel_match_is_exact_so_combined_runs_stay_out_of_linear() -> None:
    spec = WaveformSpec(name="g", kind="grid", channel="combined")
    assert not spec.matches(channels=["linear"])
    assert spec.matches(channels=["combined"])
    assert spec.matches()


def test_unannotated_logs_are_reported_as_orphans(tmp_path: Path) -> None:
    (tmp_path / "LOG-3.TXT").write_text("x")
    session = load_session_dir(tmp_path)
    assert session.orphans == ["LOG-3.TXT"]


def test_command_rows_skip_torn_lines(tmp_path: Path) -> None:
    path = tmp_path / "c.cmd.csv"
    path.write_text("# columns: t_host_s,linear\n0.0,0.5\n0.1,0.6\n0.2,abc")
    names, rows = _read_command_rows(path)
    assert names == ["t_host_s", "linear"]
    assert rows == [
        [0.0, 0.5],
        [0.1, 0.6],
    ]


def test_command_rows_drop_short_rows(tmp_path: Path) -> None:
    path = tmp_path / "c.cmd.csv"
    path.write_text("# columns: t_host_s,linear\n0.0,0.5\n0.1\n")
    assert _read_command_rows(path)[1] == [[0.0, 0.5]]


def test_command_log_requires_the_host_clock_column(tmp_path: Path) -> None:
    path = tmp_path / "c.cmd.csv"
    path.write_text("# columns: t_s,linear\n0.0,0.5\n")
    with pytest.raises(ValueError, match="t_host_s"):
        read_command_log(path)


def test_command_log_refuses_a_stepped_host_clock(tmp_path: Path) -> None:
    path = tmp_path / "c.cmd.csv"
    path.write_text("# columns: t_host_s,linear\n1.0,0.5\n0.5,0.5\n")
    with pytest.raises(ValueError, match="monotone"):
        read_command_log(path)


def test_moving_average_does_not_pull_the_ends_toward_zero() -> None:
    out = _moving_average(np.full(20, 3.0), 5)
    np.testing.assert_allclose(out, 3.0)


def test_closure_error_is_whole_displacement_on_a_straight_run() -> None:
    truth = SimpleNamespace(
        t_s=np.linspace(0.0, 1.0, 11),
        v_body_mps=np.full(11, 1.0),
        x=np.linspace(0.0, 1.0, 11),
        y=np.zeros(11),
    )
    err, path_len = GroundTruth.closure_error(truth)  # type: ignore[arg-type]
    assert err == pytest.approx(1.0)
    assert path_len == pytest.approx(1.0)


def _flat_log(n: int, encoder: bool) -> JigLog:
    return JigLog(
        path=Path("x"),
        header=JigHeader(),
        t_boot_s=np.arange(n) * 0.001,
        count_x4=np.arange(n) * 10 if encoder else np.zeros(n, dtype=np.int64),
        gyro_raw_int16=np.zeros((n, 3), dtype=np.int64),
        accel_raw_int16=np.zeros((n, 3), dtype=np.int64),
    )


def test_detached_encoder_is_flagged_not_read_as_a_stationary_run() -> None:
    calib = JigCalibration(meters_per_count=1e-4)
    detached = solve_ground_truth(_flat_log(200, encoder=False), calib)
    attached = solve_ground_truth(_flat_log(200, encoder=True), calib)
    assert detached.encoder_valid is False
    assert attached.encoder_valid is True


def test_zoh_is_zero_outside_the_command_log() -> None:
    out = zoh(np.array([1.0, 2.0]), np.array([0.5, 0.7]), np.array([0.0, 1.5, 2.0, 3.0]))
    assert out.tolist() == [0.0, 0.5, 0.7, 0.0]


def test_pause_windows_are_excluded_from_gates() -> None:
    pauses = [PauseWindow(t_start=2.0, t_end=5.0, program_t=1.0)]
    t_host = np.array([1.9, 2.0, 3.5, 5.0, 5.1])
    assert pause_mask(t_host, pauses).tolist() == [False, True, True, True, False]
    assert overlaps_pause(np.array([4.0, 6.0]), pauses)
    assert not overlaps_pause(np.array([5.5, 6.0]), pauses)


def test_from_toml_rejects_zero_meters_per_count(tmp_path: Path) -> None:
    path = tmp_path / "calib.toml"
    path.write_text("[encoder]\nmeters_per_count = 0.0\n")
    with pytest.raises(ValueError, match="meters_per_count"):
        JigCalibration.from_toml(path)


def _still_run_record(pauses: list[PauseWindow]) -> RunRecord:
    return RunRecord(
        run_id="r",
        session_id="s",
        spec=WaveformSpec(name="w", kind="step", channel="linear"),
        rep=1,
        encoder=EncoderState.DETACHED,
        log_file="LOG-1.TXT",
        command_file=None,
        verdict=None,
        gate_note=None,
        notes="",
        trim=None,
        samples=None,
        dropped=None,
        clock_pre=None,
        clock_post=None,
        skew_ppm=None,
        hold_pre_host_s=None,
        hold_post_host_s=None,
        segments=[],
        pauses=pauses,
        cmd_t_host_s=np.array([0.0, 8.0]),
        cmd_lin=np.zeros(2),
        cmd_ang=np.zeros(2),
    )


def test_pause_windows_excluded_from_still_holds_and_commanded_time(tmp_path: Path) -> None:
    (tmp_path / "LOG-1.TXT").write_text(
        _log_text([f"{i * 1000},0,0,0,0,0,0,4096" for i in range(1, 8001)])
    )
    calib = JigCalibration(meters_per_count=1e-4)
    free = load_run(_still_run_record([]), tmp_path, calib)
    paused = load_run(_still_run_record([PauseWindow(1.0, 3.0, 0.0)]), tmp_path, calib)
    assert free.quality.still_holds == 1
    assert paused.quality.still_holds == 0
    assert paused.quality.commanded_s == pytest.approx(free.quality.commanded_s - 2.0, abs=0.05)


def test_pause_windows_are_excluded_from_clipped_accel_counts(tmp_path: Path) -> None:
    rows = [f"{i * 1000},0,0,0,0,0,0,{32000 if 1500 <= i < 1600 else 4096}" for i in range(1, 8001)]
    header = "accel_g_per_lsb=0.00048828125"
    (tmp_path / "LOG-1.TXT").write_text(_log_text(rows, header))
    calib = JigCalibration(meters_per_count=1e-4)
    free = load_run(_still_run_record([]), tmp_path, calib)
    paused = load_run(_still_run_record([PauseWindow(1.0, 3.0, 0.0)]), tmp_path, calib)
    assert free.quality.accel_clip_samples == 100
    assert paused.quality.accel_clip_samples == 0
