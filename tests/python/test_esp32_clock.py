import numpy as np

from auto_battlebot.recording.esp32_clock import split_reboots


def test_small_backstep_is_not_a_reboot() -> None:
    robot_ms = np.array([0, 100, 200, 150, 300, 400])
    assert split_reboots(robot_ms, max_backstep_ms=1000) == [(0, 5)]


def test_backstep_past_limit_starts_a_new_boot() -> None:
    robot_ms = np.array([0, 5000, 10000, 20, 120])
    assert split_reboots(robot_ms, max_backstep_ms=1000) == [(0, 2), (3, 4)]
