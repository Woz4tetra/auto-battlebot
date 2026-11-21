"""Each lockstep eval server runs single-threaded so parallel workers do not oversubscribe."""

from __future__ import annotations

from auto_battlebot.rl import lockstep_eval


def test_server_env_caps_every_blas_and_openmp_pool_at_one_thread() -> None:
    env = lockstep_eval.server_env({"OMP_NUM_THREADS": "8", "PATH": "/bin"})

    assert env["OMP_NUM_THREADS"] == "1"
    assert env["MKL_NUM_THREADS"] == "1"
    assert env["OPENBLAS_NUM_THREADS"] == "1"
    assert env["PATH"] == "/bin"
    assert env["PYTHONUNBUFFERED"] == "1"


def test_server_env_does_not_modify_the_parent_environment() -> None:
    base = {"OMP_NUM_THREADS": "8"}

    lockstep_eval.server_env(base)

    assert base == {"OMP_NUM_THREADS": "8"}
