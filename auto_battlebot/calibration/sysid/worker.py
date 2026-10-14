"""Refits for the sysid monitor, off the session's thread.

``RefitState`` keeps every usable window's sub-windows and turns a ``RefitJob`` into a
``RefitReply``: a warm-started fit, the Fisher matrix over every fit window, the contact gate,
the holdout score, every third time the model ladder, and the ranked suggestions.

Two engines run it. ``InlineEngine`` calls it on ``submit``, which keeps tests and replays
deterministic. ``ProcessEngine`` runs it in a spawned process so a live session keeps labeling
while a refit takes a few seconds.
"""

from __future__ import annotations

import math
import multiprocessing as mp
import queue
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from auto_battlebot.calibration.sysid import model
from auto_battlebot.calibration.sysid.maneuvers import (
    HOLD_S,
    PLATEAU_TOL,
    REST,
    bin_of,
    edges_of,
)
from auto_battlebot.control.plant import MODEL_LADDER

MAX_FIT_WINDOWS = 300  # the fit thins to this many windows; the Fisher matrix uses them all
LADDER_EVERY = 3
CONTACT_SIGMA = 4.0
CONTACT_AFTER_REFITS = 3
CONTACT_AFTER_WINDOWS = 10


@dataclass
class RefitJob:
    theta: np.ndarray
    ladder: dict[str, np.ndarray]
    refit_index: int
    add: list[tuple[int, str, list[model.SubWindow]]]  # (window id, used | holdout, subs)
    upto: int
    loop_on: bool
    floor_run_m: float
    view_azimuth: float
    t: float
    driving_s: float


@dataclass
class RefitReply:
    theta: np.ndarray
    ladder: dict[str, np.ndarray]
    fisher: np.ndarray
    scale: float
    rows: int
    contact: list[int]
    holdout: float
    ladder_holdout: dict[str, float]
    candidates: list[dict[str, Any]]
    floor_limited: list[bool]
    upto: int
    driving_s: float
    t: float = 0.0
    timings: dict[str, float] = field(default_factory=dict)


def _hint_cells(motion: model.Motion) -> list[list[int]]:
    """Coverage-map cells a motion holds for at least HOLD_S, or sweeps through if it holds
    none (a creep)."""
    thr, st = motion.throttle, motion.steer
    n_hold = int(round(HOLD_S / model.DT))
    cells: set[tuple[int, int]] = set()
    start = 0
    for k in range(1, len(thr) + 1):
        if (
            k == len(thr)
            or abs(thr[k] - thr[start]) > PLATEAU_TOL / 3
            or abs(st[k] - st[start]) > PLATEAU_TOL / 3
        ):
            if k - start >= n_hold and (abs(thr[start]) > REST or abs(st[start]) > REST):
                cells.add((bin_of(float(thr[start])), bin_of(float(st[start]))))
            start = k
    if not cells:
        for a, b in zip(thr, st):
            if abs(a) > REST or abs(b) > REST:
                cells.add((bin_of(float(a)), bin_of(float(b))))
    return [list(c) for c in sorted(cells)]


class RefitState:
    def __init__(self) -> None:
        self.fit_windows: dict[int, list[model.SubWindow]] = {}
        self.holdout_windows: dict[int, list[model.SubWindow]] = {}
        self.contact: set[int] = set()
        self.last_ladder_holdout: dict[str, float] = {}

    def process(self, job: RefitJob) -> RefitReply:
        import time

        timings: dict[str, float] = {}
        for wid, status, subs in job.add:
            (self.holdout_windows if status == "holdout" else self.fit_windows)[wid] = subs
        ids = sorted(w for w in self.fit_windows if w not in self.contact)

        t0 = time.monotonic()
        if len(ids) > MAX_FIT_WINDOWS:
            pick = np.unique(np.linspace(0, len(ids) - 1, MAX_FIT_WINDOWS).round().astype(int))
            fit_ids = [ids[i] for i in pick]
        else:
            fit_ids = ids
        thin = self._batch(fit_ids, job.view_azimuth)
        result = model.fit(job.theta, thin, max_nfev=6)
        timings["fit"] = time.monotonic() - t0

        full = self._batch(ids, job.view_azimuth)
        contact: list[int] = []
        if full.rows:
            r0, jac = model.jacobian(result.theta, full)
            mask = model.data_mask(full)
            n = int(mask.sum())
            scale = float(np.sum(r0[mask] ** 2) / max(n, 1))
            if job.refit_index >= CONTACT_AFTER_REFITS and len(ids) >= CONTACT_AFTER_WINDOWS:
                limit = CONTACT_SIGMA * math.sqrt(max(scale, 1.0))
                tag_cols = 3 * model.MAX_TAGS
                worst = np.max(np.abs(r0[:, :tag_cols]), axis=1)
                bad = {int(full.owner[i]) for i in np.flatnonzero(worst > limit)}
                contact = sorted(bad)
                self.contact |= bad
                keep = ~np.isin(full.owner, list(bad))
                jac = jac[keep]
                r0 = r0[keep]
                mask = mask[keep]
                n = int(mask.sum())
                scale = float(np.sum(r0[mask] ** 2) / max(n, 1))
            fisher = model.information(jac)
        else:
            fisher, scale, n = np.zeros((model.NP, model.NP)), 1.0, 0
        timings["fisher"] = time.monotonic() - t0 - timings["fit"]

        holdout_subs = [s for subs in self.holdout_windows.values() for s in subs]
        holdout = model.holdout_error(result.theta, holdout_subs) if holdout_subs else math.nan

        ladder = {k: v.copy() for k, v in job.ladder.items()}
        if job.refit_index % LADDER_EVERY == 0 and thin.rows:
            t1 = time.monotonic()
            scores: dict[str, float] = {}
            for rung in MODEL_LADDER:
                fitted = model.fit(
                    ladder[rung.name], thin, model.RUNG_FREE[rung.name], rung, max_nfev=3
                )
                ladder[rung.name] = fitted.theta
                if holdout_subs:
                    scores[rung.name] = model.holdout_error(fitted.theta, holdout_subs, rung)
            self.last_ladder_holdout = scores
            timings["ladder"] = time.monotonic() - t1

        t2 = time.monotonic()
        ranked = model.rank_candidates(
            result.theta, fisher, scale, job.loop_on, job.floor_run_m, job.view_azimuth
        )
        fills = model.fills_of(np.sqrt(np.diag(model.covariance(fisher, scale))))
        limited = model.floor_limited(fills, ranked)
        timings["candidates"] = time.monotonic() - t2

        return RefitReply(
            theta=result.theta,
            ladder=ladder,
            fisher=fisher,
            scale=scale,
            rows=n,
            contact=contact,
            holdout=holdout,
            ladder_holdout=self.last_ladder_holdout,
            candidates=[self._candidate(c, job.loop_on) for c in ranked],
            floor_limited=limited.tolist(),
            upto=job.upto,
            driving_s=job.driving_s,
            t=job.t,
            timings=timings,
        )

    def _batch(self, ids: list[int], view_azimuth: float) -> model.Batch:
        subs: list[model.SubWindow] = []
        owners: list[int] = []
        for wid in ids:
            subs += self.fit_windows[wid]
            owners += [wid] * len(self.fit_windows[wid])
        return model.stack(subs, owners, view_azimuth)

    @staticmethod
    def _candidate(c: model.Candidate, loop_on: bool) -> dict[str, Any]:
        m = c.motion
        edges = edges_of(m.throttle, m.steer, np.full(len(m.throttle), loop_on), model.DT)
        return {
            "id": m.id,
            "name": m.name,
            "how": m.how,
            "gains": np.round(c.gains, 4).tolist(),
            "score": c.score,
            "per_sec": c.per_sec,
            "travel": c.travel,
            "fits": c.fits,
            "seconds": len(m.throttle) * model.DT,
            "thr": np.round(m.throttle[::4], 3).tolist(),
            "steer": np.round(m.steer[::4], 3).tolist(),
            "cells": _hint_cells(m),
            "edge_cells": sorted({e.edge_cell for e in edges if e.edge_cell is not None}),
        }


class RefitEngine(Protocol):
    @property
    def busy(self) -> bool: ...

    def submit(self, job: RefitJob) -> None: ...

    def poll(self) -> RefitReply | None: ...

    def close(self) -> None: ...


class InlineEngine:
    """Runs each refit on submit. The reply is handed back on the next poll."""

    def __init__(self) -> None:
        self.state = RefitState()
        self._reply: RefitReply | None = None

    @property
    def busy(self) -> bool:
        return self._reply is not None

    def submit(self, job: RefitJob) -> None:
        self._reply = self.state.process(job)

    def poll(self) -> RefitReply | None:
        reply, self._reply = self._reply, None
        return reply

    def close(self) -> None:
        pass


def _serve(jobs: Any, replies: Any) -> None:
    state = RefitState()
    while True:
        job = jobs.get()
        if job is None:
            return
        replies.put(state.process(job))


class ProcessEngine:
    """Runs refits in a spawned process; one job in flight at a time."""

    def __init__(self) -> None:
        ctx = mp.get_context("spawn")
        self._jobs = ctx.Queue()
        self._replies = ctx.Queue()
        self._proc = ctx.Process(target=_serve, args=(self._jobs, self._replies), daemon=True)
        self._proc.start()
        self._busy = False

    @property
    def busy(self) -> bool:
        return self._busy

    def submit(self, job: RefitJob) -> None:
        self._busy = True
        self._jobs.put(job)

    def poll(self) -> RefitReply | None:
        if not self._busy:
            return None
        try:
            reply: RefitReply = self._replies.get_nowait()
        except queue.Empty:
            if not self._proc.is_alive():
                raise RuntimeError("the refit process died") from None
            return None
        self._busy = False
        return reply

    def close(self) -> None:
        self._jobs.put(None)
        self._proc.join(timeout=5)
