"""Self-contained HTML report on a velocity jig plant fit.

Answers two questions the fitted numbers alone do not: which excitation needs more runs, and
how much more data is worth collecting. Everything is embedded, so the file opens with no
network and can be copied around on its own.

Sections, in reading order:

  0  What to collect next    parameters ranked by confidence, mapped to the waveform that
                             constrains them, with a run count to reach the target
  1  Data inventory          per waveform: runs, usable seconds, windows contributed
  2  Run quality             the capture and offline gates, per run
  3  Parameters              stage A estimates with spread and provenance
  4  Evidence                gain scatters, staircases, tau trends, coupling, drift, delay
  5  Convergence             leave-one-run-out influence, and the learning curve
  6  Coverage                what of the input space has actually been visited
  7  Horizon quality         RMSE against the acceptance criteria
  8  Residual diagnostics    what a missing model term would look like
  9  Per run                 measured against predicted, with residuals
 10  Excluded and provenance

Imported by `fit_jig_plant.py --report`. It imports from `calib_lib`, never from a CLI.
"""

from __future__ import annotations

import base64
import html
import io
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from auto_battlebot.plant import (  # noqa: E402
    PARAM_BOUNDS,
    ModelStructure,
    PlantParams,
    WindowErrors,
    effective_command,
    predict_windows,
)
from auto_battlebot.velocity_jig import Run  # noqa: E402
from calib_lib.jig_fit import (  # noqa: E402
    PARAM_SOURCES,
    Estimate,
    FitWeights,
    Loaded,
    build_windows,
    joint_fit,
    score,
    window_delay,
)

# Relative spread below which a parameter counts as measured. 10% is the point where the
# parameter stops being the thing limiting the model.
TARGET_REL = 0.10
WEAK_REL = 0.30

CSS = """
:root { --bg:#fff; --fg:#1a1a1a; --muted:#666; --line:#ddd; --ok:#0a7d32; --warn:#a86400;
        --bad:#b3261e; --okbg:#eaf6ed; --warnbg:#fdf3e2; --badbg:#fdeceb; --code:#f6f6f6; }
* { box-sizing: border-box; }
body { margin:0; font:15px/1.55 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
       color:var(--fg); background:var(--bg); }
.wrap { display:flex; align-items:flex-start; }
nav { position:sticky; top:0; width:230px; flex:0 0 230px; padding:24px 16px; height:100vh;
      overflow:auto; border-right:1px solid var(--line); font-size:13px; }
nav a { display:block; padding:4px 0; color:var(--muted); text-decoration:none; }
nav a:hover { color:var(--fg); }
main { flex:1; min-width:0; padding:24px 32px 80px; max-width:1100px; }
h1 { font-size:26px; margin:0 0 4px; }
h2 { font-size:19px; margin:38px 0 10px; padding-top:10px; border-top:1px solid var(--line); }
h3 { font-size:15px; margin:20px 0 6px; color:var(--muted); font-weight:600; }
p.lede { color:var(--muted); margin:0 0 18px; }
table { border-collapse:collapse; width:100%; font-size:13px; margin:10px 0 18px; }
th,td { text-align:left; padding:5px 9px; border-bottom:1px solid var(--line); vertical-align:top; }
th { font-weight:600; color:var(--muted); font-size:12px; text-transform:uppercase;
     letter-spacing:.03em; }
td.num { text-align:right; font-variant-numeric:tabular-nums; }
tr.ok td { background:var(--okbg); } tr.warn td { background:var(--warnbg); }
tr.bad td { background:var(--badbg); }
.chip { display:inline-block; padding:1px 7px; border-radius:9px; font-size:11px;
        margin:1px 3px 1px 0; }
.chip.ok { background:var(--okbg); color:var(--ok); }
.chip.warn { background:var(--warnbg); color:var(--warn); }
.chip.bad { background:var(--badbg); color:var(--bad); }
figure { margin:14px 0 22px; } figure img,figure svg { max-width:100%; height:auto;
        display:block; }
figcaption { color:var(--muted); font-size:12.5px; margin-top:6px; }
pre { background:var(--code); padding:12px; overflow:auto; font-size:12px; border-radius:4px; }
details { border:1px solid var(--line); border-radius:4px; margin:8px 0; padding:8px 12px; }
summary { cursor:pointer; font-size:13.5px; }
.note { border-left:3px solid var(--line); padding:2px 0 2px 12px; color:var(--muted);
        font-size:13.5px; margin:10px 0; }
.scroll { overflow-x:auto; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#141414; --fg:#e8e8e8; --muted:#9a9a9a; --line:#333; --code:#1e1e1e;
          --okbg:#122a18; --warnbg:#2c2411; --badbg:#2e1614; --ok:#5fcf80; --warn:#e0aa4a;
          --bad:#ef6a60; }
}
"""


# ---------------------------------------------------------------------------
# Plumbing
# ---------------------------------------------------------------------------


def figure_uri(fig, fmt: str = "png", dpi: int = 110) -> str:
    """Render a figure to a data: URI. SVG for line plots, PNG where SVG would balloon."""
    buf = io.BytesIO()
    fig.savefig(buf, format=fmt, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    mime = "image/svg+xml" if fmt == "svg" else "image/png"
    return f"data:{mime};base64,{base64.b64encode(buf.getvalue()).decode('ascii')}"


def figure_html(fig, caption: str = "", fmt: str = "png") -> str:
    uri = figure_uri(fig, fmt)
    cap = f"<figcaption>{html.escape(caption)}</figcaption>" if caption else ""
    return f'<figure><img src="{uri}" alt="{html.escape(caption)}">{cap}</figure>'


def esc(value: Any) -> str:
    return html.escape(str(value))


def table_html(
    headers: Sequence[str],
    rows: Sequence[Sequence[Any]],
    *,
    row_cls: Sequence[str] | None = None,
    numeric: Sequence[int] = (),
) -> str:
    if not rows:
        return '<p class="note">nothing to show.</p>'
    out = ['<div class="scroll"><table><thead><tr>']
    out += [f"<th>{esc(h)}</th>" for h in headers]
    out.append("</tr></thead><tbody>")
    for i, row in enumerate(rows):
        cls = f' class="{row_cls[i]}"' if row_cls and row_cls[i] else ""
        out.append(f"<tr{cls}>")
        for j, cell in enumerate(row):
            td = ' class="num"' if j in numeric else ""
            out.append(f"<td{td}>{cell if isinstance(cell, str) else esc(cell)}</td>")
        out.append("</tr>")
    out.append("</tbody></table></div>")
    return "".join(out)


def chip(text: str, level: str = "ok") -> str:
    return f'<span class="chip {level}">{esc(text)}</span>'


def fmt(value: float, digits: int = 3) -> str:
    if value is None or not np.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


@dataclass
class Section:
    ident: str
    title: str
    body: str = ""
    parts: list[str] = field(default_factory=list)

    def add(self, part: str) -> None:
        self.parts.append(part)

    def render(self) -> str:
        return (
            f'<section id="{self.ident}"><h2>{esc(self.title)}</h2>'
            + self.body
            + "".join(self.parts)
            + "</section>"
        )


def render_page(path: Path, title: str, sections: Sequence[Section], lede: str) -> None:
    nav = "".join(f'<a href="#{s.ident}">{esc(s.title)}</a>' for s in sections)
    body = "".join(s.render() for s in sections)
    doc = (
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        f"<title>{esc(title)}</title><style>{CSS}</style></head><body><div class='wrap'>"
        f"<nav>{nav}</nav><main><h1>{esc(title)}</h1>"
        f'<p class="lede">{esc(lede)}</p>{body}</main></div></body></html>'
    )
    path.write_text(doc, encoding="utf-8")
    print(f"wrote {path}")


# ---------------------------------------------------------------------------
# 0. What to collect next
# ---------------------------------------------------------------------------


@dataclass
class ParamStatus:
    name: str
    value: float
    spread: float
    n: int
    note: str
    sources: tuple[tuple[str, str], ...]
    runs_now: int
    valid_s_now: float

    @property
    def rel(self) -> float:
        """Spread relative to the value, or to the bound width when the value is near zero.

        A coupling coefficient of 0.02 +/- 0.04 has an infinite relative spread by the naive
        definition, which would sort it above everything else forever. Scaling by the
        parameter's own bound width keeps "we cannot tell this from zero" comparable to
        "this gain is known to 3%".
        """
        if not np.isfinite(self.value) or not np.isfinite(self.spread):
            return float("inf")
        scale = abs(self.value)
        if scale < 1e-6:
            lo, hi = PARAM_BOUNDS.get(self.name, (0.0, 1.0))
            scale = max(abs(hi - lo) * 0.05, 1e-6)
        return self.spread / scale

    @property
    def status(self) -> str:
        if not np.isfinite(self.value):
            return "not measured"
        if self.rel < TARGET_REL:
            return "measured"
        if self.rel < WEAK_REL:
            return "weak"
        return "not measured"

    @property
    def level(self) -> str:
        return {"measured": "ok", "weak": "warn", "not measured": "bad"}[self.status]

    def runs_needed(self, target_rel: float = TARGET_REL) -> int:
        """How many more runs to reach the target, by the root-N law.

        Spread falls as 1/sqrt(N), so N scales as (rel_now / rel_target)^2. This is an
        estimate from one point on the curve; the learning curve section measures the slope
        instead of assuming it.
        """
        if self.runs_now < 1 or not np.isfinite(self.rel) or self.rel <= target_rel:
            return 0
        return max(1, int(np.ceil(self.runs_now * (self.rel / target_rel) ** 2)) - self.runs_now)


def param_status(loaded: Loaded, stage) -> list[ParamStatus]:
    by_kc = loaded.by_kind_channel()
    out: list[ParamStatus] = []
    for name, est in stage.estimates.items():
        sources = PARAM_SOURCES.get(name)
        if sources is None:
            continue  # a derived diagnostic like k_ang_left, not a model parameter
        runs: list[Run] = []
        for kind, channel in sources:
            for (k, c), rs in by_kc.items():
                if k == kind and channel in ("*", c):
                    runs.extend(rs)
        unique = {id(r): r for r in runs}.values()
        out.append(
            ParamStatus(
                name=name,
                value=est.value,
                spread=est.spread,
                n=est.n,
                note=est.note,
                sources=sources,
                runs_now=len(unique),
                valid_s_now=float(sum(r.quality.valid_s for r in unique)),
            )
        )
    out.sort(key=lambda s: (-s.rel if np.isfinite(s.rel) else -1e18))
    return out


def _source_label(sources: Sequence[tuple[str, str]]) -> str:
    return ", ".join(f"{k}/{c}" for k, c in sources)


def section_next(statuses: Sequence[ParamStatus], curves: dict[str, float]) -> Section:
    sec = Section("next", "0. What to collect next")
    sec.body = (
        '<p class="note">Parameters ranked by how poorly determined they are, each mapped to '
        "the excitation that constrains it. The run count is an estimate from the root-N law "
        "unless a learning curve measured the slope, in which case the measured slope is used."
        "</p>"
    )
    rows, cls = [], []
    for s in statuses:
        if s.status == "measured":
            continue
        need = s.runs_needed()
        slope = curves.get(s.name)
        basis = "measured slope" if slope else "root-N estimate"
        if slope and slope < -1e-6 and np.isfinite(s.rel) and s.rel > TARGET_REL:
            need = max(1, int(np.ceil(s.runs_now * (TARGET_REL / s.rel) ** (1.0 / slope))) - s.runs_now)
        advice = f"+{need} runs of {_source_label(s.sources)}" if need else "hold"
        if s.runs_now <= 1:
            advice += " (one run means no cross-run check exists)"
        rows.append(
            [
                f"<code>{esc(s.name)}</code>",
                fmt(s.value, 4),
                "n/a" if not np.isfinite(s.rel) else f"{s.rel:.0%}",
                chip(s.status, s.level) + (f" {esc(s.note)}" if s.note else ""),
                esc(_source_label(s.sources)),
                f"{s.runs_now} runs, {s.valid_s_now:.0f} s",
                esc(advice) + f' <span class="chip">{basis}</span>',
            ]
        )
        cls.append(s.level)
    if not rows:
        sec.add(
            '<p class="note">Every model parameter is measured to better than '
            f"{TARGET_REL:.0%}. More data of the same kind will not move them.</p>"
        )
    else:
        sec.add(
            table_html(
                ["parameter", "value", "rel. spread", "status", "constrained by", "data now",
                 "collect"],
                rows,
                row_cls=cls,
                numeric=(1, 2),
            )
        )
    return sec


# ---------------------------------------------------------------------------
# 1-3. Inventory, quality, parameters
# ---------------------------------------------------------------------------


def section_inventory(loaded: Loaded, window_counts: dict[str, int]) -> Section:
    sec = Section("inventory", "1. Data inventory")
    sec.body = (
        '<p class="note">Usable seconds, not run count, is what a fit actually consumes: a '
        "run that was mostly encoder slip or mostly uncommanded contributes far less than its "
        "length suggests.</p>"
    )
    rows = []
    for name, runs in sorted(loaded.by_waveform().items()):
        first = runs[0]
        rows.append(
            [
                esc(name),
                esc(first.kind),
                esc(first.channel),
                esc(first.role),
                len(runs),
                f"{sum(r.quality.duration_s for r in runs):.0f}",
                f"{sum(r.quality.commanded_s for r in runs):.0f}",
                f"{sum(r.quality.valid_s for r in runs):.0f}",
                window_counts.get(name, 0),
            ]
        )
    sec.add(
        table_html(
            ["waveform", "kind", "channel", "role", "runs", "duration s", "commanded s",
             "valid s", "windows"],
            rows,
            numeric=(4, 5, 6, 7, 8),
        )
    )
    return sec


def section_quality(loaded: Loaded) -> Section:
    sec = Section("quality", "2. Run quality")
    rows, cls = [], []
    for run in loaded.runs:
        q = run.quality
        problems = q.problems()
        chips = "".join(chip(p, "bad") for p in problems) or chip("clean", "ok")
        rows.append(
            [
                esc(run.name),
                esc(q.verdict or "-"),
                f"{q.duration_s:.0f}",
                q.dropped if q.dropped is not None else "-",
                q.malformed_rows,
                f"{q.worst_saturation:.2%}",
                fmt(q.bias_drift_dps, 4),
                f"{q.slip_fraction:.1%}",
                fmt(q.clock_residual_ms, 2),
                f"{q.valid_s:.1f}",
                esc(q.command_source),
                chips,
            ]
        )
        cls.append("bad" if problems else "ok")
    sec.add(
        table_html(
            ["run", "verdict", "duration s", "dropped", "malformed", "saturation",
             "bias drift", "slip", "clock resid", "valid s", "commands", "gates"],
            rows,
            row_cls=cls,
            numeric=(2, 3, 4, 5, 6, 7, 8, 9),
        )
    )
    return sec


def section_params(stage, params: PlantParams, statuses: Sequence[ParamStatus]) -> Section:
    sec = Section("params", "3. Fitted parameters")
    sec.body = (
        '<p class="note">Stage A is the per-phase fit, in near-closed form. Final is after the '
        "joint fit against multi-step open-loop prediction error, which is the quantity the "
        "filter actually depends on.</p>"
    )
    by_name = {s.name: s for s in statuses}
    rows, cls = [], []
    for name, value in params.to_dict().items():
        est = stage.estimates.get(name)
        st = by_name.get(name)
        rows.append(
            [
                f"<code>{esc(name)}</code>",
                fmt(est.value, 4) if est else "-",
                fmt(est.spread, 4) if est and est.n else "-",
                est.n if est else "-",
                fmt(value, 4),
                chip(st.status, st.level) if st else "",
                esc(est.note) if est and est.note else "",
            ]
        )
        cls.append(st.level if st else "")
    sec.add(
        table_html(
            ["parameter", "stage A", "spread", "n", "final", "status", "note"],
            rows,
            row_cls=cls,
            numeric=(1, 2, 3, 4),
        )
    )
    return sec


# ---------------------------------------------------------------------------
# 4. Stage A evidence
# ---------------------------------------------------------------------------


def section_evidence(stage, params: PlantParams, profile) -> Section:
    sec = Section("evidence", "4. Evidence behind the fit")

    # Gain scatters, each with the residual from its own fitted line underneath. A number
    # alone cannot show a scatter that curves; a residual panel can.
    for segs, label, gains in (
        (stage.lin_segments, "linear", (("forward", params.k_fwd, 1), ("reverse", params.k_rev, -1))),
        (stage.ang_segments, "angular", (("left", params.k_ang, 1), ("right", params.k_ang, -1))),
    ):
        pts = [(s.u_eff, s.v_ss) for s in segs if np.isfinite(s.u_eff)]
        if not pts:
            continue
        fig, (ax, axr) = plt.subplots(
            2, 1, figsize=(7, 4.6), sharex=True, gridspec_kw={"height_ratios": [2.4, 1]}
        )
        for name, k, sign in gains:
            sel = [(abs(u), v) for u, v in pts if np.sign(u) == sign]
            if not sel:
                continue
            x = np.array([u for u, _ in sel])
            y = np.array([v for _, v in sel])
            colour = "C0" if sign > 0 else "C1"
            ax.plot(x, y, "o", ms=4, color=colour, label=f"{name} (n={len(x)})")
            xs = np.linspace(0, max(x.max(), 1e-6), 20)
            ax.plot(xs, k * xs, "-", lw=1, color=colour)
            axr.plot(x, y - k * x, "o", ms=3, color=colour)
        axr.axhline(0.0, color="0.5", lw=1)
        ax.set_ylabel("steady state", fontsize=9)
        ax.legend(fontsize=8)
        axr.set_ylabel("residual", fontsize=9)
        axr.set_xlabel("effective command", fontsize=9)
        sec.add(
            figure_html(
                fig,
                f"{label} gain. Curvature in the residual panel means the gain is not a "
                "constant over the commanded range, which no amount of extra data fixes.",
            )
        )

    # Rise tau against level. A tau that trends with amplitude means the first-order lag is
    # the wrong model shape, which is the cheapest structural check available.
    for segs, label in ((stage.lin_segments, "linear"), (stage.ang_segments, "angular")):
        pts = [(abs(s.level), s.tau) for s in segs if np.isfinite(s.tau)]
        if len(pts) < 3:
            continue
        fig, ax = plt.subplots(figsize=(6, 2.8))
        ax.plot([p[0] for p in pts], [p[1] for p in pts], "o", ms=4)
        ax.set_xlabel("|command|", fontsize=9)
        ax.set_ylabel("rise tau (s)", fontsize=9)
        sec.add(
            figure_html(
                fig,
                f"{label} rise constant against command level. A trend here says the "
                "first-order lag is the wrong shape, not that tau needs more samples.",
            )
        )

    # Straight-line drift: the evidence for the arc, and which of the two forms fits it.
    if stage.drift_points:
        x = np.array([u for u, _ in stage.drift_points])
        y = np.array([w for _, w in stage.drift_points])
        fig, ax = plt.subplots(figsize=(6.5, 3.2))
        ax.plot(x, y, "o", ms=5, color="C3")
        xs = np.linspace(x.min(), x.max(), 100)
        ax.plot(
            xs,
            params.c_drift * xs + params.c_drift_bias * np.sign(xs),
            "-",
            lw=1.2,
            color="C0",
            label=f"c_drift {params.c_drift:.3f}, bias {params.c_drift_bias:.3f}",
        )
        ax.axhline(0.0, color="0.6", lw=1)
        ax.axvline(0.0, color="0.6", lw=1)
        ax.set_xlabel("effective linear command", fontsize=9)
        ax.set_ylabel("steady yaw rate (rad/s)", fontsize=9)
        ax.legend(fontsize=8)
        sec.add(
            figure_html(
                fig,
                "Straight-line drift: yaw produced by a pure forward command. A line through "
                "the origin is a gain mismatch; a step across zero is an asymmetric drag "
                "torque. Both directions must be present or the two are indistinguishable.",
            )
        )

    # Coupling grid, with the mirror structure visible. Empty mirrors are the finding.
    if stage.coupling_cells:
        cells = [(a, b, v, w) for a, b, v, w in stage.coupling_cells if abs(b) > 1e-3]
        if cells:
            fig, ax = plt.subplots(figsize=(6.5, 3.4))
            xs = [c[0] for c in cells]
            ys = [c[1] for c in cells]
            sc = ax.scatter(xs, ys, c=[c[3] for c in cells], cmap="coolwarm", s=46)
            fig.colorbar(sc, ax=ax, label="yaw rate (rad/s)")
            ax.axhline(0.0, color="0.4", lw=1)
            ax.set_xlabel("effective linear command", fontsize=9)
            ax.set_ylabel("effective angular command", fontsize=9)
            has_negative = any(c[1] < -1e-3 for c in cells)
            caption = (
                "Coupling grid. Cells must come in mirrored pairs across the horizontal axis: "
                "angular droop flips with the turn direction and straight-line drift does not, "
                "so an unmirrored grid cannot separate them."
            )
            if not has_negative:
                caption += " THIS GRID HAS NO NEGATIVE ANGULAR CELLS, so c_ad is not identified."
            sec.add(figure_html(fig, caption))

    # Delay: the onset stack and its profile likelihood.
    delay = stage.delay
    if delay is not None and len(getattr(delay, "lags_ms", [])):
        fig, ax = plt.subplots(figsize=(6.5, 3.0))
        ax.plot(delay.lags_ms, delay.stack, color="0.7", lw=1, label="stack")
        ax.plot(delay.lags_ms, delay.smooth, color="C0", lw=1.5, label="smoothed")
        if np.isfinite(delay.lag_ms):
            ax.axvline(delay.lag_ms, color="m", ls="--", lw=1, label=f"{delay.lag_ms:.0f} ms")
        ax.set_xlabel("lag from command edge (ms)", fontsize=9)
        ax.set_ylabel("normalized accel", fontsize=9)
        ax.legend(fontsize=8)
        note = f"{delay.count} edges, SNR {delay.snr:.1f}."
        if not delay.clock_measured:
            note += (
                " The clock was never probed, so this is offset plus delay, not delay."
            )
        sec.add(figure_html(fig, "Transport delay onset stack. " + note))

    if profile is not None:
        fig, ax = plt.subplots(figsize=(6.5, 3.0))
        ax.plot(profile.delays * 1e3, profile.costs, "C0-o", ms=3)
        ax.axvline(profile.best_delay * 1e3, color="m", ls="--", lw=1)
        ax.set_xlabel("transport delay (ms)", fontsize=9)
        ax.set_ylabel("window fit cost", fontsize=9)
        span = float(profile.costs.max() - profile.costs.min())
        flat = span < 1e-9 or (profile.costs.min() / max(profile.costs.max(), 1e-12)) > 0.999
        caption = "Delay profile likelihood, refitting every other parameter at each grid point."
        if flat:
            caption += (
                " The curve is flat, so the delay is not identifiable from this excitation. "
                "That calls for high-frequency content: a chirp, or a shorter PRBS bit period."
            )
        sec.add(figure_html(fig, caption))
    return sec


# ---------------------------------------------------------------------------
# 5. Convergence
# ---------------------------------------------------------------------------


def jackknife(
    loaded: Loaded,
    start: PlantParams,
    structure: ModelStructure,
    weights: FitWeights,
    horizons: Sequence[float],
    *,
    stride_s: float,
    max_windows: int,
) -> tuple[list[str], dict[str, np.ndarray]]:
    """Refit with each run dropped in turn.

    A parameter that moves more than its own spread when one run is dropped rests on that
    single run. That is the sharpest possible "collect more of this", and unlike the
    learning curve it costs one short refit per run rather than dozens.
    """
    names = structure.free_names()
    shifts = {n: [] for n in names}
    labels: list[str] = []
    base = start.to_dict()
    for i, run in enumerate(loaded.runs):
        train, _ = loaded.split_run(i)
        if len(train) < 2:
            continue
        ws = build_windows(train, window_delay(start, structure), horizons, stride_s, max_windows)
        if ws is None:
            continue
        try:
            fitted, _ = joint_fit(ws, start, structure, weights, max_nfev=30)
        except Exception:
            continue
        labels.append(run.name)
        for n in names:
            ref = abs(base.get(n, 0.0)) or 1.0
            shifts[n].append(abs(getattr(fitted, n) - base.get(n, 0.0)) / ref)
    return labels, {n: np.array(v) for n, v in shifts.items()}


def learning_curve(
    runs: Sequence[Run],
    start: PlantParams,
    structure: ModelStructure,
    weights: FitWeights,
    horizons: Sequence[float],
    *,
    stride_s: float,
    max_windows: int,
    n_boot: int,
    seed: int = 12345,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Parameter spread against the number of runs included, by bootstrap over runs.

    The point is the slope, not any single value. A curve still falling at the right edge
    means more runs will keep helping; one that has flattened means the limit is the
    excitation, not the sample count, and more of the same will not move it.
    """
    names = structure.free_names()
    rng = np.random.default_rng(seed)
    counts = [c for c in (2, 3, 5, 8, 12, 20) if c <= len(runs)]
    if not counts:
        return np.array([]), {}
    values = {n: np.full((len(counts), n_boot), np.nan) for n in names}
    for ci, count in enumerate(counts):
        for b in range(n_boot):
            pick = rng.choice(len(runs), size=count, replace=False)
            subset = [runs[i] for i in pick]
            ws = build_windows(
                subset, window_delay(start, structure), horizons, stride_s, max_windows
            )
            if ws is None:
                continue
            try:
                fitted, _ = joint_fit(ws, start, structure, weights, max_nfev=30)
            except Exception:
                continue
            for n in names:
                values[n][ci, b] = getattr(fitted, n)
    return np.array(counts), values


def section_convergence(
    labels: Sequence[str],
    shifts: dict[str, np.ndarray],
    counts: np.ndarray,
    curves: dict[str, np.ndarray],
    statuses: Sequence[ParamStatus],
) -> tuple[Section, dict[str, float]]:
    sec = Section("convergence", "5. How much more data")
    slopes: dict[str, float] = {}

    if labels and shifts:
        names = [n for n in shifts if len(shifts[n])]
        matrix = np.array([shifts[n] for n in names])
        fig, ax = plt.subplots(figsize=(min(12, 2 + 0.5 * len(labels)), 0.35 * len(names) + 1.6))
        im = ax.imshow(matrix, aspect="auto", cmap="magma_r")
        ax.set_yticks(range(len(names)))
        ax.set_yticklabels(names, fontsize=8)
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels([label.split("/")[-1] for label in labels], rotation=90, fontsize=7)
        fig.colorbar(im, ax=ax, label="fractional shift when dropped")
        sec.add(
            figure_html(
                fig,
                "Leave-one-run-out influence. A bright cell means that parameter rests on that "
                "one run: drop it and the fitted value moves by a large fraction of itself. "
                "Those are the runs to repeat first.",
            )
        )
        worst = sorted(
            ((float(np.max(v)), n, labels[int(np.argmax(v))]) for n, v in shifts.items() if len(v)),
            reverse=True,
        )[:6]
        rows = [[f"<code>{esc(n)}</code>", f"{s:.1%}", esc(run)] for s, n, run in worst]
        sec.add(table_html(["parameter", "largest shift", "when dropping"], rows, numeric=(1,)))

    if len(counts) and curves:
        fig, ax = plt.subplots(figsize=(7, 4))
        for name, values in curves.items():
            spread = np.nanstd(values, axis=1)
            centre = np.nanmedian(values, axis=1)
            rel = spread / np.maximum(np.abs(centre), 1e-9)
            good = np.isfinite(rel) & (rel > 0)
            if np.count_nonzero(good) < 2:
                continue
            ax.loglog(counts[good], rel[good], "-o", ms=3, label=name)
            slope = np.polyfit(np.log(counts[good]), np.log(rel[good]), 1)[0]
            slopes[name] = float(slope)
        ax.axhline(TARGET_REL, color="0.5", ls=":", lw=1)
        ax.set_xlabel("runs included", fontsize=9)
        ax.set_ylabel("relative spread", fontsize=9)
        ax.legend(fontsize=7, ncol=2)
        sec.add(
            figure_html(
                fig,
                "Learning curve. Ideal sampling gives a slope of -0.5. A curve that has "
                "flattened well above the dotted target is limited by the excitation rather "
                "than the run count, so more of the same waveform will not help.",
            )
        )
        rows = [
            [f"<code>{esc(n)}</code>", f"{s:+.2f}",
             "sampling-limited" if s < -0.3 else "excitation-limited"]
            for n, s in sorted(slopes.items(), key=lambda kv: kv[1])
        ]
        sec.add(table_html(["parameter", "log-log slope", "reading"], rows, numeric=(1,)))
    else:
        sec.add(
            '<p class="note">Learning curve not run. Pass <code>--bootstrap 8</code> to '
            "measure the slope instead of assuming the root-N law.</p>"
        )
    return sec, slopes


# ---------------------------------------------------------------------------
# 6. Input coverage
# ---------------------------------------------------------------------------


def section_coverage(loaded: Loaded, params: PlantParams) -> Section:
    sec = Section("coverage", "6. Input coverage")
    sec.body = (
        '<p class="note">What of the input space has actually been driven. A gap here is a '
        "waveform to add, not more repetitions of one already run.</p>"
    )
    runs = loaded.runs
    if not runs:
        return sec

    lin = np.concatenate([r.cmd_lin[r.commanded & ~r.slip] for r in runs]) if runs else np.array([])
    ang = np.concatenate([r.cmd_ang[r.commanded & ~r.slip] for r in runs]) if runs else np.array([])
    dt = float(np.median([r.dt for r in runs]))

    if len(lin) > 10:
        lin_eff = effective_command(lin, params.dz_lin_fwd, params.dz_lin_rev)
        ang_eff = effective_command(ang, params.dz_ang_l, params.dz_ang_r)
        edges = np.linspace(-1, 1, 12)
        grid, _, _ = np.histogram2d(lin_eff, ang_eff, bins=[edges, edges])
        grid *= dt  # seconds held, which is what the fit weighs by
        fig, ax = plt.subplots(figsize=(5.6, 4.4))
        masked = np.ma.masked_where(grid <= 0, grid)
        cmap = plt.get_cmap("viridis").copy()
        cmap.set_bad("#d8d8d8")
        im = ax.pcolormesh(edges, edges, masked.T, cmap=cmap)
        fig.colorbar(im, ax=ax, label="seconds held")
        ax.set_xlabel("effective linear command", fontsize=9)
        ax.set_ylabel("effective angular command", fontsize=9)
        empty = int(np.count_nonzero(grid <= 0))
        sec.add(
            figure_html(
                fig,
                f"Command occupancy, weighted by time held. {empty} of {grid.size} cells are "
                "empty (grey). Empty cells off the axes are what a coupling grid fills.",
            )
        )

    # Command spectrum against the model corner. This is the view that most often explains a
    # weak time constant: a step battery has almost no energy where the plant rolls off.
    try:
        from scipy.signal import welch

        fig, ax = plt.subplots(figsize=(6.8, 3.2))
        drawn = False
        for label, getter in (("linear", lambda r: r.cmd_lin), ("angular", lambda r: r.cmd_ang)):
            series = [getter(r) for r in runs if len(getter(r)) > 64]
            if not series:
                continue
            stacked = np.concatenate(series)
            f, pxx = welch(stacked, fs=1.0 / dt, nperseg=min(1024, len(stacked)))
            keep = f > 0
            ax.loglog(f[keep], pxx[keep], label=label)
            drawn = True
        if drawn:
            for tau, name in (
                (params.tau_lin_a, "linear corner"),
                (params.tau_ang_a, "angular corner"),
            ):
                if tau > 1e-6:
                    ax.axvline(1.0 / (2 * np.pi * tau), ls="--", lw=1, color="0.4")
                    ax.annotate(
                        f"{name} {1.0 / (2 * np.pi * tau):.1f} Hz",
                        (1.0 / (2 * np.pi * tau), ax.get_ylim()[1]),
                        fontsize=7,
                        rotation=90,
                        va="top",
                    )
            ax.set_xlabel("frequency (Hz)", fontsize=9)
            ax.set_ylabel("command PSD", fontsize=9)
            ax.legend(fontsize=8)
            sec.add(
                figure_html(
                    fig,
                    "Command power spectrum against the fitted model corners. If the "
                    "excitation dies before the corner, the time constants are extrapolated "
                    "rather than measured, and a chirp or a shorter PRBS bit period is the fix.",
                )
            )
        else:
            plt.close(fig)
    except Exception:
        pass

    # Declared amplitudes that were never actually reached.
    rows = []
    for name, waveform_runs in sorted(loaded.by_waveform().items()):
        spec = waveform_runs[0].spec
        declared = spec.amplitudes
        if not declared:
            continue
        reached = np.concatenate(
            [np.abs(r.cmd_lin if spec.channel != "angular" else r.cmd_ang) for r in waveform_runs]
        )
        missing = [a for a in declared if not np.any(np.abs(reached - abs(a)) < 0.02)]
        rows.append(
            [
                esc(name),
                ", ".join(f"{a:g}" for a in declared),
                (
                    chip("all reached", "ok")
                    if not missing
                    else chip("never reached: " + ", ".join(f"{a:g}" for a in missing), "warn")
                ),
            ]
        )
    if rows:
        sec.add(table_html(["waveform", "declared amplitudes", "coverage"], rows))
    return sec


# ---------------------------------------------------------------------------
# 7-8. Horizon quality and residual diagnostics
# ---------------------------------------------------------------------------


def section_horizons(reports: dict) -> Section:
    sec = Section("horizons", "7. Prediction quality against horizon")
    for label, report in reports.items():
        rows = []
        for i, h in enumerate(report.horizons):
            rows.append(
                [
                    f"{h * 1e3:.0f}",
                    *[f"{report.rows[k][i]:.2f}" for k in report.rows],
                ]
            )
        sec.add(f"<h3>{esc(label)}</h3>")
        sec.add(
            table_html(
                ["horizon ms", *report.rows.keys()],
                rows,
                numeric=tuple(range(len(report.rows) + 1)),
            )
        )

    if reports:
        fig, ax = plt.subplots(figsize=(6.8, 3.4))
        for label, report in reports.items():
            ax.plot(report.horizons * 1e3, report.rows["pos_rmse_mm"], "-o", ms=3, label=label)
        ax.axhline(15.0, color="0.5", ls=":", lw=1)
        ax.axhline(80.0, color="0.5", ls=":", lw=1)
        ax.set_xlabel("horizon (ms)", fontsize=9)
        ax.set_ylabel("position RMSE (mm)", fontsize=9)
        ax.legend(fontsize=8)
        sec.add(
            figure_html(
                fig,
                "Open-loop position error against horizon. The dotted lines are the acceptance "
                "criteria: 15 mm at 100 ms and 80 mm at 400 ms.",
            )
        )
    return sec


def residual_acf(err: WindowErrors, max_lag: int = 40, horizon_index: int = -1) -> np.ndarray:
    """Autocorrelation of the position residual across window starts.

    The scalar lag-1 version answers "is there structure"; the curve answers "how much and
    over what scale". Structure lasting tens of lags is a missing model term, and no amount
    of process-noise tuning covers that honestly.
    """
    series = err.position[:, horizon_index]
    series = series - series.mean()
    denom = float(series @ series)
    if denom <= 0:
        return np.zeros(max_lag + 1)
    return np.array(
        [float(series[: len(series) - k] @ series[k:]) / denom for k in range(max_lag + 1)]
    )


def section_residuals(err: WindowErrors | None, runs: Sequence[Run]) -> Section:
    sec = Section("residuals", "8. Residual diagnostics")
    if err is None:
        sec.add('<p class="note">No holdout windows, so nothing to diagnose.</p>')
        return sec
    sec.body = (
        '<p class="note">A residual that varies with a conditioning variable names the term '
        "the model is missing. A fitted slope larger than twice its standard error is flagged."
        "</p>"
    )

    acf = residual_acf(err)
    fig, ax = plt.subplots(figsize=(6.8, 2.8))
    ax.bar(range(len(acf)), acf, width=0.8)
    band = 2.0 / np.sqrt(max(len(err.starts), 1))
    ax.axhline(band, color="C3", ls="--", lw=1)
    ax.axhline(-band, color="C3", ls="--", lw=1)
    ax.set_xlabel("lag (windows)", fontsize=9)
    ax.set_ylabel("autocorrelation", fontsize=9)
    sec.add(
        figure_html(
            fig,
            "Residual autocorrelation at the longest horizon, with the white-noise band. "
            "Staying outside the band for many lags is structure, not noise.",
        )
    )

    panels = [
        ("speed at window start (m/s)", err.speed, "along", "gain error or missing drag"),
        ("|angular command|", np.abs(err.u_ang0), "along", "steer-brake coupling"),
        ("|linear command|", np.abs(err.u_lin0), "heading", "angular droop"),
        ("yaw rate at window start (rad/s)", err.yaw_rate, "cross", "angular lag"),
    ]
    findings = []
    fig, axes = plt.subplots(2, 2, figsize=(10, 5.6))
    for ax, (xlabel, x, channel, term) in zip(axes.ravel(), panels):
        y = getattr(err, channel)[:, -1] * (1e3 if channel != "heading" else 180 / np.pi)
        if x is None or len(x) != len(y):
            ax.axis("off")
            continue
        ax.plot(x, y, ".", ms=2, alpha=0.35, color="0.5")
        bins = np.linspace(float(np.min(x)), float(np.max(x)), 9)
        idx = np.clip(np.digitize(x, bins) - 1, 0, len(bins) - 2)
        centres, means, errs = [], [], []
        for b in range(len(bins) - 1):
            sel = idx == b
            if np.count_nonzero(sel) < 5:
                continue
            centres.append(0.5 * (bins[b] + bins[b + 1]))
            means.append(float(np.mean(y[sel])))
            errs.append(float(np.std(y[sel]) / np.sqrt(np.count_nonzero(sel))))
        if len(centres) >= 3:
            ax.errorbar(centres, means, yerr=errs, fmt="o-", ms=4, color="C0", lw=1.2)
            slope, _ = np.polyfit(np.array(centres), np.array(means), 1)
            resid = np.array(means) - np.polyval([slope, np.mean(means)], np.array(centres))
            se = float(np.std(resid) / max(np.std(centres), 1e-9) / np.sqrt(len(centres)))
            if np.isfinite(se) and abs(slope) > 2 * se:
                findings.append(f"{channel} vs {xlabel}: slope {slope:.3g} ({term})")
        ax.axhline(0.0, color="0.4", lw=1)
        ax.set_xlabel(xlabel, fontsize=8)
        ax.set_ylabel(f"{channel} error ({'deg' if channel == 'heading' else 'mm'})", fontsize=8)
    fig.tight_layout()
    sec.add(figure_html(fig, "Residual against each conditioning variable, binned."))
    if findings:
        sec.add("<p>" + "".join(chip(f, "warn") for f in findings) + "</p>")
    else:
        sec.add("<p>" + chip("no conditioning variable shows a significant slope", "ok") + "</p>")

    # The panel that adjudicates the drift model: split by turn direction. A slope common to
    # both directions is unabsorbed drift; one that flips with direction is angular droop.
    if err.u_ang0 is not None and err.u_lin0 is not None:
        fig, ax = plt.subplots(figsize=(6.8, 3.2))
        for sign, label, colour in ((1, "turning left", "C0"), (-1, "turning right", "C1")):
            sel = np.sign(err.u_ang0) == sign
            if np.count_nonzero(sel) < 20:
                continue
            x = err.u_lin0[sel]
            y = err.heading[sel, -1] * 180 / np.pi
            order = np.argsort(x)
            ax.plot(x[order], y[order], ".", ms=2, alpha=0.3, color=colour)
            if len(x) > 5:
                coef = np.polyfit(x, y, 1)
                xs = np.linspace(x.min(), x.max(), 20)
                ax.plot(xs, np.polyval(coef, xs), "-", lw=1.6, color=colour, label=label)
        ax.axhline(0.0, color="0.4", lw=1)
        ax.set_xlabel("signed linear command at window start", fontsize=9)
        ax.set_ylabel("heading error (deg)", fontsize=9)
        ax.legend(fontsize=8)
        sec.add(
            figure_html(
                fig,
                "Heading residual against the linear command, split by turn direction. Two "
                "parallel lines means drift the model has not absorbed; lines that diverge "
                "with the turn direction means angular droop. This is the fastest read on "
                "which drift rung should ship.",
            )
        )

    # Which waveform the residuals come from. The single most decision-relevant plot here.
    if err.origin is not None and len(runs):
        labels, values = [], []
        for i, run in enumerate(runs):
            sel = err.origin == i
            if np.count_nonzero(sel) < 5:
                continue
            labels.append(f"{run.waveform}#{run.record.rep}")
            values.append(float(np.sqrt(np.mean(err.position[sel, -1] ** 2)) * 1e3))
        if labels:
            fig, ax = plt.subplots(figsize=(min(11, 2 + 0.45 * len(labels)), 3.2))
            ax.bar(range(len(labels)), values, color="C0")
            ax.set_xticks(range(len(labels)))
            ax.set_xticklabels(labels, rotation=60, ha="right", fontsize=7)
            ax.set_ylabel("500 ms position RMSE (mm)", fontsize=9)
            sec.add(
                figure_html(
                    fig,
                    "Residual per run. A run far above the rest is either bad data or the "
                    "regime the model misses, and the two are told apart by whether its "
                    "siblings of the same waveform are also bad.",
                )
            )
    return sec


# ---------------------------------------------------------------------------
# 9-10. Per run, excluded, provenance
# ---------------------------------------------------------------------------


def run_trace_figure(run: Run, params: PlantParams, structure: ModelStructure, reinit_s: float):
    """Commands, measured against predicted, and the residual.

    Built on the same windowing the fit uses rather than a private integrator: a bespoke
    plotting path that disagreed with the fit would be worse than no plot at all. The model
    is reinitialized from truth every `reinit_s`, so a 30 s run stays readable instead of
    diverging in the first second and telling you nothing after it.
    """
    ws = build_windows([run], window_delay(params, structure), (reinit_s,), reinit_s, 4000)
    if ws is None:
        return None
    err = predict_windows(ws, params, structure)
    starts = ws.starts
    t = run.t - run.t[0]

    fig, axes = plt.subplots(4, 1, figsize=(10, 7), sharex=True,
                             gridspec_kw={"height_ratios": [1, 1.4, 1.4, 1]})
    ax = axes[0]
    ax.plot(t, run.cmd_lin, lw=1, label="linear")
    ax.plot(t, run.cmd_ang, lw=1, label="angular")
    for seg in run.record.segments or []:
        ax.axvspan(seg.t0, seg.t1, color="0.9", zorder=0)
    ax.set_ylabel("command", fontsize=8)
    ax.legend(fontsize=7, ncol=2)

    axes[1].plot(t, run.v, lw=1, color="0.3", label="measured")
    axes[1].set_ylabel("v (m/s)", fontsize=8)
    axes[2].plot(t, run.w, lw=1, color="0.3", label="measured")
    axes[2].set_ylabel("w (rad/s)", fontsize=8)
    for ax in (axes[1], axes[2]):
        ax.legend(fontsize=7)

    axes[3].plot(t[starts], err.position[:, -1] * 1e3, ".", ms=3, color="C3")
    axes[3].set_ylabel(f"|err| at {reinit_s * 1e3:.0f} ms (mm)", fontsize=8)
    axes[3].set_xlabel("time (s)", fontsize=9)

    # Grey out what the fit ignores, so a run that looks bad because it was excluded is
    # visibly different from one that is bad because the model is wrong.
    bad = run.slip | ~run.commanded
    if np.any(bad):
        edges = np.flatnonzero(np.diff(bad.astype(int)) != 0) + 1
        bounds = np.concatenate([[0], edges, [len(bad)]])
        for lo, hi in zip(bounds[:-1], bounds[1:]):
            if bad[lo]:
                for ax in axes:
                    ax.axvspan(t[lo], t[min(hi, len(t) - 1)], color="#f2d6d6", zorder=0)
    fig.tight_layout()
    return fig


def section_per_run(
    runs: Sequence[Run], params: PlantParams, structure: ModelStructure, detail: str
) -> Section:
    sec = Section("runs", "9. Per run")
    if detail == "none" or not runs:
        sec.add('<p class="note">Per-run detail disabled (<code>--detail none</code>).</p>')
        return sec

    chosen = list(runs)
    if detail == "failing":
        chosen = [r for r in runs if r.quality.problems()]
        if not chosen:
            chosen = list(runs)[:3]
        sec.body = (
            '<p class="note">Showing runs that failed a gate, plus a sample. Pass '
            "<code>--detail all</code> for every run; the file grows quickly.</p>"
        )
    for run in chosen:
        fig = run_trace_figure(run, params, structure, 0.5)
        if fig is None:
            continue
        problems = run.quality.problems()
        chips = "".join(chip(p, "bad") for p in problems) or chip("clean", "ok")
        sec.add(
            f"<details><summary>{esc(run.name)} {chips}</summary>"
            + figure_html(fig, "Model reinitialized from truth every 500 ms. Red shading is "
                               "encoder slip or uncommanded time, which the fit ignores.")
            + "</details>"
        )
    return sec


def section_tail(loaded: Loaded, params: PlantParams, extra: Sequence[str]) -> Section:
    sec = Section("provenance", "10. Excluded runs and provenance")
    if loaded.excluded:
        sec.add("<h3>Excluded</h3>")
        sec.add(
            table_html(
                ["run", "why"], [[esc(name), esc(why)] for name, why in loaded.excluded]
            )
        )
    for line in extra:
        sec.add(f'<p class="note">{esc(line)}</p>')
    sec.add("<h3>Sessions</h3>")
    sec.add(
        table_html(
            ["session", "name", "robot", "operator", "floor", "runs"],
            [
                [esc(s.session_id), esc(s.name), esc(s.robot), esc(s.operator),
                 esc(s.floor_surface), len(s.runs)]
                for s in loaded.sessions
            ],
            numeric=(5,),
        )
    )
    sec.add("<h3>Fitted parameters</h3>")
    sec.add(f"<pre>{esc(params.to_toml('plant'))}</pre>")
    return sec


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def render_report(
    path: Path,
    *,
    loaded: Loaded,
    stage,
    params: PlantParams,
    structure: ModelStructure,
    profile,
    reports: dict,
    fit_runs: Sequence[Run],
    test_runs: Sequence[Run],
    horizons: Sequence[float],
    stride_s: float,
    max_windows: int,
    weights: FitWeights,
    detail: str = "failing",
    bootstrap: int = 0,
    title: str = "",
    track_width: float = 0.10,
) -> None:
    path = Path(path)

    window_counts: dict[str, int] = {}
    for run in loaded.runs:
        ws = build_windows([run], window_delay(params, structure), horizons, stride_s, max_windows)
        window_counts[run.waveform] = window_counts.get(run.waveform, 0) + (
            ws.count() if ws is not None else 0
        )

    statuses = param_status(loaded, stage)

    labels, shifts = jackknife(
        loaded, params, structure, weights, horizons,
        stride_s=stride_s, max_windows=max_windows,
    )
    counts, curves = (np.array([]), {})
    if bootstrap > 0 and len(fit_runs) >= 2:
        counts, curves = learning_curve(
            fit_runs, params, structure, weights, horizons,
            stride_s=stride_s, max_windows=max_windows, n_boot=bootstrap,
        )
    convergence, slopes = section_convergence(labels, shifts, counts, curves, statuses)

    holdout_err = None
    scored = test_runs or fit_runs
    ws = build_windows(scored, window_delay(params, structure), horizons, stride_s, max_windows)
    if ws is not None:
        holdout_err = predict_windows(ws, params, structure)

    extra: list[str] = []
    if not test_runs:
        extra.append(
            "No holdout runs. Residual diagnostics are computed on the training runs, which "
            "flatters the model: record a waveform with role = \"holdout\"."
        )

    sections = [
        section_next(statuses, slopes),
        section_inventory(loaded, window_counts),
        section_quality(loaded),
        section_params(stage, params, statuses),
        section_evidence(stage, params, profile),
        convergence,
        section_coverage(loaded, params),
        section_horizons(reports),
        section_residuals(holdout_err, scored),
        section_per_run(loaded.runs, params, structure, detail),
        section_tail(loaded, params, extra),
    ]
    lede = (
        f"{len(loaded.runs)} runs across {len(loaded.sessions)} session(s), model "
        f"{structure.name}, {len(fit_runs)} train / {len(test_runs)} holdout."
    )
    render_page(path, title or "Velocity jig plant fit", sections, lede)
