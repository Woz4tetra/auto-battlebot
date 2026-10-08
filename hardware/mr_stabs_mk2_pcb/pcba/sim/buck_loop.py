"""TPS54202 loop gain: crossover and phase margin, with and without the feed-forward cap.

    ~/.local/share/pcba-design/venv/bin/python sim/buck_loop.py   -> sim/buck_loop.{png,json}

TI does not publish the internal compensation, so the model is built from what the datasheet
does give:
- Crossover without C6, equation 14 (p.16): fo = 3.95 / (VOUT x COUT), a -1 slope through fo.
- C6 across R2 adds a zero at 1 / (2 pi R2 C6) and a pole at 1 / (2 pi (R2 || R3) C6).
- Peak current mode's sampling double pole at fsw / 2 (500 kHz typical, p.5), Q = 2 / pi.
Effective COUT is the board's three 22 uF 25 V X5R 0805 after DC bias at 5.6 V (about 10 uF each,
from datasheets.json U2), the previous two, and TI's 44 uF for comparison. This is a small-signal
estimate, not TI's transistor model: it says whether the margin is comfortable, not its exact value.
"""

import json
import math
import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
VOUT, R2, R3, FSW = 5.56, 100e3, 12e3, 500e3
f = np.logspace(2, 6.5, 4000)
s = 2j * math.pi * f


def loop(cout, c6):
    fo = 3.95 / (VOUT * cout)
    t = (2 * math.pi * fo) / s
    if c6:
        wz, wp = 1 / (R2 * c6), 1 / ((R2 * R3 / (R2 + R3)) * c6)
        t = t * (1 + s / wz) / (1 + s / wp)
    wn, q = math.pi * FSW, 2 / math.pi
    return t / (1 + s / (q * wn) + (s / wn) ** 2), fo


results = []
fig, (ag, ap) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
for cout, c6, label in (
    (20e-6, 47e-12, "2 x 22 uF (20 eff), 47 pF: previous"),
    (30e-6, 47e-12, "3 x 22 uF (30 eff), 47 pF: board"),
    (30e-6, 0, "3 x 22 uF, no C6"),
    (44e-6, 75e-12, "44 uF, 75 pF: TI table 7-2"),
):
    t, fo = loop(cout, c6)
    mag = 20 * np.log10(abs(t))
    ph = np.degrees(np.unwrap(np.angle(t)))
    i = np.argmax(mag < 0)
    fc, pm = f[i], 180 + ph[i]
    results.append(
        {
            "case": label,
            "fo_eq14_kHz": round(fo / 1e3, 1),
            "crossover_kHz": round(fc / 1e3, 1),
            "phase_margin_deg": round(pm, 0),
            "fc_over_fsw": round(fc / FSW, 3),
        }
    )
    ag.semilogx(f, mag, label=label)
    ap.semilogx(f, ph + 180, label=label)
ag.axhline(0, color="k", lw=0.5)
ag.set_ylabel("loop gain, dB")
ag.set_ylim(-40, 60)
ap.set_ylabel("phase + 180, deg")
ap.set_ylim(0, 120)
ap.set_xlabel("Hz")
ag.legend(fontsize=8)
ag.set_title("TPS54202 loop estimate (eq 14 crossover, C6 zero/pole, fsw/2 sampling poles)")
for a in (ag, ap):
    a.grid(True, which="both", lw=0.3)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "buck_loop.png"), dpi=110)
json.dump(results, open(os.path.join(HERE, "buck_loop.json"), "w"), indent=1)
for r in results:
    print(r)
