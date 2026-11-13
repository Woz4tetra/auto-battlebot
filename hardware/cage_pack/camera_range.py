"""Camera range check: sweeps the ZED X One S through the SmallRig 2948 ball head's whole
range and measures how close it comes to everything fixed around it.

    PYTHONPATH=$SKILL/scripts ~/.local/share/build123d-part/venv/bin/python camera_range.py

$SKILL is the build123d-part skill's directory: the part files import its recipe.py.

What moves: the camera body, the bracket, the front mount and the FAKRA plug in the
camera's back, as points sampled off their tessellated surfaces (0.5 mm tolerance), carried
by the head's pose. The GMSL cable is not swept; it hangs slack.

Three checks, from the head's range (smallrig_2948_ball_head._check_pose):
- mount: the camera and bracket spun all the way round on the stud at the default pan and
  tilt, plug off, as when screwing the camera on. Must clear.
- aim: pan and spin all the way round (15° steps), the neck tilted up to the socket's 20°
  in eight directions, plug on, keeping the poses that look into the cage: the optical
  axis within AIM_CONE of the glass normal. Must clear. (Spun to look away from the glass,
  the plug in the camera's back swings into the glass; nobody aims it there.)
- notch: the neck dropped into the notch, 15° steps to 90°, at every pan and spin, plug
  on. Reported as the deepest drop that clears at each pan; not a pass/fail check, since
  a full 90° drop folds the camera back through the wall or the glass at most pans.

Fixed: the glass, the tray and lid (one box), the shield's flange, hood and ribs, and the
three RP-SMA antennas on the +X wall (boxes round each hinge and whip). Each is a box; the
clearance is the smallest distance from any sampled point to it over all poses, negative
inside it. Exits 1 when any clearance is under MIN_CLEAR.
"""

import sys
from functools import cache

import cage_layout as cl
import numpy as np
from build123d import Location, Pos

MIN_CLEAR = 2.0  # mm
AIM_CONE = 60.0  # degrees off the glass normal (-Z)


def _matrix(loc: Location) -> np.ndarray:
    o = loc.position
    cols = [(loc * Pos(*e)).position - o for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    m = np.eye(4)
    m[:3, :3] = np.array([[c.X, c.Y, c.Z] for c in cols]).T
    m[:3, 3] = (o.X, o.Y, o.Z)
    return m


@cache
def moving_points(plug: bool = True) -> np.ndarray:
    """Sampled surface points of everything the head carries, in the head's top frame."""
    import cage_pack_assembly as a
    import connectors as cn
    from build123d import import_step

    shapes = [
        a.BRACKET_AT * a.bracket.build(a.BRP),
        a.BRACKET_AT * a.bracket.placed_front_mount(a.BRP),
        a.CAMERA_AT * import_step(str(a.bracket.camera_body_step())),
    ]
    if plug:
        shapes.append(cn.place(cn.fakra_plug(), *a.fakra_cam(), (1, 0, 0)))
    pts = []
    for s in shapes:
        verts, _ = s.tessellate(0.5)
        pts += [(v.X, v.Y, v.Z) for v in verts]
    world = np.array(pts)
    top = _matrix(a.HEAD * a.ballhead.top_location(a.BALL))
    h = np.c_[world, np.ones(len(world))]
    return (np.linalg.inv(top) @ h.T).T


def _pose(**kw):
    import cage_pack_assembly as a

    p = a.ballhead.Params(**kw)
    return p, _matrix(a.HEAD * a.ballhead.top_location(p))


def poses(mode: str):
    import cage_pack_assembly as a

    steps = range(0, 360, 15)
    if mode == "mount":
        b = a.BALL
        for spin in steps:
            yield _pose(pan=b.pan, tilt=b.tilt, tilt_dir=b.tilt_dir, spin=spin)
        return
    if mode == "aim":
        tilts = [(0.0, 0.0)] + [(20.0, d) for d in range(0, 360, 45)]
    else:
        tilts = [(t, 0.0) for t in range(15, 91, 15)]
    for pan in steps:
        for spin in steps:
            for tilt, tdir in tilts:
                if tilt == 0 and pan:  # with the neck straight, pan + spin is one rotation
                    continue
                yield _pose(pan=pan, tilt=tilt, tilt_dir=tdir, spin=spin)


def obstacles() -> dict[str, tuple[tuple, tuple]]:
    import camera_shield as sh
    import enclosure_lid

    big = 1e4
    x1 = cl.X_OUT[1]
    out = {
        "glass": ((-big, -big, -big), (big, big, 0.0)),
        "tray + lid": (
            (cl.X_OUT[0], cl.Y_OUT[0], cl.PANEL_Z0),
            (x1, cl.Y_OUT[1], enclosure_lid.LID_TOP_Z),
        ),
    }
    out.update(sh.keep_out())
    for name, (y, z) in cl.ANTENNAS.items():
        out[f"antenna {name}"] = ((x1, y - 6.0, z - 6.0), (x1 + 22.0, y + 54.0, z + 6.0))
    return out


def _clearances(w: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Smallest signed distance from the points to each box: outside > 0, inside < 0."""
    below = lo[None] - w[:, None]
    above = w[:, None] - hi[None]
    out = np.maximum(np.maximum(below, above), 0.0)
    dist_out = np.linalg.norm(out, axis=2)
    inside = np.minimum(-below, -above).min(axis=2)
    return np.where(dist_out > 0, dist_out, -inside).min(axis=0)


def _notch(clear_at: dict[tuple, bool]) -> dict[float, float]:
    """Deepest drop at each pan that clears at some spin, every shallower step clearing too."""
    notch = {}
    for pan in sorted({k[0] for k in clear_at}):
        deepest = 0.0
        for spin in sorted({k[1] for k in clear_at}):
            t = 0.0
            for tilt in range(15, 91, 15):
                if not clear_at.get((pan, spin, float(tilt)), False):
                    break
                t = float(tilt)
            deepest = max(deepest, t)
        notch[pan] = deepest
    return notch


def sweep(mode: str) -> dict:
    pts = moving_points(plug=mode != "mount")
    boxes = obstacles()
    lo = np.array([b[0] for b in boxes.values()])
    hi = np.array([b[1] for b in boxes.values()])
    best = {k: (np.inf, None) for k in boxes}
    env_lo, env_hi = np.full(3, np.inf), np.full(3, -np.inf)
    notch: dict[float, float] = {}
    clear_at: dict[tuple, bool] = {}
    for p, m in poses(mode):
        if mode != "mount" and -m[2, 1] < np.cos(np.radians(AIM_CONE)):
            continue  # bracket +Y is the optical axis; this pose looks away from the cage
        w = (m @ pts.T).T[:, :3]
        env_lo, env_hi = np.minimum(env_lo, w.min(0)), np.maximum(env_hi, w.max(0))
        d = _clearances(w, lo, hi)
        for k, v in zip(boxes, d, strict=True):
            if v < best[k][0]:
                best[k] = (float(v), p)
        if mode == "notch":
            clear_at[(p.pan, p.spin, p.tilt)] = bool(d.min() >= MIN_CLEAR)
    if mode == "notch":
        notch = _notch(clear_at)
    return {"clearance": best, "envelope": (env_lo, env_hi), "notch": notch}


def main() -> int:
    fails = 0
    for mode in ("mount", "aim"):
        r = sweep(mode)
        lo, hi = r["envelope"]
        print(f"[{mode}] swept envelope x {lo[0]:.1f}..{hi[0]:.1f}  y {lo[1]:.1f}..{hi[1]:.1f}"
              f"  z {lo[2]:.1f}..{hi[2]:.1f}")  # fmt: skip
        for k, (v, p) in r["clearance"].items():
            ok = v >= MIN_CLEAR
            fails += not ok
            pose = f"pan {p.pan:.0f} spin {p.spin:.0f} tilt {p.tilt:.0f} toward {p.tilt_dir:.0f}"
            print(f"  {'PASS' if ok else 'FAIL'}  {k:22s} {v:7.1f} mm  ({pose})")
    r = sweep("notch")
    print("[notch] deepest clear drop by pan (deg): "
          + ", ".join(f"{pan:.0f}:{t:.0f}" for pan, t in r["notch"].items()))  # fmt: skip
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
