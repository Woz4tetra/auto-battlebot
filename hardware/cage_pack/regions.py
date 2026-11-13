"""Region pick points for recipe extrudes: one point well inside each face of a 2D shape.

A recipe extrude picks sketch regions by a point inside each (recipe.py). Where crossing
profiles split a region into many cells (a lip broken by boss notches, pocket cells cut
round lands), the cells are built here in build123d and one point is taken inside each, as
far from its edges as a grid search finds.
"""

from build123d import Face, Vector


def _margin(face: Face, v: Vector) -> float:
    if face.distance_to(v) > 1e-9:
        return -1.0
    return min(e.distance_to(v) for e in face.edges())


def width(face: Face, grid: int = 24) -> float:
    """About the widest circle that fits in the face: twice the best clearance a grid finds."""
    bb = face.bounding_box()
    best = 0.0
    for i in range(grid):
        for j in range(grid):
            x = bb.min.X + (i + 0.5) * bb.size.X / grid
            y = bb.min.Y + (j + 0.5) * bb.size.Y / grid
            best = max(best, _margin(face, Vector(x, y, 0)))
    return 2 * best


def _from_edges(face: Face) -> tuple[tuple[float, float] | None, float]:
    """A thin strip (a 1.6 mm lip round a 200 mm lid) falls between grid points: step in
    from the middle of each edge, both ways, and keep the point furthest from all edges."""
    best, score = None, 0.0
    for e in face.edges():
        mid, t = e.position_at(0.5), e.tangent_at(0.5)
        nrm = Vector(-t.Y, t.X, 0)
        for k in (0.1, 0.2, 0.4, 0.8, 1.6, 3.2):
            for s in (-1, 1):
                q = mid + nrm * (s * k)
                m = _margin(face, Vector(q.X, q.Y, 0))
                if m > score:
                    best, score = (q.X, q.Y), m
    return best, score


def point(face: Face, grid: int = 24) -> tuple[float, float]:
    """A point inside the face (on Z = 0), well clear of its edges."""
    c = face.center()
    if _margin(face, Vector(c.X, c.Y, 0)) > 0.5:
        return (c.X, c.Y)
    bb = face.bounding_box()
    best, score = None, 0.0
    for i in range(grid):
        for j in range(grid):
            x = bb.min.X + (i + 0.5) * bb.size.X / grid
            y = bb.min.Y + (j + 0.5) * bb.size.Y / grid
            m = _margin(face, Vector(x, y, 0))
            if m > score:
                best, score = (x, y), m
    if score < 0.25:
        edge_best, edge_score = _from_edges(face)
        if edge_score > score:
            best, score = edge_best, edge_score
    if best is None or score < 0.05:
        raise ValueError(f"no point found inside the face at {tuple(c)}")
    return best


def points(shape) -> list[tuple[float, float]]:
    """One point inside each face of a 2D shape, in a stable order (by x, then y)."""
    pts = [point(f) for f in shape.faces()]
    return sorted((round(x, 3), round(y, 3)) for x, y in pts)


def _faces(shape) -> list:
    """Non-empty faces of a boolean result, which is None when nothing is left."""
    return [] if shape is None else [g for g in shape.faces() if g.area > 1e-9]


def cells(profiles: list[dict]) -> list[Face]:
    """The faces of a recipe sketch's arrangement: every region its profiles bound."""
    import recipe

    out: list = []
    seen = None
    for pr in (recipe._profile(p) for p in profiles):
        f = recipe._face(pr)
        nxt = []
        for c in out:
            nxt += [g for part in (c & f, c - f) for g in _faces(part)]
        rest = f if seen is None else f - seen
        nxt += _faces(rest)
        seen = f if seen is None else seen + f
        out = nxt
    return out


def cell_points(profiles: list[dict], target) -> list[tuple[float, float]]:
    """A point in every arrangement cell of the sketch that lies inside `target`, a 2D shape:
    the regions an extrude must pick to make that shape from the sketch."""
    pts = []
    for c in cells(profiles):
        common = c & target
        if common is not None and common.area > 0.5 * c.area:
            x, y = point(c)
            pts.append((round(x, 3), round(y, 3)))
    return sorted(pts)
