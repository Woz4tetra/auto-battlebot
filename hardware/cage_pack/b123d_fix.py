"""Patch Face.radius so trimmed cylinders and spheres report a radius."""

from build123d import Face, GeomType
from OCP.Geom import Geom_RectangularTrimmedSurface

_orig = Face.radius.fget


def _radius(self):  # type: ignore[no-untyped-def]
    r = _orig(self)
    if r is not None or self.geom_type not in (GeomType.CYLINDER, GeomType.SPHERE):
        return r
    surf = self.geom_adaptor()
    if isinstance(surf, Geom_RectangularTrimmedSurface):
        return surf.BasisSurface().Radius()
    return None


Face.radius = property(_radius)
