"""Work around build123d 0.12 Face.radius returning None for trimmed cylinders.

A boss merged into a wall, or a key slot cut across a hole, leaves cylindrical faces whose
surface is a Geom_RectangularTrimmedSurface. build123d returns None for their radius, and the
build123d-part skill's check.py crashes rounding it. Import this module before building.
"""

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
