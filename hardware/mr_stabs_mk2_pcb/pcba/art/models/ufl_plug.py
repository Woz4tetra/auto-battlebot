"""Mated U.FL plug and cable render proxy."""

from dataclasses import dataclass

from build123d import Align, Cylinder, Part, Pos, Rot, Sphere  # noqa: F401

PROCESS = "none"
MATERIAL = "brass"


@dataclass
class Params:
    socket_z: float = 0.8
    mated_h: float = 2.5
    cap_dia: float = 2.0
    cable_dia: float = 1.13
    shield_top: float = 2.55
    run_y: float = 28.4


def build(p: Params) -> Part:
    c = (Align.CENTER, Align.CENTER, Align.MIN)
    plug = Pos(0, 0, p.socket_z) * Cylinder(p.cap_dia / 2, p.mated_h, align=c)
    zc = p.shield_top + p.cable_dia / 2
    plug += Pos(0, 0, zc) * Sphere(p.cable_dia / 2)
    plug += Pos(0, 0, zc) * (Rot(90, 0, 0) * Cylinder(p.cable_dia / 2, p.run_y, align=c))
    return plug


SECTIONS = ["x", "y"]


def expect(p: Params) -> dict:
    return {"bbox": (None, None, p.shield_top + p.cable_dia - p.socket_z)}
