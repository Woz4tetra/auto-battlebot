"""Fail when a non-VBATT track or via sits in the B.Cu VBATT neck keepout of a routed board."""

import json
import sys

import pcbnew

KEEPOUT_NAME = "VBATT neck keepout"


def neck_box(placement_path):
    zone = next(
        z for z in json.load(open(placement_path))["zones"] if z.get("name") == KEEPOUT_NAME
    )
    xs = [p[0] for p in zone["outline"]]
    ys = [p[1] for p in zone["outline"]]
    return pcbnew.BOX2I(
        pcbnew.VECTOR2I(pcbnew.FromMM(min(xs)), pcbnew.FromMM(min(ys))),
        pcbnew.VECTOR2I(pcbnew.FromMM(max(xs) - min(xs)), pcbnew.FromMM(max(ys) - min(ys))),
    )


def intruders(board, box):
    return [
        (item.GetClass(), item.GetNetname())
        for item in board.GetTracks()
        if item.GetNetname() != "VBATT"
        and (item.GetClass() == "PCB_VIA" or item.IsOnLayer(pcbnew.B_Cu))
        and box.Intersects(item.GetBoundingBox())
    ]


def main(board_path, placement_path):
    found = intruders(pcbnew.LoadBoard(board_path), neck_box(placement_path))
    print(f"VBATT neck: {len(found)} foreign track or via items {found}")
    sys.exit(1 if found else 0)


main(sys.argv[1], sys.argv[2])
