"""Onshape URDF export of Mr Stabs Mk2: kinematics, lumped mass properties, tag frames.

The export's root frame is the Onshape assembly frame: the axle runs along X_asm and the
robot faces -Y_asm. The body frame used everywhere else (C++ filter, MJCF, smoother) is FLU
at the axle midpoint, so x = -Y_asm, y = +X_asm, z = +Z_asm.

The export carries no mass for the motors, ESCs, flight controller, batteries or switch (Onshape
mass overrides do not survive it), so only the wheels are lumped from it. The chassis is the
Onshape whole-assembly total minus the wheels; see `chassis_from_assembly`.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

# Assembly frame -> body frame rotation: x = -Y_asm, y = +X_asm, z = +Z_asm.
R_BODY_ASM = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

# Wheels spin on these joints; everything below them in the tree turns with the wheel.
WHEEL_JOINTS = ("cylindrical_1_1", "cylindrical_2_1")

# Structural parts whose meshes make the chassis collision shape.
STRUCTURAL_LINKS = ("mr_stabs_mk2_chassis", "mr_stabs_mk2_top_plate", "mr_stabs_mk2_bottom_plate")


def _origin_matrix(origin: ET.Element | None) -> np.ndarray:
    out = np.eye(4)
    if origin is None:
        return out
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    # URDF rpy is extrinsic roll-pitch-yaw about fixed x, y, z.
    out[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    out[:3, 3] = xyz
    return out


@dataclass
class LinkInertial:
    mass: float
    com: np.ndarray  # (3,) in the link frame
    inertia: np.ndarray  # (3, 3) about the COM, link-frame axes


@dataclass
class MassBody:
    """A lumped body: mass, COM and inertia about the COM, all in one frame."""

    mass: float
    com: np.ndarray
    inertia: np.ndarray

    def transformed(self, rotation: np.ndarray, translation: np.ndarray) -> MassBody:
        return MassBody(
            mass=self.mass,
            com=rotation @ self.com + translation,
            inertia=rotation @ self.inertia @ rotation.T,
        )


def _skew_parallel(mass: float, offset: np.ndarray) -> np.ndarray:
    """Parallel-axis term: inertia of a point mass at `offset` about the origin."""
    return mass * (float(offset @ offset) * np.eye(3) - np.outer(offset, offset))


def combine(bodies: list[MassBody]) -> MassBody:
    """Lump several bodies (same frame) into one, inertia about the combined COM."""
    mass = sum(b.mass for b in bodies)
    com = sum((b.mass * b.com for b in bodies), np.zeros(3)) / mass
    inertia = np.zeros((3, 3))
    for b in bodies:
        inertia += b.inertia + _skew_parallel(b.mass, b.com - com)
    return MassBody(mass=mass, com=com, inertia=inertia)


def subtract(whole: MassBody, parts: list[MassBody]) -> MassBody:
    """Remove `parts` from `whole` (same frame); the inverse of `combine`."""
    parts_mass = sum(p.mass for p in parts)
    mass = whole.mass - parts_mass
    moment = whole.mass * whole.com - sum((p.mass * p.com for p in parts), np.zeros(3))
    com = moment / mass
    about_origin = whole.inertia + _skew_parallel(whole.mass, whole.com)
    for p in parts:
        about_origin -= p.inertia + _skew_parallel(p.mass, p.com)
    return MassBody(mass=mass, com=com, inertia=about_origin - _skew_parallel(mass, com))


class UrdfAssembly:
    """Kinematic tree of the Onshape export, all poses in the root (assembly) frame."""

    def __init__(self, urdf_path: Path) -> None:
        self.path = Path(urdf_path)
        root = ET.parse(self.path).getroot()
        self.links = {link.get("name", ""): link for link in root.findall("link")}
        self.joints = {j.get("name", ""): j for j in root.findall("joint")}
        self._parent_joint: dict[str, ET.Element] = {}
        self._children: dict[str, list[str]] = {}
        for joint in self.joints.values():
            child = _require(joint.find("child")).get("link", "")
            parent = _require(joint.find("parent")).get("link", "")
            self._parent_joint[child] = joint
            self._children.setdefault(parent, []).append(child)

    def link_pose(self, name: str) -> np.ndarray:
        """4x4 pose of a link frame in the root frame, joints at zero."""
        pose = np.eye(4)
        while name in self._parent_joint:
            joint = self._parent_joint[name]
            pose = _origin_matrix(joint.find("origin")) @ pose
            name = _require(joint.find("parent")).get("link", "")
        return pose

    def subtree(self, link: str) -> list[str]:
        out = [link]
        for child in self._children.get(link, []):
            out.extend(self.subtree(child))
        return out

    def inertial(self, name: str) -> LinkInertial | None:
        node = self.links[name].find("inertial")
        if node is None:
            return None
        mass = float(_require(node.find("mass")).get("value", "0"))
        origin = _origin_matrix(node.find("origin"))
        attrib = _require(node.find("inertia")).attrib
        ixx, iyy, izz = (float(attrib[k]) for k in ("ixx", "iyy", "izz"))
        ixy, ixz, iyz = (float(attrib[k]) for k in ("ixy", "ixz", "iyz"))
        tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        rot = origin[:3, :3]
        return LinkInertial(mass=mass, com=origin[:3, 3], inertia=rot @ tensor @ rot.T)

    def mass_body_in_root(self, name: str) -> MassBody | None:
        inertial = self.inertial(name)
        if inertial is None or inertial.mass <= 0.0:
            return None
        pose = self.link_pose(name)
        return MassBody(inertial.mass, inertial.com, inertial.inertia).transformed(
            pose[:3, :3], pose[:3, 3]
        )

    def lumped(self, names: list[str]) -> MassBody:
        bodies = [b for b in (self.mass_body_in_root(n) for n in names) if b is not None]
        return combine(bodies)

    def wheel_links(self, joint_name: str) -> list[str]:
        child = _require(self.joints[joint_name].find("child")).get("link", "")
        return self.subtree(child)

    def visual_mesh(self, name: str) -> tuple[str, np.ndarray]:
        """Mesh file name and the 4x4 pose of the mesh frame in the root frame."""
        visual = _require(self.links[name].find("visual"))
        mesh = _require(_require(visual.find("geometry")).find("mesh"))
        filename = mesh.get("filename", "").split("/")[-1]
        return filename, self.link_pose(name) @ _origin_matrix(visual.find("origin"))


def _require(node: ET.Element | None) -> ET.Element:
    if node is None:
        raise ValueError("malformed URDF: missing element")
    return node


def body_from_root(assembly: UrdfAssembly) -> tuple[np.ndarray, np.ndarray]:
    """Rotation and translation taking root-frame points into the body frame.

    The origin is the axle midpoint: the wheel joints sit on the axle line, and the midpoint
    along it is halfway between the two lumped wheel COMs.
    """
    wheel_coms = [assembly.lumped(assembly.wheel_links(j)).com for j in WHEEL_JOINTS]
    joint_points = [assembly.link_pose(assembly.wheel_links(j)[0])[:3, 3] for j in WHEEL_JOINTS]
    axle_mid = 0.5 * (joint_points[0] + joint_points[1])
    axle_mid[0] = 0.5 * (wheel_coms[0][0] + wheel_coms[1][0])
    return R_BODY_ASM, -R_BODY_ASM @ axle_mid


def load_mesh_in_root(
    assembly: UrdfAssembly, mesh_dir: Path, name: str
) -> list[tuple[trimesh.Trimesh, np.ndarray]]:
    """Every sub-mesh of a link's visual in the root frame, with its RGBA base color."""
    filename, pose = assembly.visual_mesh(name)
    scene: trimesh.Scene = trimesh.load_scene(mesh_dir / filename)
    out: list[tuple[trimesh.Trimesh, np.ndarray]] = []
    for node in scene.graph.nodes_geometry:
        transform, geom_name = scene.graph[node]
        geom = scene.geometry[geom_name].copy()
        geom.apply_transform(pose @ transform)
        color = np.array([1.0, 1.0, 1.0, 1.0])
        material = getattr(geom.visual, "material", None)
        base = getattr(material, "baseColorFactor", None)
        if base is not None:
            color = np.asarray(base, dtype=float)
            if color.max() > 1.0:
                color = color / 255.0
        out.append((geom, color))
    return out


@dataclass
class TagFrame:
    """A tag's marker frame in the body frame.

    `rotation` and `translation` form T_body_tag, mapping points in the OpenCV marker frame
    (origin at the tag center, x right and y up in the printed image, z out of the face; the
    object points solvePnP IPPE_SQUARE expects) into the body frame.
    """

    tag_id: int
    rotation: np.ndarray
    translation: np.ndarray
    size_m: float  # black-square side, the length OpenCV calls the marker length
    tilt_deg: float  # angle between the tag normal and the body's +z (or -z when underneath)
    mirrored: bool  # the CAD face only decodes when mirrored


def extract_tag_frame(
    meshes_in_body: list[tuple[trimesh.Trimesh, np.ndarray]],
    tag_id: int,
    outward: np.ndarray,
    px_per_m: float = 20000.0,
) -> TagFrame:
    """Find a tag's marker frame by rendering its CAD face and running the aruco detector.

    The export does not say which in-plane direction is the printed image's "up", so the face
    is rasterized as seen from outside (looking along -normal) and decoded; the detected corner
    order then fixes the frame. The face is the set of triangles facing `outward` that lie on
    the outermost plane.
    """
    triangles: list[np.ndarray] = []
    shades: list[float] = []
    for mesh, color in meshes_in_body:
        normals = mesh.face_normals
        keep = normals @ outward > 0.9
        for tri in mesh.triangles[keep]:
            triangles.append(tri)
            shades.append(float(color[:3].mean()))
    if not triangles:
        raise ValueError(f"tag {tag_id}: no faces toward {outward}")
    tris = np.array(triangles)
    shade = np.array(shades)
    normal = np.mean(
        [
            np.cross(t[1] - t[0], t[2] - t[0]) / np.linalg.norm(np.cross(t[1] - t[0], t[2] - t[0]))
            for t in tris
            if np.linalg.norm(np.cross(t[1] - t[0], t[2] - t[0])) > 1e-12
        ],
        axis=0,
    )
    normal /= np.linalg.norm(normal)
    heights = tris.reshape(-1, 3) @ normal
    top = heights.max()
    on_face = np.all(np.abs(tris @ normal - top) < 0.5e-3, axis=1)
    tris, shade = tris[on_face], shade[on_face]

    # Camera looking along -normal: image u = e1, image v (down) = e2, e1 x e2 = -normal.
    e1 = np.cross(normal, [0.0, 0.0, 1.0] if abs(normal[2]) < 0.9 else [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(-normal, e1)
    center = tris.reshape(-1, 3).mean(axis=0)
    uv = np.stack([(tris - center) @ e1, (tris - center) @ e2], axis=-1)
    extent = np.abs(uv).max() * 1.4
    size_px = int(2 * extent * px_per_m)
    image = np.full((size_px, size_px), 255, np.uint8)
    # Paint light faces first so the black pattern lands on top of any overlapping plate.
    for idx in np.argsort(-shade):
        pts = ((uv[idx] + extent) * px_per_m).astype(np.int32)
        cv2.fillConvexPoly(image, pts, 0 if shade[idx] < 0.2 else 255)

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters(),
    )
    mirrored = False
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None or tag_id not in ids.flatten():
        corners, ids, _ = detector.detectMarkers(np.ascontiguousarray(image[:, ::-1]))
        mirrored = True
        if ids is None or tag_id not in ids.flatten():
            raise ValueError(f"tag {tag_id}: rendered CAD face does not decode")
    found = corners[[int(i) for i in ids.flatten()].index(tag_id)].reshape(4, 2).astype(float)
    if mirrored:
        found[:, 0] = size_px - 1 - found[:, 0]
    # Back to 3D on the face plane.
    uv_m = found / px_per_m - extent
    pts3 = center + uv_m[:, :1] * e1 + uv_m[:, 1:] * e2
    tag_center = pts3.mean(axis=0)
    x_axis = 0.5 * ((pts3[1] - pts3[0]) + (pts3[2] - pts3[3]))
    y_axis = 0.5 * ((pts3[0] - pts3[3]) + (pts3[1] - pts3[2]))
    size = 0.5 * (np.linalg.norm(x_axis) + np.linalg.norm(y_axis))
    x_axis /= np.linalg.norm(x_axis)
    y_axis -= (y_axis @ x_axis) * x_axis
    y_axis /= np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    rotation = np.column_stack([x_axis, y_axis, z_axis])
    reference = np.array([0.0, 0.0, 1.0]) if outward[2] > 0 else np.array([0.0, 0.0, -1.0])
    tilt = float(np.degrees(np.arccos(np.clip(z_axis @ reference, -1.0, 1.0))))
    return TagFrame(
        tag_id=tag_id,
        rotation=rotation,
        translation=tag_center,
        size_m=float(size),
        tilt_deg=tilt,
        mirrored=mirrored,
    )


def onshape_inertia_to_tensor(
    lxx: float, lyy: float, lzz: float, lxy: float, lxz: float, lyz: float
) -> np.ndarray:
    """Onshape's "Mass moments of inertia" panel, in g mm^2, as a kg m^2 tensor.

    The panel shows the tensor elements themselves (products already carry the minus sign):
    subtracting the wheels from it reproduces the chassis product of inertia the URDF parts
    give, in sign and size.
    """
    tensor = np.array([[lxx, lxy, lxz], [lxy, lyy, lyz], [lxz, lyz, lzz]])
    return tensor * 1e-9


def chassis_from_assembly(
    assembly_total: MassBody, wheels: list[MassBody], scale_mass: float
) -> tuple[MassBody, MassBody]:
    """Chassis = Onshape whole assembly minus the wheels, then topped up to the scale.

    Returns (chassis at CAD mass, chassis at scale mass). The unmodeled difference between the
    scale and CAD (wire, glue, solder) is spread like the rest of the chassis: added at its COM
    with the inertia scaled by the mass ratio.
    """
    cad = subtract(assembly_total, wheels)
    extra = scale_mass - assembly_total.mass
    ratio = (cad.mass + extra) / cad.mass
    scaled = MassBody(mass=cad.mass + extra, com=cad.com.copy(), inertia=cad.inertia * ratio)
    return cad, scaled
