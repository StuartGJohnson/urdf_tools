#!/usr/bin/env python3
"""
xacro2stl_cq.py — Convert Xacro/URDF (primitives-only) → STL (GitHub-viewable).

Deps:
  pip install yourdfpy cadquery numpy
  # for .xacro:
  sudo apt install ros-$ROS_DISTRO-xacro  # or: pip install xacro (ensure `xacro` on PATH)

Notes:
- Assumes geometry is defined with URDF primitives: box / cylinder / sphere.
- Exports a *single STL* (union of all visuals) at a chosen joint configuration.
"""

import argparse
import math
import os
import sys
import tempfile
import subprocess
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yourdfpy as urdf_api
from yourdfpy import URDF

import cadquery as cq


# ------------------------- FK and transforms -------------------------

def parse_joint_overrides(pairs: List[str]) -> Dict[str, float]:
    """Parse --joint name=angle_deg pairs → radians dict for FK (revolute/continuous)."""
    res: Dict[str, float] = {}
    for p in pairs or []:
        if "=" not in p:
            raise argparse.ArgumentTypeError(f"Invalid --joint '{p}', expected NAME=DEG")
        name, val = p.split("=", 1)
        res[name] = math.radians(float(val))
    return res


def transform_from_origin(origin) -> np.ndarray:
    """yourdfpy origins are already 4×4 numpy arrays; accept that or build from xyz/rpy."""
    if origin is None:
        return np.eye(4)
    if isinstance(origin, np.ndarray) and origin.shape == (4, 4):
        return origin.astype(float)
    # Fallback (rare)
    T = np.eye(4)
    xyz = getattr(origin, "xyz", None)
    rpy = getattr(origin, "rpy", None)
    if xyz is not None:
        T[:3, 3] = np.array(xyz, dtype=float)
    if rpy is not None:
        r, p, y = [float(v) for v in rpy]
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        T[:3, :3] = Rz @ Ry @ Rx
    return T


def axis_angle_to_R(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation from unit axis and angle (rad)."""
    a = axis / (np.linalg.norm(axis) + 1e-12)
    x, y, z = a
    K = np.array([[0, -z, y],
                  [z, 0, -x],
                  [-y, x, 0]])
    I = np.eye(3)
    return I + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def joint_motion_transform(joint, q: float) -> np.ndarray:
    """
    4×4 motion from yourdfpy Joint: type in {'fixed','revolute','continuous','prismatic'}
    q in radians (revolute/continuous) or meters (prismatic).
    """
    T = np.eye(4)
    jtype = getattr(joint, "type", "fixed")
    axis = np.array(getattr(joint, "axis", [1.0, 0.0, 0.0]), dtype=float)
    if jtype in ("revolute", "continuous"):
        T[:3, :3] = axis_angle_to_R(axis, float(q))
    elif jtype == "prismatic":
        T[:3, 3] = axis * float(q)
    return T


def compute_fk(robot: URDF, cfg: Dict[str, float]) -> Dict[str, np.ndarray]:
    """
    FK over yourdfpy model.
    Returns dict: link_name -> 4x4 pose in base frame.
    Uses joint_map (name->Joint) and link_map (name->Link). Joint.origin is 4×4.
    """
    joints = robot.joint_map          # dict[str, Joint]
    links  = robot.link_map           # dict[str, Link]

    # Build adjacency: parent_link -> [Joint,...]
    parent_children: Dict[str, List[object]] = {}
    child_names = set()
    for j in joints.values():
        parent = j.parent if isinstance(j.parent, str) else getattr(j.parent, "name", None)
        child  = j.child  if isinstance(j.child,  str) else getattr(j.child,  "name", None)
        if parent is None or child is None:
            continue
        child_names.add(child)
        parent_children.setdefault(parent, []).append(j)

    # Base link = link that is never a child
    link_names = list(links.keys())
    base = next((nm for nm in link_names if nm not in child_names), link_names[0])

    # BFS accumulate transforms
    T_links: Dict[str, np.ndarray] = {base: np.eye(4)}
    stack = [base]
    while stack:
        parent_name = stack.pop()
        T_parent = T_links[parent_name]
        for j in parent_children.get(parent_name, []):
            T_joint_origin = transform_from_origin(j.origin)   # yourdfpy: 4×4
            q = cfg.get(j.name, 0.0)                           # rad (rev/cont), m (pris)
            T_motion = joint_motion_transform(j, q)
            child_name = j.child if isinstance(j.child, str) else getattr(j.child, "name", None)
            T_links[child_name] = T_parent @ T_joint_origin @ T_motion
            stack.append(child_name)

    # Ensure all links present
    for nm in link_names:
        T_links.setdefault(nm, np.eye(4))
    return T_links


def mat_to_rpy_zyx(R: np.ndarray) -> Tuple[float, float, float]:
    """Rotation matrix → (roll, pitch, yaw), ZYX convention (URDF)."""
    # guard for numerical issues
    sy = -R[2, 0]
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    if cy < 1e-9:
        # gimbal lock: yaw ≈ atan2(-R01,R11); pitch = ±pi/2; roll = 0
        yaw = math.atan2(-R[0, 1], R[1, 1])
        pitch = math.asin(np.clip(-R[2, 0], -1.0, 1.0))
        roll = 0.0
    else:
        pitch = math.asin(np.clip(-R[2, 0], -1.0, 1.0))
        roll  = math.atan2(R[2, 1], R[2, 2])
        yaw   = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


# ------------------------- geometry → CadQuery -------------------------

def make_cq_solid_from_geometry(geometry, segments: int) -> cq.Workplane:
    """
    Build a CadQuery solid centered at origin from a yourdfpy Visual.geometry wrapper.
    Supports box/cylinder/sphere only (as requested).
    """
    g = geometry
    box = getattr(g, "box", None)
    if box is not None:
        sx, sy, sz = map(float, box.size)   # URDF boxes are full lengths
        # centered box
        return cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, True))

    cyl = getattr(g, "cylinder", None)
    if cyl is not None:
        r = float(cyl.radius)
        h = float(cyl.length)
        # URDF cylinder axis = Z; make centered cylinder
        return cq.Workplane("XY").circle(r).extrude(h/2.0, both=True)

    sph = getattr(g, "sphere", None)
    if sph is not None:
        r = float(sph.radius)
        return cq.Workplane("XY").sphere(r)

    raise TypeError("Only box/cylinder/sphere are supported for STL export (no external meshes).")


def apply_pose(wp: cq.Workplane, T: np.ndarray) -> cq.Workplane:
    """
    Apply pose T (4x4) to a CadQuery solid:
      - rotate about world X, then Y, then Z (degrees) for R = Rz*Ry*Rx
      - then translate
    """
    R = T[:3, :3]; t = T[:3, 3]
    roll, pitch, yaw = mat_to_rpy_zyx(R)  # radians
    wp = wp.rotate((0, 0, 0), (1, 0, 0), math.degrees(roll))
    wp = wp.rotate((0, 0, 0), (0, 1, 0), math.degrees(pitch))
    wp = wp.rotate((0, 0, 0), (0, 0, 1), math.degrees(yaw))
    wp = wp.translate(tuple(t.tolist()))
    return wp


# ------------------------- I/O -------------------------

def xacro_to_temp_urdf(xacro_path: Path) -> Path:
    """Run `xacro <file>` and write the resulting URDF to a temporary file. Return its path."""
    try:
        res = subprocess.run(["xacro", str(xacro_path)], check=True, capture_output=True, text=True)
    except FileNotFoundError:
        raise RuntimeError(
            "xacro CLI not found. Install with `sudo apt install ros-$ROS_DISTRO-xacro` "
            "or `pip install xacro` and ensure `xacro` is on PATH."
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"xacro failed:\n{e.stderr}") from e
    tmp = tempfile.NamedTemporaryFile(prefix="xacro2stl_", suffix=".urdf", delete=False)
    tmp.write(res.stdout.encode("utf-8")); tmp.flush(); tmp.close()
    return Path(tmp.name)


def load_urdf(path: Path) -> URDF:
    """Load URDF; for .xacro, invoke xacro CLI then load temp URDF."""
    if path.suffix.lower() == ".xacro":
        tmp = xacro_to_temp_urdf(path)
        try:
            model = URDF.load(str(tmp))
        finally:
            try: os.unlink(tmp)
            except Exception: pass
        return model
    return URDF.load(str(path))


# ------------------------- main build/export -------------------------

def build_union_solid(robot: URDF, joint_rad: Dict[str, float], segments: int) -> cq.Workplane:
    """Create a single CadQuery solid by unioning all link visuals at the FK pose."""
    T_links = compute_fk(robot, joint_rad)

    result = None  # cq.Workplane
    for link_name, link in robot.link_map.items():
        T_link = T_links.get(link_name, np.eye(4))
        for vis in getattr(link, "visuals", []) or []:
            T = T_link @ transform_from_origin(getattr(vis, "origin", None))
            wp = make_cq_solid_from_geometry(vis.geometry, segments)
            wp = apply_pose(wp, T)
            if result is None:
                result = wp
            else:
                # union this solid into the accumulated result
                result = result.union(wp)

    if result is None:
        raise RuntimeError("No visual geometry found. Ensure your URDF/Xacro has box/cylinder/sphere visuals.")
    # Make sure it's a single solid if possible
    try:
        result = result.combineSolids()
    except Exception:
        pass
    return result


def main():
    ap = argparse.ArgumentParser(description="Convert Xacro/URDF (primitives-only) → STL using yourdfpy + CadQuery.")
    ap.add_argument("input", type=Path, help="Path to .xacro or .urdf")
    ap.add_argument("-o", "--output", type=Path, help="Output .stl path (defaults to input stem + .stl)")
    ap.add_argument("--segments", type=int, default=32, help="Tessellation hint for cylinders (default 32)")
    ap.add_argument("--joint", action="append", default=[], metavar="NAME=DEG",
                    help="Set a revolute/continuous joint angle in degrees (repeatable).")
    ap.add_argument("--scale", type=float, default=1.0, help="Uniform scale factor applied at end (URDF is meters).")
    args = ap.parse_args()

    if not args.input.exists():
        print(f"ERROR: Input file not found: {args.input}", file=sys.stderr)
        sys.exit(2)

    robot = load_urdf(args.input)
    joint_rad = parse_joint_overrides(args.joint)

    solid = build_union_solid(robot, joint_rad, segments=args.segments)

    if args.scale != 1.0:
        solid = solid.scale(args.scale)

    out_path = args.output if args.output is not None else args.input.with_suffix(".stl")
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # STL export (you can pass mesh quality via tolerance if desired)
    cq.exporters.export(solid, str(out_path), exportType="STL")
    print(f"Wrote {out_path}")

if __name__ == "__main__":
    main()
