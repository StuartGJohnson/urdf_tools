#!/usr/bin/env python3
"""
xacro2mesh.py — Convert Xacro/URDF → GLB (default) or STL for GitHub-friendly viewing.

Deps:
  pip install yourdfpy trimesh numpy
  # ensure xacro CLI exists (for .xacro files):
  #   sudo apt install ros-$ROS_DISTRO-xacro
  #   or: pip install xacro

This version uses yourdfpy only and performs forward kinematics internally
(across fixed/revolute/continuous/prismatic joints) — no link_fk() needed.
"""

import argparse
import math
import os
import sys
import tempfile
import subprocess
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import numpy as np
import yourdfpy as urdf_api  # single explicit dependency
from yourdfpy import URDF

import trimesh
from trimesh.creation import box as tm_box
from trimesh.creation import cylinder as tm_cylinder
from trimesh.creation import icosphere as tm_icosphere


# ------------------------- helpers -------------------------

def parse_joint_overrides(pairs: List[str]) -> Dict[str, float]:
    """Parse --joint name=angle_deg pairs → radians dict for FK (revolute/continuous)."""
    res: Dict[str, float] = {}
    for p in pairs or []:
        if "=" not in p:
            raise argparse.ArgumentTypeError(f"Invalid --joint '{p}', expected NAME=DEG")
        name, val = p.split("=", 1)
        res[name] = math.radians(float(val))
    return res


def color_to_uint8(rgba: Tuple[float, float, float, float]) -> np.ndarray:
    arr = np.clip(np.array(rgba, dtype=float), 0.0, 1.0) * 255.0
    return arr.astype(np.uint8)


def get_visual_color(visual, default_rgba=(0.75, 0.75, 0.75, 1.0)) -> np.ndarray:
    rgba = None
    try:
        mat = getattr(visual, "material", None)
        if mat is not None:
            if hasattr(mat, "color") and getattr(mat.color, "rgba", None) is not None:
                rgba = mat.color.rgba  # yourdfpy material.color.rgba
            elif getattr(mat, "rgba", None) is not None:
                rgba = mat.rgba
    except Exception:
        rgba = None
    if rgba is None:
        rgba = default_rgba
    return color_to_uint8(tuple(rgba))


def transform_from_origin(origin) -> np.ndarray:
    """Accept yourdfpy 4x4 origins (preferred) or xyz/rpy dict-like."""
    if origin is None:
        return np.eye(4)
    # yourdfpy: origin is already a 4x4 numpy array
    if isinstance(origin, np.ndarray) and origin.shape == (4, 4):
        return origin.astype(float)
    # Fallback: xyz/rpy
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
    a = axis / (np.linalg.norm(axis) + 1e-12)
    x, y, z = a
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    I = np.eye(3)
    return I + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)

def joint_motion_transform(joint, q: float) -> np.ndarray:
    """
    Build 4x4 motion from yourdfpy Joint: joint.type in {'fixed','revolute','continuous','prismatic'}
    q in radians for revolute/continuous; meters for prismatic.
    """
    T = np.eye(4)
    jtype = getattr(joint, "type", "fixed")
    axis = np.array(getattr(joint, "axis", [1.0, 0.0, 0.0]), dtype=float)
    if jtype in ("revolute", "continuous"):
        T[:3, :3] = axis_angle_to_R(axis, float(q))
    elif jtype == "prismatic":
        T[:3, 3] = axis * float(q)
    return T

def compute_fk(robot, cfg: Dict[str, float]) -> Dict[str, np.ndarray]:
    """
    FK over yourdfpy model.
    Returns dict: link_name -> 4x4 pose in base frame.
    Uses joint_map (name->Joint) and link_map (name->Link).
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
            T_joint_origin = transform_from_origin(j.origin)   # yourdfpy: 4x4
            q = cfg.get(j.name, 0.0)                           # rad for revolute/continuous; m for prismatic
            T_motion = joint_motion_transform(j, q)
            child_name = j.child if isinstance(j.child, str) else getattr(j.child, "name", None)
            T_child = T_parent @ T_joint_origin @ T_motion
            T_links[child_name] = T_child
            stack.append(child_name)

    # Ensure all links present
    for nm in link_names:
        T_links.setdefault(nm, np.eye(4))
    return T_links

def _geom_kind(geom: object) -> str:
    """Return 'box'|'cylinder'|'sphere'|'mesh' by duck-typing across URDF libs."""
    n = geom.__class__.__name__.lower()
    if hasattr(geom, "size") and n == "box":
        return "box"
    if all(hasattr(geom, a) for a in ("radius", "length")) and n == "cylinder":
        return "cylinder"
    if hasattr(geom, "radius") and n == "sphere":
        return "sphere"
    if n == "mesh" or hasattr(geom, "filename") or hasattr(geom, "filenames"):
        return "mesh"
    return n


def geom_to_trimesh(geom, segments: int, sphere_subdiv: int) -> trimesh.Trimesh:
    kind = _geom_kind(geom)
    if kind == "box":
        size = np.array(getattr(geom, "size"), dtype=float)
        return tm_box(extents=size)
    elif kind == "cylinder":
        radius = float(getattr(geom, "radius"))
        length = float(getattr(geom, "length"))
        return tm_cylinder(radius=radius, height=length, sections=max(8, int(segments)))
    elif kind == "sphere":
        radius = float(getattr(geom, "radius"))
        return tm_icosphere(subdivisions=max(1, int(sphere_subdiv)), radius=radius)
    elif kind == "mesh":
        filenames: List[str] = []
        if hasattr(geom, "filenames") and getattr(geom, "filenames"):
            filenames = list(getattr(geom, "filenames"))
        elif hasattr(geom, "filename") and getattr(geom, "filename"):
            filenames = [getattr(geom, "filename")]
        scales: List[np.ndarray] = []
        if hasattr(geom, "scales") and getattr(geom, "scales"):
            scales = [np.array(s, dtype=float) for s in getattr(geom, "scales")]
        elif hasattr(geom, "scale") and getattr(geom, "scale") is not None:
            s = getattr(geom, "scale")
            scales = [np.array(s, dtype=float)]
        else:
            scales = [np.ones(3)] * max(1, len(filenames))

        meshes: List[trimesh.Trimesh] = []
        for fn, sc in zip(filenames, scales):
            tm = trimesh.load_mesh(fn, process=False)
            if sc is not None:
                tm.apply_scale(sc)
            if not isinstance(tm, trimesh.Trimesh):
                tm = tm.dump(concatenate=True)
            meshes.append(tm)
        if not meshes:
            raise RuntimeError("URDF Mesh geometry had no loadable files.")
        return trimesh.util.concatenate(meshes)
    else:
        raise TypeError(f"Unsupported geometry type: {type(geom)}")


def xacro_cli_to_temp_urdf(xacro_path: Path) -> Path:
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
    tmp = tempfile.NamedTemporaryFile(prefix="xacro2mesh_", suffix=".urdf", delete=False)
    tmp.write(res.stdout.encode("utf-8")); tmp.flush(); tmp.close()
    return Path(tmp.name)


def load_urdf_model(path: Path) -> URDF:
    """Load URDF model; for .xacro, invoke xacro CLI then load temp URDF."""
    if path.suffix.lower() == ".xacro":
        tmp = xacro_cli_to_temp_urdf(path)
        try:
            model = URDF.load(str(tmp))
        finally:
            try:
                os.unlink(tmp)
            except Exception:
                pass
        return model
    return URDF.load(str(path))


def build_scene_from_urdf(
    robot: URDF,
    joint_rad: Dict[str, float],
    segments: int = 24,
    sphere_subdiv: int = 3,
    default_color=(0.75, 0.75, 0.75, 1.0),
    fuse: bool = False,
    center: bool = False,
) -> Tuple[trimesh.Scene, Optional[trimesh.Trimesh]]:
    """Build a trimesh.Scene (and optional fused mesh) from URDF visuals at the given joint config."""
    # FK for all links (relative to base)
    link_T = compute_fk(robot, joint_rad)

    meshes: List[trimesh.Trimesh] = []
    names: List[str] = []

    for link_name, link in robot.link_map.items():
        T_link = link_T.get(link_name, np.eye(4))
        for idx, visual in enumerate(getattr(link, "visuals", []) or []):
            T = T_link @ transform_from_origin(getattr(visual, "origin", None))  # origin is often 4x4
            try:
                m = geom_to_trimesh(visual.geometry, segments=segments, sphere_subdiv=sphere_subdiv)
            except TypeError:
                continue
            rgba = get_visual_color(visual, default_color)
            m = m.copy()
            if not (m.visual and getattr(m.visual, "face_colors", None) is not None and len(m.visual.face_colors) > 0):
                m.visual = trimesh.visual.ColorVisuals(mesh=m, face_colors=np.tile(rgba, (len(m.faces), 1)))
            else:
                m.visual.face_colors = np.tile(rgba, (len(m.faces), 1))
            m.apply_transform(T)
            meshes.append(m)
            names.append(f"{getattr(link, 'name', 'link')}_{idx}")

    if center and meshes:
        fused_tmp = trimesh.util.concatenate(meshes)
        offset = -fused_tmp.bounding_box.centroid
        for m in meshes:
            m.apply_translation(offset)

    scene = trimesh.Scene()
    for m, n in zip(meshes, names):
        scene.add_geometry(m, node_name=n)

    fused: Optional[trimesh.Trimesh] = None
    if fuse and meshes:
        fused = trimesh.util.concatenate(meshes)
    return scene, fused


# ------------------------- CLI -------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Convert Xacro/URDF → GLB (default) or STL/OBJ using yourdfpy + trimesh."
    )
    ap.add_argument("input", type=Path, help="Path to .xacro or .urdf")
    ap.add_argument("-o", "--output", type=Path, help="Output path (defaults to input stem + .glb)")
    ap.add_argument("--format", choices=["glb", "gltf", "stl", "obj"], default="glb", help="Export format")
    ap.add_argument("--fuse", action="store_true", help="Fuse all link visuals into one mesh (recommended for STL)")
    ap.add_argument("--segments", type=int, default=24, help="Cylinder tessellation")
    ap.add_argument("--sphere-subdiv", type=int, default=3, help="Sphere tessellation (icosphere subdivisions)")
    ap.add_argument("--joint", action="append", default=[], metavar="NAME=DEG",
                    help="Set a joint angle in degrees (repeatable): --joint shoulder=15 --joint elbow=-30")
    ap.add_argument("--center", action="store_true", help="Center model at origin before export")
    ap.add_argument("--scale", type=float, default=1.0, help="Uniform scale at the end (URDF uses meters)")
    args = ap.parse_args()

    if not args.input.exists():
        print(f"ERROR: Input file not found: {args.input}", file=sys.stderr)
        sys.exit(2)

    robot = load_urdf_model(args.input)

    # Parse joint overrides (deg → rad) for revolute/continuous; prismatic (m) can be added by editing compute_fk
    joint_rad = parse_joint_overrides(args.joint)

    scene, fused = build_scene_from_urdf(
        robot, joint_rad, segments=args.segments, sphere_subdiv=args.sphere_subdiv, fuse=args.fuse, center=args.center
    )

    out_path = args.output if args.output is not None else args.input.with_suffix("." + args.format)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Final uniform scale
    if args.scale != 1.0:
        if fused is not None:
            fused.apply_scale(args.scale)
        for geom in scene.geometry.values():
            geom.apply_scale(args.scale)

    if args.format in ("glb", "gltf"):
        data = scene.export(file_type=args.format)
        if isinstance(data, (bytes, bytearray)):
            out_path.write_bytes(data)
        else:
            out_path.write_text(data)
    elif args.format in ("stl", "obj"):
        mesh = fused or (trimesh.util.concatenate(list(scene.geometry.values())) if scene.geometry else None)
        if mesh is None:
            print("WARNING: No geometry to export.", file=sys.stderr)
            sys.exit(0)
        mesh.export(out_path, file_type=args.format)
    else:
        print(f"ERROR: Unsupported format: {args.format}", file=sys.stderr)
        sys.exit(2)

    print(f"Wrote {out_path}")

if __name__ == "__main__":
    main()

