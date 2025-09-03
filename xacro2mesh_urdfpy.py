
#!/usr/bin/env python3
import argparse
import io
import math
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import numpy as np

# Dependencies:
#   pip install urdfpy trimesh xacro numpy
try:
    import xacro
except Exception:
    xacro = None

try:
    from urdfpy import URDF
    from urdfpy import Box, Cylinder, Sphere, Mesh as UrdfMesh
except Exception as e:
    print("ERROR: urdfpy is required. Install with: pip install urdfpy", file=sys.stderr)
    raise

try:
    import trimesh
    from trimesh.creation import box as tm_box
    from trimesh.creation import cylinder as tm_cylinder
    from trimesh.creation import icosphere as tm_icosphere
except Exception as e:
    print("ERROR: trimesh is required. Install with: pip install trimesh", file=sys.stderr)
    raise


def parse_joint_overrides(pairs: List[str]) -> Dict[str, float]:
    """Parse --joint name=angle_deg pairs into radians."""
    result: Dict[str, float] = {}
    for p in pairs or []:
        if "=" not in p:
            raise argparse.ArgumentTypeError(f"Invalid joint override '{p}', expected name=deg")
        name, val = p.split("=", 1)
        deg = float(val)
        result[name] = math.radians(deg)
    return result


def color_to_uint8(rgba: Tuple[float, float, float, float]) -> np.ndarray:
    arr = np.clip(np.array(rgba, dtype=float), 0.0, 1.0) * 255.0
    return arr.astype(np.uint8)


def get_visual_color(visual, default_rgba=(0.75, 0.75, 0.75, 1.0)) -> np.ndarray:
    rgba = None
    try:
        if visual.material is not None and getattr(visual.material, "color", None) is not None:
            rgba = getattr(visual.material.color, "rgba", None)
    except Exception:
        rgba = None
    if rgba is None:
        rgba = default_rgba
    return color_to_uint8(rgba)


def transform_from_origin(origin) -> np.ndarray:
    """Convert a URDF origin (xyz, rpy) into a 4x4 transform."""
    T = np.eye(4)
    if origin is None:
        return T
    xyz = getattr(origin, "xyz", None)
    rpy = getattr(origin, "rpy", None)
    if xyz is not None:
        T[:3, 3] = np.array(xyz, dtype=float)
    if rpy is not None:
        r, p, y = [float(v) for v in rpy]
        # ZYX convention (roll-pitch-yaw)
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        Rz = np.array([[cy, -sy, 0],
                       [sy,  cy, 0],
                       [ 0,   0, 1]])
        Ry = np.array([[ cp, 0, sp],
                       [  0, 1,  0],
                       [-sp, 0, cp]])
        Rx = np.array([[1,  0,   0],
                       [0, cr, -sr],
                       [0, sr,  cr]])
        R = Rz @ Ry @ Rx
        T[:3, :3] = R
    return T


def geom_to_trimesh(geom, segments: int, sphere_subdiv: int) -> trimesh.Trimesh:
    """Convert a URDF geometry primitive to a trimesh.Trimesh."""
    if isinstance(geom, Box):
        size = np.array(geom.size, dtype=float)  # (x, y, z)
        m = tm_box(extents=size)
        return m
    elif isinstance(geom, Cylinder):
        radius = float(geom.radius)
        length = float(geom.length)
        m = tm_cylinder(radius=radius, height=length, sections=max(8, int(segments)))
        return m
    elif isinstance(geom, Sphere):
        radius = float(geom.radius)
        m = tm_icosphere(subdivisions=max(1, int(sphere_subdiv)), radius=radius)
        return m
    elif isinstance(geom, UrdfMesh):
        meshes: List[trimesh.Trimesh] = []
        # Note: urdfpy Mesh may contain multiple filenames (stl/dae). Scales may be per-mesh.
        filenames = getattr(geom, "filenames", []) or []
        scales = getattr(geom, "scales", None)
        if scales is None or len(scales) != len(filenames):
            scales = [np.ones(3)] * len(filenames)
        for filename, scale in zip(filenames, scales):
            try:
                tm = trimesh.load_mesh(filename, {'process':False})
                if scale is not None:
                    tm.apply_scale(np.array(scale, dtype=float))
                if not isinstance(tm, trimesh.Trimesh):
                    tm = tm.dump(concatenate=True)
                meshes.append(tm)
            except Exception as e:
                raise RuntimeError(f"Failed to load mesh '{filename}': {e}")
        if not meshes:
            raise RuntimeError("URDF Mesh geometry had no loadable files.")
        return trimesh.util.concatenate(meshes)
    else:
        raise TypeError(f"Unsupported geometry type: {type(geom)}")


def xacro_to_urdf_xml(xacro_path: Path) -> str:
    if xacro is None:
        raise RuntimeError("xacro Python module not available. Install with: pip install xacro")
    doc = xacro.process_file(str(xacro_path))
    return doc.toprettyxml(indent="  ")


def load_urdf_model(path: Path) -> URDF:
    if path.suffix.lower() == ".xacro":
        xml = xacro_to_urdf_xml(path)
        return URDF.from_xml_string(xml)
    else:
        return URDF.load(str(path))


def build_scene_from_urdf(
    robot: URDF,
    joint_deg: Dict[str, float],
    segments: int = 32,
    sphere_subdiv: int = 3,
    default_color=(0.75, 0.75, 0.75, 1.0),
    fuse: bool = False,
    center: bool = False,
) -> Tuple[trimesh.Scene, Optional[trimesh.Trimesh]]:
    """Build a trimesh.Scene (and optionally a fused mesh) from URDF visuals."""
    # Convert joint overrides (deg) to radians for urdfpy
    cfg = {}
    for name, val in (joint_deg or {}).items():
        # If user passed degrees already converted, accept small > 2pi as deg, else keep as rad
        if abs(val) > 2 * math.pi:
            cfg[name] = math.radians(val)
        else:
            cfg[name] = val

    # Forward kinematics to get link transforms at this joint state
    link_T = robot.link_fk(cfg=cfg)

    meshes: List[trimesh.Trimesh] = []
    names: List[str] = []

    for link in robot.links:
        T_link = link_T.get(link, np.eye(4))
        for idx, visual in enumerate(getattr(link, "visuals", []) or []):
            T_vis = transform_from_origin(visual.origin)
            T = T_link @ T_vis

            try:
                m = geom_to_trimesh(visual.geometry, segments=segments, sphere_subdiv=sphere_subdiv)
            except TypeError:
                continue

            rgba = get_visual_color(visual, default_color)
            if m.visual is None or m.visual.face_colors is None or len(m.visual.face_colors) == 0:
                m.visual = trimesh.visual.ColorVisuals(mesh=m, face_colors=np.tile(rgba, (len(m.faces), 1)))
            else:
                m.visual.face_colors = np.tile(rgba, (len(m.faces), 1))

            m = m.copy()
            m.apply_transform(T)
            meshes.append(m)
            names.append(f"{link.name}_{idx}")

    if center and len(meshes) > 0:
        fused_tmp = trimesh.util.concatenate(meshes)
        offset = -fused_tmp.bounding_box.centroid
        for m in meshes:
            m.apply_translation(offset)

    scene = trimesh.Scene()
    for m, n in zip(meshes, names):
        scene.add_geometry(m, node_name=n)

    fused: Optional[trimesh.Trimesh] = None
    if fuse and len(meshes) > 0:
        fused = trimesh.util.concatenate(meshes)
    return scene, fused


def main():
    ap = argparse.ArgumentParser(
        description="Convert Xacro/URDF to a GitHub-viewable 3D file (GLB default, STL optional) using urdfpy + trimesh."
    )
    ap.add_argument("input", type=Path, help="Path to .xacro or .urdf")
    ap.add_argument("-o", "--output", type=Path, help="Output file path (defaults to input stem + .glb)")
    ap.add_argument("--format", choices=["glb", "gltf", "stl", "obj"], default="glb", help="Export format")
    ap.add_argument("--fuse", action="store_true", help="Fuse all link visuals into one mesh (recommended for STL)")
    ap.add_argument("--segments", type=int, default=32, help="Cylinder circle sections (primitives tessellation)")
    ap.add_argument("--sphere-subdiv", type=int, default=3, help="Sphere icosphere subdivisions")
    ap.add_argument("--joint", action="append", default=[], metavar="NAME=DEG",
                    help="Set a joint to an angle in degrees (can be repeated). Example: --joint shoulder=15 --joint elbow=-30")
    ap.add_argument("--center", action="store_true", help="Center the model at the origin before export")
    ap.add_argument("--scale", type=float, default=1.0, help="Uniform scale factor applied at the end")
    args = ap.parse_args()

    if not args.input.exists():
        print(f"ERROR: Input file not found: {args.input}", file=sys.stderr)
        sys.exit(2)

    # Load URDF (process xacro if needed)
    robot = load_urdf_model(args.input)

    # Build scene + optional fused mesh
    joint_overrides = {}
    for pair in args.joint or []:
        if "=" not in pair:
            print(f"Invalid --joint '{pair}', expected NAME=DEG", file=sys.stderr)
            sys.exit(2)
        name, val = pair.split("=", 1)
        joint_overrides[name] = float(val)  # degrees; build_scene will convert

    scene, fused = build_scene_from_urdf(
        robot, joint_overrides, segments=args.segments, sphere_subdiv=args.sphere_subdiv, fuse=args.fuse, center=args.center
    )

    # Decide output
    if args.output is None:
        args.output = args.input.with_suffix("." + args.format)

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Apply final scale if any
    if args.scale != 1.0:
        if fused is not None:
            fused.apply_scale(args.scale)
        for geom in scene.geometry.values():
            geom.apply_scale(args.scale)

    # Export
    if args.format in ("glb", "gltf"):
        data = scene.export(file_type=args.format)
        if isinstance(data, (bytes, bytearray)):
            out_path.write_bytes(data)
        else:
            out_path.write_text(data)
    elif args.format in ("stl", "obj"):
        mesh = fused
        if mesh is None:
            if len(scene.geometry) == 0:
                print("WARNING: No geometry to export.", file=sys.stderr)
                sys.exit(0)
            mesh = trimesh.util.concatenate(list(scene.geometry.values()))
        mesh.export(out_path, file_type=args.format)
    else:
        print(f"ERROR: Unsupported format: {args.format}", file=sys.stderr)
        sys.exit(2)

    print(f"Wrote {out_path}")

if __name__ == "__main__":
    main()
