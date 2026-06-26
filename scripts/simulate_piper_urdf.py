#!/usr/bin/env python3
"""Render the Piper URDF meshes with interactive joint sliders; never touches hardware."""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import sys

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from piper_vr.piper_kinematics import PiperKinematics  # noqa: E402


def read_stl(path: Path, max_triangles: int) -> np.ndarray:
    """Read binary or ASCII STL into an optionally decimated (N, 3, 3) array."""
    data = path.read_bytes()
    triangle_count = struct.unpack("<I", data[80:84])[0] if len(data) >= 84 else -1
    if triangle_count >= 0 and len(data) == 84 + 50 * triangle_count:
        dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
        triangles = np.frombuffer(data, dtype=dtype, offset=84, count=triangle_count)["vertices"].astype(float)
    else:
        values = []
        for line in data.decode("utf-8", errors="ignore").splitlines():
            fields = line.split()
            if len(fields) == 4 and fields[0].lower() == "vertex":
                values.append([float(value) for value in fields[1:]])
        triangles = np.asarray(values, dtype=float).reshape(-1, 3, 3)
    stride = max(1, int(np.ceil(len(triangles) / max_triangles)))
    return triangles[::stride]


def read_visual_mesh(path: Path, max_triangles: int) -> np.ndarray:
    """Read a URDF visual mesh, including AgileX's detailed Collada DAE files."""
    if path.suffix.lower() == ".stl":
        return read_stl(path, max_triangles)
    if path.suffix.lower() != ".dae":
        raise ValueError(f"Unsupported mesh format: {path}")
    try:
        import trimesh
    except ImportError as exc:
        raise RuntimeError("DAE visual meshes require `pip install trimesh pycollada`.") from exc
    scene = trimesh.load(path, force="scene")
    mesh = trimesh.util.concatenate(tuple(scene.geometry.values()))
    # Sampling arbitrary faces makes a sparse point cloud. Cluster vertices into
    # a regular grid instead: this preserves the visible exterior while reducing
    # the dense CAD visual mesh enough for an animated Matplotlib renderer.
    extent = float(np.max(mesh.bounds[1] - mesh.bounds[0]))
    cell_size = extent / max(8.0, np.sqrt(float(max_triangles)))
    keys = np.floor(mesh.vertices / cell_size + 0.5).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    count = int(inverse.max()) + 1
    vertices = np.zeros((count, 3), dtype=float)
    np.add.at(vertices, inverse, mesh.vertices)
    vertices /= np.bincount(inverse, minlength=count)[:, None]
    faces = inverse[mesh.faces]
    valid = (faces[:, 0] != faces[:, 1]) & (faces[:, 0] != faces[:, 2]) & (faces[:, 1] != faces[:, 2])
    faces = faces[valid]
    # Remove coincident triangles created by vertex clustering while keeping the
    # first winding order for shading.
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return vertices[faces[np.sort(first)]]


def transform_triangles(triangles: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return triangles @ transform[:3, :3].T + transform[:3, 3]


def main() -> int:
    parser = argparse.ArgumentParser(description="Interactive, offline Piper URDF mesh simulator")
    parser.add_argument("--urdf", default="third_party/agx_arm_urdf/piper/urdf/piper_description.urdf")
    parser.add_argument("--output", help="save a PNG of the initial pose")
    parser.add_argument("--no-gui", action="store_true", help="render only; requires --output")
    parser.add_argument("--triangles-per-link", type=int, default=1500)
    args = parser.parse_args()
    if args.no_gui and not args.output:
        parser.error("--no-gui requires --output")

    model = PiperKinematics(args.urdf)
    mesh_dir = Path(args.urdf).resolve().parents[1] / "meshes" / "dae"
    meshes = {"base_link": read_visual_mesh(mesh_dir / "base_link.dae", args.triangles_per_link)}
    meshes.update({f"link{index}": read_visual_mesh(mesh_dir / f"link{index}.dae", args.triangles_per_link) for index in range(1, 7)})
    q = (model.lower + model.upper) / 2

    figure = plt.figure(figsize=(10, 9))
    figure.patch.set_facecolor("white")
    axis = figure.add_axes([0.05, 0.26, 0.9, 0.7], projection="3d")
    axis.set_title("Piper URDF simulator — sliders change model joints only")
    axis.set_xlabel("X (m)"); axis.set_ylabel("Y (m)"); axis.set_zlabel("Z (m)")
    axis.set_box_aspect((1, 1, 1))
    axis.set_xlim(-0.65, 0.65); axis.set_ylim(-0.65, 0.65); axis.set_zlim(0.0, 0.85)
    artists: list[Poly3DCollection] = []

    def draw() -> None:
        nonlocal artists
        for artist in artists:
            artist.remove()
        artists = []
        transforms = model.link_transforms(q)
        transforms["base_link"] = np.eye(4)
        for name, triangles in meshes.items():
            collection = Poly3DCollection(
                transform_triangles(triangles, transforms[name]),
                facecolors="#4C78A8" if name != "base_link" else "#2D4059",
                edgecolors="none",
                linewidths=0,
            )
            axis.add_collection3d(collection)
            artists.append(collection)
        figure.canvas.draw_idle()

    sliders = []
    for index, joint in enumerate(model.joints):
        slider_axis = figure.add_axes([0.13, 0.20 - index * 0.029, 0.74, 0.016])
        slider = Slider(slider_axis, joint.name, joint.lower, joint.upper, valinit=q[index], valfmt="%.2f rad")
        sliders.append(slider)

        def update(_: float, joint_index: int = index) -> None:
            q[joint_index] = sliders[joint_index].val
            draw()

        slider.on_changed(update)

    draw()
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output, dpi=180)
        print(f"Wrote {args.output}")
    if not args.no_gui:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
