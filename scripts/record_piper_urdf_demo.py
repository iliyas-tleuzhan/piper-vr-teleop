#!/usr/bin/env python3
"""Run an offline joint-limit/FK/IK test and record the Piper mesh motion as a GIF."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from piper_vr.piper_kinematics import PiperKinematics  # noqa: E402
from simulate_piper_urdf import read_stl, transform_triangles  # noqa: E402


def matrix_to_rpy_deg(rotation: np.ndarray) -> np.ndarray:
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    return np.degrees([
        np.arctan2(rotation[2, 1], rotation[2, 2]),
        pitch,
        np.arctan2(rotation[1, 0], rotation[0, 0]),
    ])


def main() -> int:
    parser = argparse.ArgumentParser(description="Record an offline Piper URDF mesh test")
    parser.add_argument("--urdf", default="third_party/agx_arm_urdf/piper/urdf/piper_description.urdf")
    parser.add_argument("--output", default="outputs/piper_urdf_motion_test.gif")
    parser.add_argument("--frames", type=int, default=32)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--triangles-per-link", type=int, default=500)
    args = parser.parse_args()

    model = PiperKinematics(args.urdf)
    mesh_dir = Path(args.urdf).resolve().parents[1] / "meshes"
    meshes = {"base_link": read_stl(mesh_dir / "base_link.stl", args.triangles_per_link)}
    meshes.update({f"link{index}": read_stl(mesh_dir / f"link{index}.stl", args.triangles_per_link) for index in range(1, 7)})

    # Motion remains 30% inside every joint's legal range, so no frame can
    # approach the hard stops. Different phase offsets exercise all six axes.
    center = (model.lower + model.upper) / 2
    amplitude = 0.30 * (model.upper - model.lower) / 2
    phase = np.linspace(0.0, np.pi, 6)
    time_values = np.linspace(0.0, 2 * np.pi, args.frames, endpoint=False)
    trajectory = np.array([center + amplitude * np.sin(time_value + phase) for time_value in time_values])

    checked = 0
    for q in trajectory:
        assert np.all(q >= model.lower) and np.all(q <= model.upper)
        position, rotation = model.forward(q)
        result = model.solve(position, matrix_to_rpy_deg(rotation), q)
        assert result.success, result
        checked += 1

    figure = plt.figure(figsize=(7, 7))
    figure.patch.set_facecolor("white")
    axis = figure.add_subplot(111, projection="3d")
    axis.set_title(f"Piper URDF motion test — {checked} FK/IK frames within joint limits")
    axis.set_xlabel("X (m)"); axis.set_ylabel("Y (m)"); axis.set_zlabel("Z (m)")
    axis.set_box_aspect((1, 1, 1))
    axis.set_xlim(-0.55, 0.55); axis.set_ylim(-0.55, 0.55); axis.set_zlim(0.0, 0.75)
    artists: list[Poly3DCollection] = []

    def draw(frame: int) -> list[Poly3DCollection]:
        nonlocal artists
        for artist in artists:
            artist.remove()
        artists = []
        transforms = model.link_transforms(trajectory[frame])
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
        return artists

    animation = FuncAnimation(figure, draw, frames=args.frames, interval=1000 / args.fps, blit=False)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.suffix.lower() == ".gif":
        writer = PillowWriter(fps=args.fps)
    elif output.suffix.lower() == ".mp4":
        try:
            import imageio_ffmpeg
        except ImportError as exc:
            raise RuntimeError("MP4 output requires `pip install imageio-ffmpeg`.") from exc
        plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
        writer = FFMpegWriter(fps=args.fps, codec="libx264", bitrate=1800)
    else:
        parser.error("--output must end in .gif or .mp4")
    animation.save(output, writer=writer, dpi=110)
    plt.close(figure)
    print(f"URDF test passed: {checked} frames checked; wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
