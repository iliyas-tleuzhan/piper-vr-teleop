#!/usr/bin/env python3
"""Record an offline visual of a hand-held Quest controller driving the Piper URDF."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from piper_vr.piper_kinematics import PiperKinematics  # noqa: E402
from simulate_piper_urdf import read_stl, transform_triangles  # noqa: E402


def _line(axis, points: np.ndarray, *, color: str, width: float, style: str = "-"):
    return axis.plot(points[:, 0], points[:, 1], points[:, 2], color=color, linewidth=width, linestyle=style)[0]


def _controller_and_hand(axis, position: np.ndarray, yaw: float):
    """Render a proportioned Quest-style controller held in an articulated hand."""
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    def place(points: list[list[float]]) -> np.ndarray:
        return np.asarray(points) @ rotation.T + position

    artists = []

    def ellipsoid(center: list[float], radius: list[float], color: str, resolution: int = 10) -> None:
        u, v = np.mgrid[0 : 2 * np.pi : complex(resolution * 2), 0 : np.pi : complex(resolution)]
        local = np.stack((radius[0] * np.cos(u) * np.sin(v), radius[1] * np.sin(u) * np.sin(v), radius[2] * np.cos(v)), axis=-1)
        world = local @ rotation.T + place([center])[0]
        artists.append(axis.plot_surface(world[:, :, 0], world[:, :, 1], world[:, :, 2], color=color, linewidth=0, shade=True))

    # Quest 3 Touch-style body, handle, trigger, thumbstick, buttons, and halo.
    ellipsoid([0, 0, -.01], [.052, .034, .065], "#E8EBEF")
    ellipsoid([0, .002, -.105], [.031, .027, .085], "#D8DDE3")
    ellipsoid([.026, -.033, -.02], [.010, .008, .032], "#1E293B", 7)  # trigger
    ellipsoid([-.012, -.034, .025], [.014, .008, .014], "#334155", 7)  # thumbstick
    ellipsoid([.018, -.034, .025], [.007, .006, .007], "#5D6675", 7)
    ellipsoid([.035, -.034, .008], [.007, .006, .007], "#5D6675", 7)
    u, v = np.mgrid[0 : 2 * np.pi : 32j, 0 : 2 * np.pi : 8j]
    major, minor = .069, .006
    local_ring = np.stack(((major + minor * np.cos(v)) * np.cos(u), minor * np.sin(v), .070 + (major + minor * np.cos(v)) * np.sin(u)), axis=-1)
    ring = local_ring @ rotation.T + position
    artists.append(axis.plot_surface(ring[:, :, 0], ring[:, :, 1], ring[:, :, 2], color="#B9F0FF", linewidth=0, shade=True))

    # Hand: palm/wrist volumes and finger bones with rounded knuckles.
    ellipsoid([0, .045, -.115], [.065, .034, .075], "#E7A17D")
    ellipsoid([0, .062, -.205], [.042, .028, .075], "#D99170")
    for offset in (-.037, -.012, .012, .037):
        finger = place([[offset, .035, -.10], [offset * .95, .008, -.045], [offset * .78, -.005, .008]])
        artists.append(_line(axis, finger, color="#F0B18C", width=7))
        for point in finger[1:]:
            artists.append(axis.scatter(*point, s=22, c="#F5BE9A", depthshade=True))
    thumb = place([[-.060, .030, -.13], [-.071, -.014, -.06], [-.045, -.030, -.01]])
    artists.append(_line(axis, thumb, color="#F0B18C", width=8))
    for point in thumb[1:]:
        artists.append(axis.scatter(*point, s=26, c="#F5BE9A", depthshade=True))
    return artists


def main() -> int:
    parser = argparse.ArgumentParser(description="Record a 15 s offline Quest-to-Piper visual simulation")
    parser.add_argument("--urdf", default="third_party/agx_arm_urdf/piper/urdf/piper_description.urdf")
    parser.add_argument("--output", default="assets/piper_vr_teleop_demo.gif")
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--triangles-per-link", type=int, default=100)
    args = parser.parse_args()
    if args.seconds < 15:
        parser.error("The demo must be at least 15 seconds.")

    model = PiperKinematics(args.urdf)
    mesh_dir = Path(args.urdf).resolve().parents[1] / "meshes"
    meshes = {"base_link": read_stl(mesh_dir / "base_link.stl", args.triangles_per_link)}
    meshes.update({f"link{index}": read_stl(mesh_dir / f"link{index}.stl", args.triangles_per_link) for index in range(1, 7)})
    # GIF stores delays in 10 ms ticks. Pillow rounds 8 FPS to 120 ms rather
    # than 125 ms, so calculate against that effective duration and guarantee
    # the requested minimum playback time.
    effective_frame_ms = max(10, (1000 // args.fps // 10) * 10)
    frames = int(np.ceil(args.seconds * 1000 / effective_frame_ms))
    center = (model.lower + model.upper) / 2
    amplitude = .24 * (model.upper - model.lower) / 2
    phase = np.linspace(0.0, np.pi, 6)
    timeline = np.linspace(0.0, 2 * np.pi, frames, endpoint=False)
    trajectory = np.array([center + amplitude * np.sin(time_value + phase) for time_value in timeline])
    assert np.all(trajectory >= model.lower) and np.all(trajectory <= model.upper)

    figure = plt.figure(figsize=(8, 7))
    figure.patch.set_facecolor("#F7FAFC")
    axis = figure.add_subplot(111, projection="3d")
    axis.set_title("Offline Meta Quest 3 → Piper VR teleoperation simulation", pad=16)
    axis.set_xlabel("X (m)"); axis.set_ylabel("Y (m)"); axis.set_zlabel("Z (m)")
    axis.set_box_aspect((1.3, 1.0, 1.0))
    axis.set_xlim(-.65, .65); axis.set_ylim(-.65, .65); axis.set_zlim(0, .8)
    arm_artists = []
    hand_artists = []

    def draw(frame: int):
        nonlocal arm_artists, hand_artists
        for artist in arm_artists + hand_artists:
            artist.remove()
        arm_artists, hand_artists = [], []
        transforms = model.link_transforms(trajectory[frame])
        transforms["base_link"] = np.eye(4)
        for name, triangles in meshes.items():
            collection = Poly3DCollection(
                transform_triangles(triangles, transforms[name]),
                facecolors="#376FA6" if name != "base_link" else "#213B55",
                edgecolors="none",
                linewidths=0,
            )
            axis.add_collection3d(collection)
            arm_artists.append(collection)
        # Add a clear URDF joint-chain silhouette over the mesh decimation.
        joint_points = np.array([transforms["base_link"][:3, 3]] + [transforms[f"link{i}"][:3, 3] for i in range(1, 7)])
        arm_artists.append(_line(axis, joint_points, color="#174A75", width=5))
        endpoint = transforms["link6"][:3, 3]
        # This is the corresponding controller motion in a separate left-side
        # demonstration volume. Its translation follows the endpoint, scaled down.
        controller = np.array([-.42, endpoint[1] * .7, .48 + (endpoint[2] - .30) * .7])
        hand_artists = _controller_and_hand(axis, controller, float(trajectory[frame, 0]))
        label = axis.text2D(.02, .03, f"t = {frame / args.fps:04.1f} s  |  controller (left) → Piper URDF (right)", transform=axis.transAxes)
        hand_artists.append(label)
        return arm_artists + hand_artists

    animation = FuncAnimation(figure, draw, frames=frames, interval=1000 / args.fps, blit=False)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=args.fps), dpi=105)
    plt.close(figure)
    print(f"Wrote {output}: {frames} frames at {args.fps} fps ({frames / args.fps:.1f} seconds)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
