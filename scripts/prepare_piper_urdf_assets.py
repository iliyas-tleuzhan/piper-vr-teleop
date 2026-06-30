#!/usr/bin/env python3
"""Copy official Piper URDF and meshes into the Unity Quest visualization app."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = REPO_ROOT / "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf"
DEFAULT_MESH_DIR = REPO_ROOT / "third_party/agx_arm_urdf/piper/meshes"
STREAMING_DIR = REPO_ROOT / "quest_viz_app/Assets/StreamingAssets/PiperURDF"
MODEL_DIR = REPO_ROOT / "quest_viz_app/Assets/Models/Piper"


def display_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix().encode("ascii", errors="backslashreplace").decode("ascii")


def copy_tree_contents(src: Path, dst: Path) -> int:
    copied = 0
    for path in src.rglob("*"):
        if path.is_dir():
            continue
        rel = path.relative_to(src)
        target = dst / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        copied += 1
    return copied


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare Piper URDF and mesh assets for Unity")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--mesh-dir", type=Path, default=DEFAULT_MESH_DIR)
    args = parser.parse_args()

    urdf = args.urdf.resolve()
    mesh_dir = args.mesh_dir.resolve()
    if not urdf.exists():
        print(f"ERROR: Piper URDF not found at {display_path(urdf)}")
        print("Fetch the official AgileX Piper ROS/URDF assets into third_party/agx_arm_urdf, then rerun this script.")
        print("Expected path: third_party/agx_arm_urdf/piper/urdf/piper_description.urdf")
        return 1
    if not mesh_dir.exists():
        print(f"ERROR: Piper mesh directory not found at {display_path(mesh_dir)}")
        print("Fetch the official AgileX Piper meshes; the Unity app should prefer official meshes over fallback geometry.")
        return 1

    STREAMING_DIR.mkdir(parents=True, exist_ok=True)
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copy2(urdf, STREAMING_DIR / "piper_description.urdf")
    mesh_count = copy_tree_contents(mesh_dir, MODEL_DIR)
    print(f"Copied URDF to {display_path(STREAMING_DIR / 'piper_description.urdf')}")
    print(f"Copied {mesh_count} Piper mesh files to {display_path(MODEL_DIR)}")
    print("Official Piper meshes are available. Unity fallback geometry should only be used if a mesh fails to load.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
