#!/usr/bin/env python3
"""Convert Piper URDF kinematics into a compact Unity runtime JSON file."""

from __future__ import annotations

import argparse
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = REPO_ROOT / "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf"
DEFAULT_OUTPUT = REPO_ROOT / "quest_viz_app/Assets/StreamingAssets/piper_kinematic_model.json"


def display_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix().encode("ascii", errors="backslashreplace").decode("ascii")


def floats(text: str | None, default: list[float]) -> list[float]:
    if text is None:
        return list(default)
    values = [float(part) for part in text.split()]
    return values if values else list(default)


def mesh_scale(mesh: ET.Element | None) -> list[float]:
    if mesh is None:
        return [1.0, 1.0, 1.0]
    return floats(mesh.attrib.get("scale"), [1.0, 1.0, 1.0])


def unity_mesh_path(filename: str | None) -> str | None:
    if not filename:
        return None
    normalized = filename.replace("\\", "/")
    match = re.search(r"/piper/meshes/(.+)$", normalized)
    if match:
        return f"Models/Piper/{match.group(1)}"
    if normalized.startswith("package://"):
        return f"Models/Piper/{Path(normalized).name}"
    return normalized


def origin_dict(element: ET.Element | None) -> dict[str, list[float]]:
    if element is None:
        return {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
    origin = element.find("origin")
    if origin is None:
        return {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
    return {
        "xyz": floats(origin.attrib.get("xyz"), [0.0, 0.0, 0.0]),
        "rpy": floats(origin.attrib.get("rpy"), [0.0, 0.0, 0.0]),
    }


def convert_urdf(urdf_path: Path) -> dict:
    root = ET.parse(urdf_path).getroot()
    try:
        source_urdf = urdf_path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        source_urdf = urdf_path.as_posix()
    links = []
    for link in root.findall("link"):
        visual = link.find("visual")
        mesh = None if visual is None else visual.find("geometry/mesh")
        links.append(
            {
                "name": link.attrib["name"],
                "visual_origin": origin_dict(visual),
                "visual_mesh": unity_mesh_path(None if mesh is None else mesh.attrib.get("filename")),
                "mesh_scale": mesh_scale(mesh),
                "fallback_geometry": mesh is None,
            }
        )

    joints = []
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        lower = None if limit is None or "lower" not in limit.attrib else math.degrees(float(limit.attrib["lower"]))
        upper = None if limit is None or "upper" not in limit.attrib else math.degrees(float(limit.attrib["upper"]))
        parent = joint.find("parent")
        child = joint.find("child")
        axis = joint.find("axis")
        joints.append(
            {
                "name": joint.attrib["name"],
                "type": joint.attrib.get("type", "fixed"),
                "parent": "" if parent is None else parent.attrib.get("link", ""),
                "child": "" if child is None else child.attrib.get("link", ""),
                "origin": origin_dict(joint),
                "axis": floats(None if axis is None else axis.attrib.get("xyz"), [0.0, 0.0, 1.0]),
                "limit_deg": {"lower": lower, "upper": upper},
                "home_deg": 0.0,
            }
        )

    return {
        "name": root.attrib.get("name", "piper"),
        "source_urdf": source_urdf,
        "units": {"position": "meters", "rotation": "radians", "joint_command": "degrees"},
        "joint_order": [joint["name"] for joint in joints if joint["type"] != "fixed"][:6],
        "links": links,
        "joints": joints,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert Piper URDF to Unity JSON")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    if not args.urdf.exists():
        print(f"ERROR: URDF not found: {display_path(args.urdf)}")
        print("Run scripts/prepare_piper_urdf_assets.py after fetching official AgileX Piper URDF assets.")
        return 1
    model = convert_urdf(args.urdf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(model, indent=2), encoding="utf-8")
    print(f"Wrote {display_path(args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
