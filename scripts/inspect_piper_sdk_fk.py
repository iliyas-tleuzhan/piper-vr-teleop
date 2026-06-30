#!/usr/bin/env python3
"""Inspect installed piper_sdk forward kinematics API and result shapes."""

from __future__ import annotations

import argparse
import inspect

import numpy as np

from piper_vr.piper_official_kinematics import _parse_sdk_fk_result, parse_rpy_to_degrees, parse_xyz_to_meters


POSES_DEG = [
    [0, 0, 0, 0, 0, 0],
    [0, 90, -90, 0, 0, 0],
    [0, 45, -45, 0, 0, 0],
    [10, 60, -60, 20, 0, 0],
]


def _public_methods(obj) -> list[str]:
    return [name for name in dir(obj) if not name.startswith("_") and callable(getattr(obj, name, None))]


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect piper_sdk C_PiperForwardKinematics API")
    parser.parse_args()
    try:
        from piper_sdk.kinematics import C_PiperForwardKinematics
    except Exception as exc:
        print(f"Could not import piper_sdk.kinematics.C_PiperForwardKinematics: {exc!r}")
        return 1

    fk = C_PiperForwardKinematics()
    print(f"FK object: {fk!r}")
    print("Public methods:")
    for name in _public_methods(fk):
        method = getattr(fk, name)
        try:
            signature = str(inspect.signature(method))
        except Exception:
            signature = "(signature unavailable)"
        print(f"  {name}{signature}")

    for pose in POSES_DEG:
        print(f"\nPose deg: {pose}")
        for name in ("CalFK", "cal_fk", "GetFK", "get_fk"):
            method = getattr(fk, name, None)
            if method is None:
                continue
            for call_name, args in (("list", (pose,)), ("expanded", tuple(pose))):
                try:
                    result = method(*args)
                except Exception as exc:
                    print(f"  {name}({call_name}) failed: {exc!r}")
                    continue
                print(f"  {name}({call_name}) -> type={type(result)} repr={result!r}")
                parsed = _parse_sdk_fk_result(result)
                if parsed is None:
                    print("    parsed: None")
                    continue
                xyz_m, rotation = parsed
                print(f"    parsed xyz_m={np.asarray(xyz_m).round(6).tolist()}")
                print(f"    parsed rotation=\n{np.asarray(rotation).round(6)}")
                break

    print("\nUnit parser examples:")
    print(f"  xyz 0.001mm -> {parse_xyz_to_meters([499567, 0, 409863]).round(6).tolist()}")
    print(f"  xyz mm      -> {parse_xyz_to_meters([499.567, 0, 409.863]).round(6).tolist()}")
    print(f"  xyz m       -> {parse_xyz_to_meters([0.499567, 0, 0.409863]).round(6).tolist()}")
    print(f"  rpy 0.001deg -> {parse_rpy_to_degrees([1000, 2000, 3000]).round(6).tolist()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
