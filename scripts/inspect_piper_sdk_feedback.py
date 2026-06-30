#!/usr/bin/env python3
"""Inspect Piper SDK feedback methods and returned object shapes."""

from __future__ import annotations

import argparse
from typing import Any

from piper_vr.piper_driver import PiperDriver


def _simple(value: Any) -> bool:
    return isinstance(value, (str, int, float, bool, type(None), tuple, list))


def print_attrs(obj: Any, *, depth: int = 0, max_depth: int = 2, prefix: str = "") -> None:
    if depth > max_depth:
        return
    for name in dir(obj):
        if name.startswith("_"):
            continue
        try:
            value = getattr(obj, name)
        except Exception as exc:
            print(f"{prefix}{name}: <error {exc!r}>")
            continue
        if callable(value):
            continue
        print(f"{prefix}{name}: {value!r}")
        if not _simple(value):
            print_attrs(value, depth=depth + 1, max_depth=max_depth, prefix=prefix + "  ")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed-percent", type=int, default=5)
    args = parser.parse_args()

    driver = PiperDriver(can=args.can, speed_percent=args.speed_percent)
    driver.connect(initial_mode=None)
    if driver.arm is None:
        raise RuntimeError("Piper SDK interface was not initialized")

    keywords = ("Joint", "EndPose", "Status", "Motor")
    print("Methods containing Joint/EndPose/Status/Motor:")
    for name in sorted(dir(driver.arm)):
        if any(keyword in name for keyword in keywords):
            print(f"  {name}")

    getters = ("GetArmJointMsgs", "GetArmJointCtrl", "GetArmLowSpdInfoMsgs", "GetArmStatus", "GetArmEndPoseMsgs")
    for getter in getters:
        method = getattr(driver.arm, getter, None)
        print(f"\n== {getter} ==")
        if method is None:
            print("missing")
            continue
        try:
            value = method()
        except Exception as exc:
            print(f"error: {exc!r}")
            continue
        print(f"repr: {value!r}")
        print(f"dir: {dir(value)}")
        print_attrs(value)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
