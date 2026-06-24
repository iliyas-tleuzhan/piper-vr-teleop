#!/usr/bin/env python3
"""Print Quest controller transforms and buttons without moving the robot."""

from __future__ import annotations

import argparse
import time

from piper_vr.quest_reader import QuestReader


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--quest-ip")
    parser.add_argument("--hz", type=float, default=5.0)
    args = parser.parse_args()
    reader = QuestReader(ip_address=args.quest_ip)
    period = 1.0 / args.hz
    while True:
        transforms, buttons = reader.poll()
        print(f"transforms={list(transforms.keys())} buttons={buttons}")
        time.sleep(period)


if __name__ == "__main__":
    raise SystemExit(main())
