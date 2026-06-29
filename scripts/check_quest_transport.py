#!/usr/bin/env python3
"""Check Quest transport samples without starting robot control."""

from __future__ import annotations

import argparse
import time

from piper_vr.quest_reader import QuestReader


def main() -> int:
    parser = argparse.ArgumentParser(description="Check Quest transport samples")
    parser.add_argument("--transport", default="adb_logcat")
    parser.add_argument("--connection", default="usb", choices=("usb", "wireless"))
    parser.add_argument("--quest-ip")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()

    reader = QuestReader(
        transport=args.transport,
        connection=args.connection,
        ip_address=args.quest_ip,
        simulate_on_missing=args.simulate,
    )
    diagnostics = reader.diagnostics()
    if diagnostics is not None:
        print(f"transport={diagnostics.transport}")
        print(f"module_path={diagnostics.module_path}")
        print(f"connection={diagnostics.connection}")
        print(f"quest_ip={diagnostics.ip_address}")
        print(f"package={diagnostics.package}")
        print(f"log_tag={diagnostics.log_tag}")

    deadline = time.monotonic() + args.seconds
    last = None
    while time.monotonic() < deadline:
        sample = reader.get_sample()
        if sample is not None:
            last = sample
            print(f"sample age_s={sample.age_s:.3f} buttons={sorted(sample.buttons.keys())}")
            for side, transform in sorted(sample.transforms_openxr.items()):
                print(f"{side}_xyz={transform[:3, 3].round(4).tolist()}")
        time.sleep(0.5)

    reader.stop()
    if last is None:
        print("No samples received. Check adb devices, headset USB authorization, and the Quest app.")
        return 1
    print("Quest transport OK.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
