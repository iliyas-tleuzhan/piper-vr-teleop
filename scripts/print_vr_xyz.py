#!/usr/bin/env python3
"""Print Quest controller XYZ positions from the selected transport."""

from __future__ import annotations

import argparse
import time

from piper_vr.quest_reader import QuestReader


def main() -> int:
    parser = argparse.ArgumentParser(description="Print Quest controller XYZ positions")
    parser.add_argument("--transport", default="adb_logcat")
    parser.add_argument("--quest-ip")
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()
    reader = QuestReader(transport=args.transport, ip_address=args.quest_ip, simulate_on_missing=args.simulate)
    deadline = None if args.seconds <= 0 else time.monotonic() + args.seconds
    try:
        while deadline is None or time.monotonic() < deadline:
            sample = reader.get_sample()
            if sample is not None:
                for side, transform in sorted(sample.transforms_openxr.items()):
                    print(f"{side}: xyz={transform[:3, 3].round(4).tolist()} age_s={sample.age_s:.3f}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
