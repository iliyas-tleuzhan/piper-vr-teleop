"""Dual Piper dry-run skeleton.

Single-arm teleop is the validated first-working path. This module keeps the
dual-arm configuration shape compiling and exercises separate controller/CAN
assignments without requiring two real arms.
"""

from __future__ import annotations

import argparse

import yaml


def main() -> int:
    parser = argparse.ArgumentParser(description="Dual Piper Quest teleoperation skeleton")
    parser.add_argument("--config", default="configs/dual_piper.yaml")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}

    if not args.dry_run:
        print("Real dual-arm hardware control is unverified in this repository. Run with --dry-run first.")
        return 2

    for arm_name in ("left", "right"):
        arm = {**config, **(config.get(arm_name) or {})}
        print(
            f"{arm_name}: can={arm.get('can')} side={arm.get('side')} "
            f"deadman={arm.get('deadman_button')} calibrate={arm.get('calibrate_button')}"
        )
    print("Dual-arm dry-run skeleton OK. Real hardware sections are intentionally marked unverified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
