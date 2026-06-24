"""Dual Piper entry point placeholder built on the single-arm structure."""

from __future__ import annotations

import argparse


def main() -> int:
    parser = argparse.ArgumentParser(description="Dual Piper Quest teleoperation skeleton")
    parser.add_argument("--config", default="configs/dual_piper.yaml")
    parser.add_argument("--dry-run", action="store_true")
    parser.parse_args()
    print("Dual Piper structure is present, but the maintained first working path is single Piper teleoperation.")
    print("Use configs/dual_piper.yaml as the starting point for assigning left and right arms.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
