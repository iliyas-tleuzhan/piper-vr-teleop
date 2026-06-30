#!/usr/bin/env python3
"""Generate a strong relative gain mapping from guided Quest axis calibration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from piper_vr.relative_calibration import generated_mapping_config


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate configs/generated_relative_mapping.yaml")
    parser.add_argument("--input", default="logs/quest_axis_calibration/latest_axis_calibration.json")
    parser.add_argument("--output", default="configs/generated_relative_mapping.yaml")
    args = parser.parse_args()

    data = json.loads(Path(args.input).read_text(encoding="utf-8"))
    generated = generated_mapping_config(data)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(generated, sort_keys=False), encoding="utf-8")

    print(f"Generated {output}")
    print("joint_mimic.relative_gain_matrix:")
    for row in generated["joint_mimic"]["relative_gain_matrix"]:
        print(f"  {row}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
