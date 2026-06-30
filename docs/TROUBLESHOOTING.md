# Troubleshooting

## No module named oculus_reader

Run:

```bash
scripts/install_oculus_reader.sh
```

Fallback:

```bash
export PYTHONPATH=~/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH
```

## Quest app installed but no samples received

Run:

```bash
scripts/check_adb_logcat.sh
python3 scripts/check_quest_transport.py --transport adb_logcat --seconds 10
```

The diagnostic tag is `wE9ryARX`; the package is `com.rail.oculus.teleop`.

## Joint mimic does not move

Check that you pressed `A` to calibrate, released `rightGrip`, then pressed it again. Run:

```bash
python3 -m piper_vr.vr_teleop --config configs/single_piper.yaml --control-mode joint_mimic --dry-run --verbose
```

Inspect `state`, `calibrated`, `human_vector_deg`, `human_delta_deg`, `target_joints_deg`, `safe_joints_deg`, `action`, and `quest_age_s`.

## Joint feedback is unavailable

Run:

```bash
python3 scripts/inspect_piper_sdk_feedback.py --can can0
python3 scripts/print_piper_joints.py --can can0 --debug-feedback
```

If the SDK getter name differs, inspect the returned feedback object and update `PiperDriver.read_joint_pose()` before real joint mimic. Real calibration is refused without measured joint feedback.

## Endpoint test moves but joint test does not

Run the tiny joint test at low speed:

```bash
python3 scripts/test_piper_joint.py --can can0 --joint 2 --delta-deg 3 --duration 3 --rate 50
```

Check CAN bitrate, power, emergency stop, `piper-sdk`, and whether `JointCtrl` is available in the installed SDK.

## Motion is too fast or joint direction is wrong

Lower `speed_percent`, `joint_mimic.max_joint_speed_deg_s`, or `joint_mimic.gains`. Flip signs in:

```yaml
joint_mimic:
  signs: [1, 1, 1, 1, 1, 1]
```

Tune signs and gains in dry-run before using the real robot.

Use the mapping debugger before real teleop:

```bash
python3 scripts/debug_joint_mimic_mapping.py --side right --calibrate
```

Use manual joint tuning at low speed for hardware sign/gain work:

```bash
python3 scripts/tune_joint_mapping_vr.py --can can0 --joint 1 --max-speed 5
```

## Endpoint axis direction is wrong

Endpoint mode uses `endpoint_firmware.axis_mapping`. Edit the affected axis and test with:

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --control-mode endpoint_firmware --dry-run --verbose
```

## Stale tracking or headset asleep

Wake the headset, keep the controller visible, and re-press the deadman after tracking returns. Stale samples intentionally hold position.
