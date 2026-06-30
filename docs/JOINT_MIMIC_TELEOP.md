# Joint Mimic Teleop

`joint_mimic` is the primary mode for this project. It does not send `EndPoseCtrl` as the main command. The host computes all six Piper joint targets and sends them together with `JointCtrl`.

## Why This Exists

Endpoint control only says "put the gripper here." Piper firmware can choose any valid internal joint configuration. That is useful for simple endpoint movement, but it does not make the arm mimic the operator shoulder, elbow, and wrist posture.

Joint mimic uses a practical inferred human-arm model:

- HMD/head pose can estimate torso/head frame when available.
- Right controller pose estimates hand/wrist pose.
- Shoulder position is estimated from HMD or fixed during calibration.
- Elbow position is inferred from shoulder-to-hand geometry, arm lengths, previous elbow state, and optional joystick swivel.
- Wrist orientation comes from controller orientation.

This is approximate because Quest does not directly track the shoulder or elbow, but it is deterministic, smooth, debuggable, and tunable.

## Safety Rules

- No motion before calibration.
- No motion unless the deadman is held.
- Deadman press creates a clutch anchor.
- Deadman release holds measured current joint pose.
- Stale tracking holds measured current joint pose and requires deadman release/repress.
- Joint targets are clamped to documented Piper limits.
- Joint speeds are rate-limited per joint.
- Ctrl+C commands a joint hold.

## Tune First

Run in this order:

```bash
python3 scripts/test_piper_joint.py --can can0 --joint 2 --delta-deg 3
python3 scripts/debug_human_arm_model.py --side right
python3 -m piper_vr.vr_teleop --config configs/single_piper.yaml --control-mode joint_mimic --dry-run --verbose
python3 -m piper_vr.vr_teleop --config configs/single_piper.yaml --control-mode joint_mimic --can can0 --speed-percent 5 --verbose
```

Start by tuning `joint_mimic.signs`, then `gains`, then `offsets_deg`. Keep `speed_percent` and `max_joint_speed_deg_s` low until every axis direction is verified.

## Endpoint and External IK

`endpoint_firmware` remains available for fallback/debug and uses `EndPoseCtrl`. `external_ik` is the middle ground for users who want an endpoint target plus predictable posture: host-side IK solves joint angles with a posture objective and sends `JointCtrl`.
