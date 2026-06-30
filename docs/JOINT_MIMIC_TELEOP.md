# Joint Mimic Teleop

`joint_mimic` is the primary mode for this project. It does not send `EndPoseCtrl` as the main command. The host computes all six Piper joint targets and sends them together with `JointCtrl`. The default mapping mode is `relative_delta`.

## Why This Exists

Endpoint control only says "put the gripper here." Piper firmware can choose any valid internal joint configuration. That is useful for simple endpoint movement, but it does not make the arm mimic the operator shoulder, elbow, and wrist posture.

Joint mimic uses a practical inferred human-arm model:

- HMD/head pose can estimate torso/head frame when available.
- Right controller pose estimates hand/wrist pose.
- Shoulder position is estimated from HMD or fixed during calibration.
- Elbow position is inferred from shoulder-to-hand geometry, arm lengths, previous elbow state, and optional joystick swivel.
- Wrist orientation comes from controller orientation.

This is approximate because Quest does not directly track the shoulder or elbow, but it is deterministic, smooth, debuggable, and tunable. The important safety property is that the host computes six joint targets and sends `JointCtrl`; Piper firmware endpoint IK is not the main path.

## Why the old movement felt wrong

- Raw controller Euler angles caused wrist twisting.
- Absolute pose-delta mapping made directions confusing when the operator/control frame was not calibrated.
- Rate limiting could leave Piper chasing an old target after the controller stopped.
- The default now uses previous-frame controller deltas in an HMD-yaw control frame, deadband, stop/backlog cancellation, and disabled wrist rotation.

## Relative Delta Mapping

In `mapping_mode: "relative_delta"`, the controller contributes only motion since the previous frame:

```text
u = [dx, dy, dz, droll, dpitch, dyaw]
dq = relative_gain_matrix @ u
target = last_command + dq
```

Translation and rotation deadbands remove jitter. If the controller stops for `settle_frames_on_stop`, the session stops advancing targets and can sync back to measured joints. Wrist rotation columns are zero in the default config, and `wrist_rotation_enabled` is false until tuning proves the axes are correct.

## Pose-Delta Mapping

Calibration stores:

- measured Piper joints as `robot_home_joints_deg`
- inferred human posture channels as `human_home_vector_deg`
- controller home and shoulder estimate

The older optional `mapping_mode: "pose_delta"` computes:

```python
human_delta_deg = human_vector_deg - human_home_vector_deg
target_joints_deg = robot_home_joints_deg + offsets_deg + signs * gains * human_delta_deg
```

This mode remains available for debugging, but it is no longer the default because real hardware testing showed direction and backlog problems.

Pressing `A` calibrates the initial relationship. Every new `rightGrip` press creates a fresh clutch anchor from the current human posture and measured robot joints. If the controller moves while the deadman is released, the next grip press should produce zero human delta and no robot jump.

## Safety Rules

- No motion before calibration.
- No motion unless the deadman is held.
- Deadman press creates a clutch anchor.
- Deadman release holds measured current joint pose.
- Stale tracking holds measured current joint pose and requires deadman release/repress.
- Joint targets are clamped to documented Piper limits.
- Joint speeds are rate-limited per joint.
- Ctrl+C commands a joint hold.
- Real calibration is refused if measured joint feedback is unavailable.
- Never run real joint mimic if `print_piper_joints.py` cannot read joints.

## Tune First

Run in this order:

```bash
git pull origin main
export PYTHONPATH=$PWD:$HOME/Iliyas/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH
python3 scripts/check_quest_transport.py --seconds 10
python3 scripts/calibrate_relative_mapping.py --side right --calibrate-button A
python3 scripts/inspect_piper_sdk_feedback.py --can can0
python3 scripts/print_piper_joints.py --can can0 --debug-feedback
python3 scripts/test_piper_joint.py --can can0 --joint 2 --delta-deg 3 --duration 3 --rate 50
python3 scripts/tune_joint_mapping_vr.py --can can0 --dry-run
python3 -m piper_vr.vr_teleop
```

Start by tuning `joint_mimic.signs`, then `gains`, then `offsets_deg`. Keep `speed_percent` and `max_joint_speed_deg_s` low until every axis direction is verified.

For slow real-hardware tuning without the human-arm inference layer:

```bash
python3 scripts/tune_joint_mapping_vr.py --can can0 --dry-run
python3 scripts/tune_joint_mapping_vr.py --can can0 --max-speed-deg-s 5
```

Joint mimic logs JSONL files under `logs/joint_mimic/` when `--log` is passed.

## Endpoint and External IK

`endpoint_firmware` remains available for fallback/debug and uses `EndPoseCtrl`. `external_ik` is the middle ground for users who want an endpoint target plus predictable posture: host-side IK solves joint angles with a posture objective and sends `JointCtrl`.
