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
- The default now uses previous-frame controller deltas in an HMD-yaw control frame, deadband, filtering, stop/backlog cancellation, and wrist rotation under the main grip deadman.

## Relative Delta Mapping

In `mapping_mode: "relative_delta"`, the controller contributes only motion since the previous frame:

```text
u = [dx, dy, dz, droll, dpitch, dyaw]
dq = relative_gain_matrix @ u
target = last_command + dq
```

Translation and rotation deadbands remove jitter. If the controller stops for `settle_frames_on_stop`, the session stops advancing targets and can sync back to measured joints. Controller translation drives joints 1-3, controller rotation drives joints 4-6, and `rightTrig` is not required by default. Wrist safety comes from main-grip clutching, small gains, deadband, filtering, joint limits, and wrist speed limits.

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
python3 scripts/record_quest_axis_movements.py --side right
python3 scripts/generate_relative_gain_config.py
python3 scripts/predict_piper_motion_from_controller.py --config configs/generated_relative_mapping.yaml
scripts/setup_can.sh can0 1000000
python3 scripts/print_piper_joints.py --can can0 --debug-feedback
python3 scripts/test_piper_joint.py --can can0 --joint 2 --delta-deg 5 --duration 2 --rate 50
python3 -m piper_vr.vr_teleop --config configs/generated_relative_mapping.yaml --debug-motion
```

The checked-in gain matrix is a strong temporary guess. Prefer the generated mapping because it measures which Quest delta channel changes for physical up/down/left/right/forward/back and roll/pitch/yaw. `--debug-motion` prints raw controller XYZ, translation deltas, raw/used rotation deltas, translation/wrist/full `dq`, safe targets, measured joints, and tracking error.

For wrist-only validation without a robot:

```bash
python3 scripts/test_controller_rotation_to_wrist.py
python3 scripts/test_controller_rotation_to_wrist.py --robot --can can0
```

Speed profiles are available when a slower first run is needed:

```bash
python3 -m piper_vr.vr_teleop --profile safe
python3 -m piper_vr.vr_teleop --profile normal
python3 -m piper_vr.vr_teleop --profile fast
```

For slow real-hardware tuning without the human-arm inference layer:

```bash
python3 scripts/tune_joint_mapping_vr.py --can can0 --dry-run
python3 scripts/tune_joint_mapping_vr.py --can can0 --max-speed-deg-s 5
```

Joint mimic logs JSONL files under `logs/joint_mimic/` when `--log` is passed.

## Endpoint and External IK

`endpoint_firmware` remains available for fallback/debug and uses `EndPoseCtrl`. `external_ik` is the middle ground for users who want an endpoint target plus predictable posture: host-side IK solves joint angles with a posture objective and sends `JointCtrl`.
